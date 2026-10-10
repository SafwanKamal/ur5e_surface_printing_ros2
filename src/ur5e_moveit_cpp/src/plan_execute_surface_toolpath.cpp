#include <ament_index_cpp/get_package_share_directory.hpp>

#include <builtin_interfaces/msg/duration.hpp>
#include <controller_manager_msgs/srv/list_controllers.hpp>
#include <geometry_msgs/msg/pose.hpp>

#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit/robot_state/robot_state.hpp>
#include <moveit_msgs/action/execute_trajectory.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <moveit_msgs/srv/get_position_ik.hpp>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rclcpp/executors/single_threaded_executor.hpp>

#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <syringe_interfaces/srv/set_flow.hpp>

#include <tf2/LinearMath/Quaternion.hpp>
#include <tf2/LinearMath/Transform.hpp>
#include <tf2/LinearMath/Vector3.hpp>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <fstream>
#include <future>
#include <map>
#include <memory>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

using namespace std::chrono_literals;

namespace
{

constexpr double kMillimetersToMeters = 0.001;
constexpr double kPi = 3.14159265358979323846;

struct CsvToolpathPoint
{
  int line_id;
  int point_index;
  geometry_msgs::msg::Pose object_pose;
};

std::vector<std::string> splitCsvLine(
  const std::string& line)
{
  std::vector<std::string> values;
  std::stringstream stream(line);
  std::string value;

  while (std::getline(stream, value, ','))
  {
    values.push_back(value);
  }

  return values;
}

double degreesToRadians(double degrees)
{
  return degrees * kPi / 180.0;
}

double durationToSeconds(
  const builtin_interfaces::msg::Duration& duration)
{
  return
    static_cast<double>(duration.sec) +
    static_cast<double>(duration.nanosec) * 1e-9;
}

std::map<std::string, double> resolveCollisionFreeIk(
  const rclcpp::Node::SharedPtr& reporting_node,
  const std::string& planning_group,
  const std::string& planning_frame,
  const std::string& tcp_link,
  const geometry_msgs::msg::Pose& target_pose,
  const moveit::core::RobotState& seed_state,
  const std::vector<std::string>& active_joint_names,
  double timeout_seconds,
  int line_id)
{
  auto ik_node = rclcpp::Node::make_shared(
    "saddle_approach_ik_" + std::to_string(line_id));

  auto client = ik_node->create_client<
    moveit_msgs::srv::GetPositionIK>("/compute_ik");

  if (!client->wait_for_service(5s))
  {
    throw std::runtime_error(
      "MoveIt /compute_ik service is unavailable");
  }

  auto request = std::make_shared<
    moveit_msgs::srv::GetPositionIK::Request>();

  request->ik_request.group_name = planning_group;
  request->ik_request.avoid_collisions = true;
  request->ik_request.ik_link_name = tcp_link;
  request->ik_request.pose_stamped.header.frame_id =
    planning_frame;
  request->ik_request.pose_stamped.pose = target_pose;

  request->ik_request.robot_state.is_diff = true;
  request->ik_request.robot_state.joint_state.name =
    active_joint_names;
  seed_state.copyJointGroupPositions(
    planning_group,
    request->ik_request.robot_state.joint_state.position);

  const auto timeout_nanoseconds =
    static_cast<std::int64_t>(
      std::llround(timeout_seconds * 1e9));

  request->ik_request.timeout.sec =
    static_cast<std::int32_t>(
      timeout_nanoseconds / 1000000000LL);
  request->ik_request.timeout.nanosec =
    static_cast<std::uint32_t>(
      timeout_nanoseconds % 1000000000LL);

  auto future = client->async_send_request(request);

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(ik_node);

  const auto deadline =
    std::chrono::steady_clock::now() +
    std::chrono::duration_cast<
      std::chrono::steady_clock::duration>(
        std::chrono::duration<double>(timeout_seconds + 2.0));

  while (
    rclcpp::ok() &&
    future.wait_for(0s) != std::future_status::ready &&
    std::chrono::steady_clock::now() < deadline)
  {
    executor.spin_some(20ms);
  }

  executor.remove_node(ik_node);

  if (future.wait_for(0s) != std::future_status::ready)
  {
    RCLCPP_WARN(
      reporting_node->get_logger(),
      "Collision-aware IK timed out for line %d",
      line_id);
    return {};
  }

  const auto response = future.get();
  if (
    !response ||
    response->error_code.val !=
      moveit_msgs::msg::MoveItErrorCodes::SUCCESS)
  {
    RCLCPP_WARN(
      reporting_node->get_logger(),
      "No collision-free IK solution for line %d (code %d)",
      line_id,
      response ? response->error_code.val : 0);
    return {};
  }

  const auto& solution = response->solution.joint_state;
  if (solution.name.size() != solution.position.size())
  {
    throw std::runtime_error(
      "MoveIt returned malformed IK joint data");
  }

  std::map<std::string, double> solution_by_name;
  for (std::size_t index = 0;
       index < solution.name.size();
       ++index)
  {
    solution_by_name[solution.name[index]] =
      solution.position[index];
  }

  std::map<std::string, double> joint_targets;
  for (const auto& joint_name : active_joint_names)
  {
    const auto entry = solution_by_name.find(joint_name);
    if (entry == solution_by_name.end())
    {
      throw std::runtime_error(
        "IK response omitted active joint: " + joint_name);
    }
    joint_targets[joint_name] = entry->second;
  }

  return joint_targets;
}

void requireControllerMode(
  const rclcpp::Node::SharedPtr& node,
  bool simulation)
{
  /*
   * Keep this synchronous safety query off the main MoveGroup node.  A
   * short-lived guard node and an explicitly owned executor avoid transient
   * executor ownership of the node later handed to MoveGroupInterface.
   */
  auto guard_node = rclcpp::Node::make_shared(
    "saddle_controller_mode_guard");

  auto client = guard_node->create_client<
    controller_manager_msgs::srv::ListControllers>(
      "/controller_manager/list_controllers");

  RCLCPP_INFO(
    node->get_logger(),
    "Checking active trajectory controller...");

  if (!client->wait_for_service(5s))
  {
    throw std::runtime_error(
      "Controller manager is unavailable");
  }

  auto future = client->async_send_request(
    std::make_shared<
      controller_manager_msgs::srv::ListControllers::Request>());

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(guard_node);

  const auto deadline =
    std::chrono::steady_clock::now() + 5s;

  while (
    rclcpp::ok() &&
    future.wait_for(0s) != std::future_status::ready &&
    std::chrono::steady_clock::now() < deadline)
  {
    executor.spin_some(20ms);
  }

  executor.remove_node(guard_node);

  if (future.wait_for(0s) != std::future_status::ready)
  {
    throw std::runtime_error(
      "Timed out reading active controllers");
  }

  const auto response = future.get();
  if (!response)
  {
    throw std::runtime_error(
      "Controller manager returned an empty response");
  }

  bool mock_active = false;
  bool real_active = false;

  for (const auto& controller : response->controller)
  {
    if (controller.state != "active")
    {
      continue;
    }
    mock_active = mock_active ||
      controller.name == "ur_manipulator_controller";
    real_active = real_active ||
      controller.name == "scaled_joint_trajectory_controller";
  }

  if (simulation && (!mock_active || real_active))
  {
    throw std::runtime_error(
      "This saddle executor is simulation-only and requires "
      "the active mock trajectory controller");
  }

  if (!simulation && (!real_active || mock_active))
  {
    throw std::runtime_error(
      "Physical execution requires the active scaled UR "
      "trajectory controller and no active mock controller");
  }

  RCLCPP_INFO(
    node->get_logger(),
    "Controller mode check passed");
}

geometry_msgs::msg::Pose transformToPose(
  const tf2::Transform& transform)
{
  geometry_msgs::msg::Pose pose;

  const tf2::Vector3& position =
    transform.getOrigin();

  tf2::Quaternion orientation =
    transform.getRotation();

  orientation.normalize();

  pose.position.x = position.x();
  pose.position.y = position.y();
  pose.position.z = position.z();

  pose.orientation.x = orientation.x();
  pose.orientation.y = orientation.y();
  pose.orientation.z = orientation.z();
  pose.orientation.w = orientation.w();

  return pose;
}

std::map<int, std::vector<CsvToolpathPoint>>
loadToolpathCsv(const std::string& csv_path)
{
  std::ifstream input(csv_path);

  if (!input.is_open())
  {
    throw std::runtime_error(
      "Could not open toolpath CSV: " + csv_path);
  }

  std::string header_line;

  if (!std::getline(input, header_line))
  {
    throw std::runtime_error(
      "Toolpath CSV is empty: " + csv_path);
  }

  if (
    !header_line.empty() &&
    header_line.back() == '\r')
  {
    header_line.pop_back();
  }

  const std::vector<std::string> headers =
    splitCsvLine(header_line);

  std::unordered_map<std::string, std::size_t>
    header_indices;

  for (
    std::size_t index = 0;
    index < headers.size();
    ++index)
  {
    header_indices[headers[index]] = index;
  }

  const std::vector<std::string> required_headers = {
    "line_id",
    "point_index",
    "tcp_x_mm",
    "tcp_y_mm",
    "tcp_z_mm",
    "qx",
    "qy",
    "qz",
    "qw"
  };

  for (const std::string& header : required_headers)
  {
    if (header_indices.count(header) == 0)
    {
      throw std::runtime_error(
        "CSV is missing required column: " + header);
    }
  }

  std::map<int, std::vector<CsvToolpathPoint>>
    points_by_line;

  std::string line;
  std::size_t csv_row = 1;

  while (std::getline(input, line))
  {
    ++csv_row;

    if (
      !line.empty() &&
      line.back() == '\r')
    {
      line.pop_back();
    }

    if (line.empty())
    {
      continue;
    }

    const std::vector<std::string> values =
      splitCsvLine(line);

    if (values.size() != headers.size())
    {
      throw std::runtime_error(
        "CSV row " +
        std::to_string(csv_row) +
        " has an unexpected number of fields");
    }

    auto value =
      [&values, &header_indices](
        const std::string& name) -> const std::string&
      {
        return values.at(header_indices.at(name));
      };

    CsvToolpathPoint point;

    point.line_id =
      std::stoi(value("line_id"));

    point.point_index =
      std::stoi(value("point_index"));

    point.object_pose.position.x =
      std::stod(value("tcp_x_mm")) *
      kMillimetersToMeters;

    point.object_pose.position.y =
      std::stod(value("tcp_y_mm")) *
      kMillimetersToMeters;

    point.object_pose.position.z =
      std::stod(value("tcp_z_mm")) *
      kMillimetersToMeters;

    tf2::Quaternion orientation(
      std::stod(value("qx")),
      std::stod(value("qy")),
      std::stod(value("qz")),
      std::stod(value("qw")));

    if (orientation.length2() < 1e-12)
    {
      throw std::runtime_error(
        "CSV row " +
        std::to_string(csv_row) +
        " contains a zero-length quaternion");
    }

    orientation.normalize();

    point.object_pose.orientation.x =
      orientation.x();

    point.object_pose.orientation.y =
      orientation.y();

    point.object_pose.orientation.z =
      orientation.z();

    point.object_pose.orientation.w =
      orientation.w();

    points_by_line[point.line_id].push_back(
      point);
  }

  if (points_by_line.empty())
  {
    throw std::runtime_error(
      "Toolpath CSV contains no path points");
  }

  for (auto& entry : points_by_line)
  {
    std::vector<CsvToolpathPoint>& points =
      entry.second;

    std::sort(
      points.begin(),
      points.end(),
      [](
        const CsvToolpathPoint& first,
        const CsvToolpathPoint& second)
      {
        return
          first.point_index <
          second.point_index;
      });
  }

  return points_by_line;
}

std::vector<geometry_msgs::msg::Pose>
transformLineToWorld(
  const std::vector<CsvToolpathPoint>& points,
  const tf2::Transform& object_transform)
{
  std::vector<geometry_msgs::msg::Pose>
    world_poses;

  world_poses.reserve(points.size());

  for (const CsvToolpathPoint& point : points)
  {
    tf2::Quaternion tool_orientation(
      point.object_pose.orientation.x,
      point.object_pose.orientation.y,
      point.object_pose.orientation.z,
      point.object_pose.orientation.w);

    tool_orientation.normalize();

    const tf2::Transform object_tool_transform(
      tool_orientation,
      tf2::Vector3(
        point.object_pose.position.x,
        point.object_pose.position.y,
        point.object_pose.position.z));

    const tf2::Transform world_tool_transform =
      object_transform *
      object_tool_transform;

    world_poses.push_back(
      transformToPose(world_tool_transform));
  }

  return world_poses;
}

std::shared_ptr<moveit::core::RobotState>
trajectoryEndState(
  const moveit::core::RobotModelConstPtr& robot_model,
  const std::shared_ptr<moveit::core::RobotState>& seed_state,
  const moveit_msgs::msg::RobotTrajectory& trajectory)
{
  const auto& joint_trajectory =
    trajectory.joint_trajectory;

  if (joint_trajectory.points.empty())
  {
    throw std::runtime_error(
      "Trajectory contains no joint points");
  }

  const auto& final_point =
    joint_trajectory.points.back();

  if (
    joint_trajectory.joint_names.size() !=
    final_point.positions.size())
  {
    throw std::runtime_error(
      "Trajectory has mismatched joint names and positions");
  }

  std::shared_ptr<moveit::core::RobotState> state;

  if (seed_state)
  {
    state =
      std::make_shared<moveit::core::RobotState>(
        *seed_state);
  }
  else
  {
    state =
      std::make_shared<moveit::core::RobotState>(
        robot_model);

    state->setToDefaultValues();
  }

  state->setVariablePositions(
    joint_trajectory.joint_names,
    final_point.positions);

  state->update();

  return state;
}

void printPose(
  const rclcpp::Logger& logger,
  const std::string& label,
  const geometry_msgs::msg::Pose& pose)
{
  RCLCPP_INFO(
    logger,
    "%s position xyz=[%.6f, %.6f, %.6f]",
    label.c_str(),
    pose.position.x,
    pose.position.y,
    pose.position.z);

  RCLCPP_INFO(
    logger,
    "%s quaternion xyzw=[%.6f, %.6f, %.6f, %.6f]",
    label.c_str(),
    pose.orientation.x,
    pose.orientation.y,
    pose.orientation.z,
    pose.orientation.w);
}

bool validateTrajectorySegment(
  const rclcpp::Logger& logger,
  const moveit_msgs::msg::RobotTrajectory& trajectory,
  std::size_t segment_index,
  double maximum_allowed_joint_step)
{
  const auto& joint_trajectory =
    trajectory.joint_trajectory;

  const auto& joint_names =
    joint_trajectory.joint_names;

  const auto& points =
    joint_trajectory.points;

  if (joint_names.empty())
  {
    RCLCPP_ERROR(
      logger,
      "Segment %zu has no joint names",
      segment_index);

    return false;
  }

  if (points.empty())
  {
    RCLCPP_ERROR(
      logger,
      "Segment %zu has no points",
      segment_index);

    return false;
  }

  const std::size_t number_of_joints =
    joint_names.size();

  double previous_time = -1.0;
  double maximum_joint_step = 0.0;
  double maximum_velocity = 0.0;
  double maximum_acceleration = 0.0;

  bool has_velocity_data = false;
  bool has_acceleration_data = false;

  for (
    std::size_t point_index = 0;
    point_index < points.size();
    ++point_index)
  {
    const auto& point = points[point_index];

    if (point.positions.size() != number_of_joints)
    {
      RCLCPP_ERROR(
        logger,
        "Segment %zu point %zu has %zu positions "
        "for %zu joints",
        segment_index,
        point_index,
        point.positions.size(),
        number_of_joints);

      return false;
    }

    for (const double position : point.positions)
    {
      if (!std::isfinite(position))
      {
        RCLCPP_ERROR(
          logger,
          "Segment %zu point %zu has "
          "a nonfinite position",
          segment_index,
          point_index);

        return false;
      }
    }

    const double current_time =
      durationToSeconds(point.time_from_start);

    if (!std::isfinite(current_time))
    {
      RCLCPP_ERROR(
        logger,
        "Segment %zu point %zu has "
        "a nonfinite timestamp",
        segment_index,
        point_index);

      return false;
    }

    if (
      point_index > 0 &&
      current_time <= previous_time)
    {
      RCLCPP_ERROR(
        logger,
        "Segment %zu timestamps are not strictly "
        "increasing at point %zu: "
        "previous=%.9f, current=%.9f",
        segment_index,
        point_index,
        previous_time,
        current_time);

      return false;
    }

    previous_time = current_time;

    if (point_index > 0)
    {
      const auto& previous_point =
        points[point_index - 1];

      for (
        std::size_t joint_index = 0;
        joint_index < number_of_joints;
        ++joint_index)
      {
        const double joint_step =
          std::abs(
            point.positions[joint_index] -
            previous_point.positions[joint_index]);

        maximum_joint_step =
          std::max(
            maximum_joint_step,
            joint_step);
      }
    }

    if (!point.velocities.empty())
    {
      has_velocity_data = true;

      if (point.velocities.size() != number_of_joints)
      {
        RCLCPP_ERROR(
          logger,
          "Segment %zu point %zu has malformed "
          "velocity data",
          segment_index,
          point_index);

        return false;
      }

      for (const double velocity : point.velocities)
      {
        if (!std::isfinite(velocity))
        {
          RCLCPP_ERROR(
            logger,
            "Segment %zu point %zu has "
            "a nonfinite velocity",
            segment_index,
            point_index);

          return false;
        }

        maximum_velocity =
          std::max(
            maximum_velocity,
            std::abs(velocity));
      }
    }

    if (!point.accelerations.empty())
    {
      has_acceleration_data = true;

      if (
        point.accelerations.size() !=
        number_of_joints)
      {
        RCLCPP_ERROR(
          logger,
          "Segment %zu point %zu has malformed "
          "acceleration data",
          segment_index,
          point_index);

        return false;
      }

      for (
        const double acceleration :
        point.accelerations)
      {
        if (!std::isfinite(acceleration))
        {
          RCLCPP_ERROR(
            logger,
            "Segment %zu point %zu has "
            "a nonfinite acceleration",
            segment_index,
            point_index);

          return false;
        }

        maximum_acceleration =
          std::max(
            maximum_acceleration,
            std::abs(acceleration));
      }
    }
  }

  const double duration =
    durationToSeconds(
      points.back().time_from_start);

  /*
   * MoveIt can return a trajectory containing one point at time zero
   * when the robot is already at the requested target. This is a
   * valid no-op transition, not an invalid trajectory.
   */
  if (
    points.size() == 1 &&
    duration <= 0.0)
  {
    RCLCPP_INFO(
      logger,
      "Segment %zu is a one-point no-op trajectory; "
      "the robot is already at this segment's target",
      segment_index);

    return true;
  }

  if (duration <= 0.0)
  {
    RCLCPP_ERROR(
      logger,
      "Segment %zu has nonpositive duration: %.9f",
      segment_index,
      duration);

    return false;
  }

  RCLCPP_INFO(
    logger,
    "Segment %zu validation: "
    "points=%zu, duration=%.6f s, "
    "max_joint_step=%.6f rad",
    segment_index,
    points.size(),
    duration,
    maximum_joint_step);

  if (has_velocity_data)
  {
    RCLCPP_INFO(
      logger,
      "Segment %zu maximum velocity: "
      "%.6f rad/s",
      segment_index,
      maximum_velocity);
  }
  else
  {
    RCLCPP_WARN(
      logger,
      "Segment %zu has no explicit velocity data",
      segment_index);
  }

  if (has_acceleration_data)
  {
    RCLCPP_INFO(
      logger,
      "Segment %zu maximum acceleration: "
      "%.6f rad/s^2",
      segment_index,
      maximum_acceleration);
  }
  else
  {
    RCLCPP_WARN(
      logger,
      "Segment %zu has no explicit acceleration data",
      segment_index);
  }

  if (
    maximum_joint_step >
    maximum_allowed_joint_step)
  {
    RCLCPP_ERROR(
      logger,
      "Segment %zu exceeds maximum joint step: "
      "%.6f > %.6f rad",
      segment_index,
      maximum_joint_step,
      maximum_allowed_joint_step);

    return false;
  }

  return true;
}

bool validateSegmentBoundary(
  const rclcpp::Logger& logger,
  const moveit_msgs::msg::RobotTrajectory& previous,
  const moveit_msgs::msg::RobotTrajectory& next,
  std::size_t boundary_index,
  double maximum_allowed_boundary_jump)
{
  const auto& previous_trajectory =
    previous.joint_trajectory;

  const auto& next_trajectory =
    next.joint_trajectory;

  if (
    previous_trajectory.points.empty() ||
    next_trajectory.points.empty())
  {
    RCLCPP_ERROR(
      logger,
      "Boundary %zu references an empty trajectory",
      boundary_index);

    return false;
  }

  if (
    previous_trajectory.joint_names !=
    next_trajectory.joint_names)
  {
    RCLCPP_ERROR(
      logger,
      "Boundary %zu has inconsistent joint ordering",
      boundary_index);

    return false;
  }

  const auto& previous_final =
    previous_trajectory.points.back();

  const auto& next_first =
    next_trajectory.points.front();

  if (
    previous_final.positions.size() !=
    next_first.positions.size())
  {
    RCLCPP_ERROR(
      logger,
      "Boundary %zu has inconsistent position dimensions",
      boundary_index);

    return false;
  }

  double maximum_boundary_jump = 0.0;

  for (
    std::size_t joint_index = 0;
    joint_index < previous_final.positions.size();
    ++joint_index)
  {
    const double jump =
      std::abs(
        next_first.positions[joint_index] -
        previous_final.positions[joint_index]);

    maximum_boundary_jump =
      std::max(
        maximum_boundary_jump,
        jump);
  }

  RCLCPP_INFO(
    logger,
    "Boundary %zu maximum joint discontinuity: "
    "%.9f rad",
    boundary_index,
    maximum_boundary_jump);

  if (
    maximum_boundary_jump >
    maximum_allowed_boundary_jump)
  {
    RCLCPP_ERROR(
      logger,
      "Boundary %zu exceeds maximum allowed jump: "
      "%.9f > %.9f rad",
      boundary_index,
      maximum_boundary_jump,
      maximum_allowed_boundary_jump);

    return false;
  }

  return true;
}

class SafeSegmentExecutor
{
public:
  using ExecuteTrajectory =
    moveit_msgs::action::ExecuteTrajectory;
  using GoalHandle =
    rclcpp_action::ClientGoalHandle<ExecuteTrajectory>;

  SafeSegmentExecutor(
    bool physical,
    double maximum_line_duration,
    double maximum_line_volume,
    double maximum_total_volume)
  : physical_(physical),
    maximum_line_duration_(maximum_line_duration),
    maximum_line_volume_(maximum_line_volume),
    maximum_total_volume_(maximum_total_volume),
    node_(rclcpp::Node::make_shared(
        "saddle_segment_executor")),
    action_client_(rclcpp_action::create_client<
        ExecuteTrajectory>(node_, "/execute_trajectory")),
    flow_client_(node_->create_client<
        syringe_interfaces::srv::SetFlow>(
          "/syringe/set_flow")),
    stop_client_(node_->create_client<
        std_srvs::srv::Trigger>(
          "/syringe/stop")),
    keepalive_publisher_(node_->create_publisher<
        std_msgs::msg::Empty>(
          "/syringe/flow_keepalive", 10))
  {
    status_subscription_ = node_->create_subscription<
      std_msgs::msg::String>(
        "/syringe/status",
        10,
        [this](const std_msgs::msg::String::SharedPtr message)
        {
          bool extruding = false;
          {
            std::lock_guard<std::mutex> lock(mutex_);
            status_received_ = std::chrono::steady_clock::now();
            extruding = extruding_;
            if (message->data.rfind("STATE ", 0) == 0)
            {
              syringe_state_ = message->data.substr(6);
            }
          }

          if (
            extruding &&
            (message->data.rfind("FAULT", 0) == 0 ||
            message->data.rfind("ERROR", 0) == 0 ||
            message->data.rfind("LEASE_EXPIRED", 0) == 0))
          {
            tripFault(message->data);
          }
        });

    speed_subscription_ = node_->create_subscription<
      std_msgs::msg::Float64>(
        "/speed_scaling_state_broadcaster/speed_scaling",
        rclcpp::SensorDataQoS(),
        [this](const std_msgs::msg::Float64::SharedPtr message)
        {
          std::lock_guard<std::mutex> lock(mutex_);
          speed_scaling_ =
            message->data > 0.0 && message->data <= 1.0
            ? message->data * 100.0
            : message->data;
          speed_received_ = std::chrono::steady_clock::now();
        });

    joint_subscription_ = node_->create_subscription<
      sensor_msgs::msg::JointState>(
        "/joint_states",
        rclcpp::SensorDataQoS(),
        [this](const sensor_msgs::msg::JointState::SharedPtr message)
        {
          std::lock_guard<std::mutex> lock(mutex_);
          latest_joints_ = *message;
          joints_received_ = std::chrono::steady_clock::now();

          if (monitored_joint_names_.empty())
          {
            return;
          }

          const std::vector<double> positions =
            orderedJointPositionsLocked();

          if (positions.empty())
          {
            return;
          }

          if (motion_reference_.empty())
          {
            motion_reference_ = positions;
            return;
          }

          double maximum_change = 0.0;
          for (std::size_t index = 0;
               index < positions.size();
               ++index)
          {
            maximum_change = std::max(
              maximum_change,
              std::abs(
                positions[index] -
                motion_reference_[index]));
          }

          if (maximum_change > 0.00005)
          {
            motion_seen_ = true;
            last_motion_ = std::chrono::steady_clock::now();
            motion_reference_ = positions;
          }
        });

    safety_timer_ = node_->create_wall_timer(
      100ms,
      [this]() { safetyTick(); });

    executor_.add_node(node_);
    spin_thread_ = std::thread(
      [this]() { executor_.spin(); });
  }

  ~SafeSegmentExecutor()
  {
    if (physical_)
    {
      try
      {
        stopFlow(true);
      }
      catch (const std::exception& error)
      {
        RCLCPP_ERROR(
          node_->get_logger(),
          "Final syringe STOP unconfirmed: %s; "
          "use the physical stop",
          error.what());
      }
    }

    executor_.cancel();
    if (spin_thread_.joinable())
    {
      spin_thread_.join();
    }
    executor_.remove_node(node_);
  }

  void waitUntilReady(bool require_syringe)
  {
    if (!action_client_->wait_for_action_server(5s))
    {
      throw std::runtime_error(
        "MoveIt execute_trajectory action is unavailable");
    }

    if (!physical_)
    {
      return;
    }

    if (!stop_client_->wait_for_service(5s))
    {
      throw std::runtime_error(
        "Syringe STOP service is unavailable");
    }

    if (
      require_syringe &&
      !flow_client_->wait_for_service(5s))
    {
      throw std::runtime_error(
        "Syringe flow service is unavailable");
    }

    const auto deadline =
      std::chrono::steady_clock::now() + 5s;

    while (std::chrono::steady_clock::now() < deadline)
    {
      {
        std::lock_guard<std::mutex> lock(mutex_);
        const auto now = std::chrono::steady_clock::now();
        if (
          now - speed_received_ < 500ms &&
          now - joints_received_ < 500ms &&
          std::isfinite(speed_scaling_) &&
          speed_scaling_ >= 1.0 &&
          speed_scaling_ <= 100.0)
        {
          session_speed_scaling_ = speed_scaling_;
          return;
        }
      }
      std::this_thread::sleep_for(20ms);
    }

    throw std::runtime_error(
      "Fresh UR speed scaling and joint feedback are required");
  }

  double speedScalingFraction() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return session_speed_scaling_ / 100.0;
  }

  void establishStopped()
  {
    if (physical_)
    {
      stopFlow(true);
    }
  }

  void execute(
    const moveit_msgs::msg::RobotTrajectory& trajectory,
    bool extrude,
    double requested_flow,
    std::size_t segment_index)
  {
    if (extrude && !physical_)
    {
      throw std::runtime_error(
        "Extrusion is forbidden in simulation");
    }

    establishStopped();

    {
      std::lock_guard<std::mutex> lock(mutex_);
      faulted_ = false;
      fault_message_.clear();
      monitored_joint_names_ =
        trajectory.joint_trajectory.joint_names;
      motion_reference_ = orderedJointPositionsLocked();
      motion_seen_ = false;
      last_motion_ = std::chrono::steady_clock::now();
    }

    ExecuteTrajectory::Goal goal;
    goal.trajectory = trajectory;

    auto goal_future = action_client_->async_send_goal(goal);
    if (goal_future.wait_for(5s) != std::future_status::ready)
    {
      throw std::runtime_error(
        "Timed out waiting for trajectory acceptance");
    }

    const std::shared_ptr<GoalHandle> goal_handle =
      goal_future.get();

    if (!goal_handle)
    {
      throw std::runtime_error(
        "MoveIt rejected trajectory segment " +
        std::to_string(segment_index));
    }

    {
      std::lock_guard<std::mutex> lock(mutex_);
      active_goal_ = goal_handle;
    }

    auto result_future =
      action_client_->async_get_result(goal_handle);

    if (extrude)
    {
      const auto motion_deadline =
        std::chrono::steady_clock::now() + 5s;

      while (std::chrono::steady_clock::now() < motion_deadline)
      {
        if (!rclcpp::ok())
        {
          tripFault("ROS shutdown requested");
        }
        throwIfFaulted();

        if (result_future.wait_for(20ms) ==
          std::future_status::ready)
        {
          throw std::runtime_error(
            "Surface trace ended before extrusion could start");
        }

        std::lock_guard<std::mutex> lock(mutex_);
        if (motion_seen_)
        {
          break;
        }
      }

      bool motion_seen = false;
      {
        std::lock_guard<std::mutex> lock(mutex_);
        motion_seen = motion_seen_;
      }
      if (!motion_seen)
      {
        tripFault("No measured robot motion before extrusion");
      }

      throwIfFaulted();
      startFlow(requested_flow);
    }

    while (result_future.wait_for(50ms) !=
      std::future_status::ready)
    {
      if (!rclcpp::ok())
      {
        tripFault("ROS shutdown requested");
      }
      throwIfFaulted();
    }

    const auto wrapped_result = result_future.get();

    if (extrude)
    {
      stopFlow(true);
    }

    {
      std::lock_guard<std::mutex> lock(mutex_);
      active_goal_.reset();
      monitored_joint_names_.clear();
      motion_reference_.clear();
    }

    throwIfFaulted();

    if (
      wrapped_result.code !=
        rclcpp_action::ResultCode::SUCCEEDED ||
      !wrapped_result.result ||
      wrapped_result.result->error_code.val != 1)
    {
      throw std::runtime_error(
        "Robot failed while executing segment " +
        std::to_string(segment_index));
    }
  }

  double totalExtrudedVolume() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return total_volume_;
  }

private:
  std::vector<double> orderedJointPositionsLocked() const
  {
    if (
      latest_joints_.name.empty() ||
      latest_joints_.name.size() !=
        latest_joints_.position.size())
    {
      return {};
    }

    std::unordered_map<std::string, double> positions;
    for (std::size_t index = 0;
         index < latest_joints_.name.size();
         ++index)
    {
      positions[latest_joints_.name[index]] =
        latest_joints_.position[index];
    }

    std::vector<double> ordered;
    ordered.reserve(monitored_joint_names_.size());
    for (const std::string& name : monitored_joint_names_)
    {
      const auto found = positions.find(name);
      if (
        found == positions.end() ||
        !std::isfinite(found->second))
      {
        return {};
      }
      ordered.push_back(found->second);
    }
    return ordered;
  }

  void startFlow(double requested_flow)
  {
    bool feedback_stale = false;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      const auto now = std::chrono::steady_clock::now();
      feedback_stale =
        now - speed_received_ > 500ms ||
        now - joints_received_ > 500ms ||
        !std::isfinite(speed_scaling_) ||
        speed_scaling_ <= 0.0 ||
        std::abs(
          speed_scaling_ -
          session_speed_scaling_) > 2.0;
    }
    if (feedback_stale)
    {
      tripFault("Robot feedback is not fresh at flow start");
    }
    throwIfFaulted();

    auto request = std::make_shared<
      syringe_interfaces::srv::SetFlow::Request>();
    request->enabled = true;
    request->flow_ml_per_min = requested_flow;
    request->retract = false;

    const auto requested_at =
      std::chrono::steady_clock::now();
    auto future = flow_client_->async_send_request(request);

    if (future.wait_for(3s) != std::future_status::ready)
    {
      tripFault("Syringe flow acknowledgement timed out");
      throwIfFaulted();
    }

    const auto response = future.get();
    if (!response->success)
    {
      tripFault(
        "Syringe rejected flow: " + response->message);
      throwIfFaulted();
    }

    if (
      !std::isfinite(response->effective_flow_ml_per_min) ||
      response->effective_flow_ml_per_min <= 0.0)
    {
      tripFault("Syringe returned an invalid effective flow");
      throwIfFaulted();
    }

    {
      std::lock_guard<std::mutex> lock(mutex_);
      effective_flow_ =
        response->effective_flow_ml_per_min;
      baseline_speed_scaling_ = session_speed_scaling_;
      flow_started_ = requested_at;
      last_motion_ = requested_at;
      extruding_ = true;
    }

    RCLCPP_WARN(
      node_->get_logger(),
      "Extrusion started at %.4f mL/min",
      response->effective_flow_ml_per_min);
  }

  void stopFlow(bool require_acknowledgement)
  {
    double line_volume = 0.0;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (extruding_)
      {
        const double seconds =
          std::chrono::duration<double>(
            std::chrono::steady_clock::now() -
            flow_started_).count();
        line_volume =
          seconds * effective_flow_ / 60.0;
        total_volume_ += line_volume;
      }
      extruding_ = false;
    }

    if (!physical_ || !stop_client_->service_is_ready())
    {
      if (require_acknowledgement && physical_)
      {
        throw std::runtime_error(
          "Syringe STOP service is unavailable");
      }
      return;
    }

    auto future = stop_client_->async_send_request(
      std::make_shared<std_srvs::srv::Trigger::Request>());

    if (future.wait_for(3s) != std::future_status::ready)
    {
      if (require_acknowledgement)
      {
        throw std::runtime_error(
          "Syringe STOP acknowledgement timed out");
      }
      return;
    }

    const auto response = future.get();
    if (!response->success && require_acknowledgement)
    {
      throw std::runtime_error(
        "Syringe STOP was rejected: " +
        response->message);
    }

    if (line_volume > 0.0)
    {
      RCLCPP_INFO(
        node_->get_logger(),
        "Extrusion stopped; estimated line volume %.4f mL, "
        "total %.4f mL",
        line_volume,
        totalExtrudedVolume());
    }
  }

  void safetyTick()
  {
    bool publish_keepalive = false;
    std::string error;

    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (!extruding_)
      {
        return;
      }

      publish_keepalive = true;
      const auto now = std::chrono::steady_clock::now();
      const double line_seconds =
        std::chrono::duration<double>(
          now - flow_started_).count();
      const double line_volume =
        line_seconds * effective_flow_ / 60.0;

      if (line_seconds > maximum_line_duration_)
      {
        error = "Maximum line extrusion duration reached";
      }
      else if (line_volume >= maximum_line_volume_)
      {
        error = "Maximum line extrusion volume reached";
      }
      else if (
        total_volume_ + line_volume >=
        maximum_total_volume_)
      {
        error = "Maximum total extrusion volume reached";
      }
      else if (now - joints_received_ > 500ms)
      {
        error = "Robot joint feedback became stale";
      }
      else if (now - speed_received_ > 500ms)
      {
        error = "UR speed-scaling feedback became stale";
      }
      else if (
        !std::isfinite(speed_scaling_) ||
        speed_scaling_ <= 0.0 ||
        std::abs(
          speed_scaling_ -
          baseline_speed_scaling_) > 2.0)
      {
        error = "Robot paused/stopped or speed scaling changed";
      }
      else if (now - last_motion_ > 1s)
      {
        error = "No measured robot progress for one second";
      }
      else if (
        line_seconds > 0.75 &&
        syringe_state_.rfind("MOVING", 0) != 0)
      {
        error = "Syringe is not reporting MOVING";
      }
      else if (
        line_seconds > 1.0 &&
        now - status_received_ > 1s)
      {
        error = "Syringe status became stale";
      }
    }

    if (publish_keepalive)
    {
      keepalive_publisher_->publish(
        std_msgs::msg::Empty());
    }

    if (!error.empty())
    {
      tripFault(error);
    }
  }

  void tripFault(const std::string& message)
  {
    bool expected = false;
    if (!faulted_.compare_exchange_strong(expected, true))
    {
      return;
    }

    std::shared_ptr<GoalHandle> goal;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      fault_message_ = message;
      extruding_ = false;
      goal = active_goal_;
    }

    RCLCPP_ERROR(
      node_->get_logger(),
      "Print safety fault: %s",
      message.c_str());

    if (stop_client_->service_is_ready())
    {
      stop_client_->async_send_request(
        std::make_shared<std_srvs::srv::Trigger::Request>());
    }
    if (goal)
    {
      action_client_->async_cancel_goal(goal);
    }
  }

  void throwIfFaulted() const
  {
    if (!faulted_)
    {
      return;
    }

    std::lock_guard<std::mutex> lock(mutex_);
    throw std::runtime_error(fault_message_);
  }

  bool physical_;
  double maximum_line_duration_;
  double maximum_line_volume_;
  double maximum_total_volume_;

  rclcpp::Node::SharedPtr node_;
  rclcpp::executors::SingleThreadedExecutor executor_;
  std::thread spin_thread_;
  rclcpp_action::Client<ExecuteTrajectory>::SharedPtr
    action_client_;
  rclcpp::Client<syringe_interfaces::srv::SetFlow>::SharedPtr
    flow_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr
    stop_client_;
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr
    keepalive_publisher_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr
    status_subscription_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr
    speed_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr
    joint_subscription_;
  rclcpp::TimerBase::SharedPtr safety_timer_;

  mutable std::mutex mutex_;
  sensor_msgs::msg::JointState latest_joints_;
  std::vector<std::string> monitored_joint_names_;
  std::vector<double> motion_reference_;
  std::shared_ptr<GoalHandle> active_goal_;
  std::chrono::steady_clock::time_point joints_received_{};
  std::chrono::steady_clock::time_point speed_received_{};
  std::chrono::steady_clock::time_point status_received_{};
  std::chrono::steady_clock::time_point flow_started_{};
  std::chrono::steady_clock::time_point last_motion_{};
  std::string syringe_state_;
  std::string fault_message_;
  double speed_scaling_ = 0.0;
  double session_speed_scaling_ = 0.0;
  double baseline_speed_scaling_ = 0.0;
  double effective_flow_ = 0.0;
  double total_volume_ = 0.0;
  bool extruding_ = false;
  bool motion_seen_ = false;
  std::atomic<bool> faulted_{false};
};

}  // namespace

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);

  auto node =
    rclcpp::Node::make_shared(
      "plan_execute_surface_toolpath");

  const auto logger = node->get_logger();

  try
  {
    const std::string package_share =
      ament_index_cpp::get_package_share_directory(
        "ur5e_moveit_cpp");

    const std::string default_csv_path =
      package_share +
      "/data/toolpaths/saddle/raster_45_deg.csv";

    const std::string csv_path =
      node->declare_parameter<std::string>(
        "csv_path",
        default_csv_path);

    const std::string planning_group =
      node->declare_parameter<std::string>(
        "planning_group",
        "ur_manipulator");

    const std::string tcp_link =
      node->declare_parameter<std::string>(
        "tcp_link",
        "probe_tcp");

    const std::string planning_frame =
      node->declare_parameter<std::string>(
        "planning_frame",
        "world");

    const std::int64_t selected_line_id =
      node->declare_parameter<std::int64_t>(
        "line_id",
        0);

    const double object_x =
      node->declare_parameter<double>(
        "object_x",
        0.55);

    const double object_y =
      node->declare_parameter<double>(
        "object_y",
        0.00);

    const double object_z =
      node->declare_parameter<double>(
        "object_z",
        0.10);

    const double object_roll_degrees =
      node->declare_parameter<double>(
        "object_roll_deg",
        0.0);

    const double object_pitch_degrees =
      node->declare_parameter<double>(
        "object_pitch_deg",
        0.0);

    const double object_yaw_degrees =
      node->declare_parameter<double>(
        "object_yaw_deg",
        0.0);

    const double planning_time =
      node->declare_parameter<double>(
        "planning_time",
        15.0);

    const std::int64_t planning_attempts =
      node->declare_parameter<std::int64_t>(
        "planning_attempts",
        20);

    const std::int64_t approach_retries =
      node->declare_parameter<std::int64_t>(
        "approach_retries",
        5);

    const double velocity_scale =
      node->declare_parameter<double>(
        "velocity_scale",
        0.10);

    const double acceleration_scale =
      node->declare_parameter<double>(
        "acceleration_scale",
        0.10);

    const double eef_step =
      node->declare_parameter<double>(
        "eef_step",
        0.001);

    const double jump_threshold =
      node->declare_parameter<double>(
        "jump_threshold",
        0.0);

    const double minimum_fraction =
      node->declare_parameter<double>(
        "minimum_fraction",
        0.999);

    const double maximum_joint_step =
      node->declare_parameter<double>(
        "maximum_joint_step",
        0.25);

    const double maximum_boundary_jump =
      node->declare_parameter<double>(
        "maximum_boundary_jump",
        0.02);

    const bool execute =
      node->declare_parameter<bool>(
        "execute",
        false);

    const bool simulation =
      node->declare_parameter<bool>(
        "simulation",
        true);

    const bool extrude =
      node->declare_parameter<bool>(
        "extrude",
        false);

    const double flow_ml_per_min =
      node->declare_parameter<double>(
        "flow_ml_per_min",
        1.0);

    const double maximum_line_extrusion_seconds =
      node->declare_parameter<double>(
        "max_line_extrusion_sec",
        60.0);

    const double maximum_line_volume_ml =
      node->declare_parameter<double>(
        "max_line_volume_ml",
        1.0);

    const double maximum_total_volume_ml =
      node->declare_parameter<double>(
        "max_total_volume_ml",
        1.0);

    const bool hardware_confirmed =
      node->declare_parameter<bool>(
        "hardware_confirmed",
        false);

    const bool extrusion_confirmed =
      node->declare_parameter<bool>(
        "extrusion_confirmed",
        false);

    /*
     * This parameter name is retained for compatibility with the
     * existing command. It is currently acting as an explicit
     * execution-confirmation guard.
     */
    const bool confirm_mock_hardware =
      node->declare_parameter<bool>(
        "confirm_mock_hardware",
        false);

    if (csv_path.empty())
    {
      throw std::runtime_error(
        "csv_path cannot be empty");
    }

    if (planning_group.empty())
    {
      throw std::runtime_error(
        "planning_group cannot be empty");
    }

    if (tcp_link.empty())
    {
      throw std::runtime_error(
        "tcp_link cannot be empty");
    }

    if (planning_frame.empty())
    {
      throw std::runtime_error(
        "planning_frame cannot be empty");
    }

    if (planning_time <= 0.0)
    {
      throw std::runtime_error(
        "planning_time must be positive");
    }

    if (planning_attempts < 1)
    {
      throw std::runtime_error(
        "planning_attempts must be at least 1");
    }

    if (approach_retries < 1 || approach_retries > 20)
    {
      throw std::runtime_error(
        "approach_retries must be within [1, 20]");
    }

    if (
      velocity_scale <= 0.0 ||
      velocity_scale > 1.0)
    {
      throw std::runtime_error(
        "velocity_scale must be in (0, 1]");
    }

    if (
      acceleration_scale <= 0.0 ||
      acceleration_scale > 1.0)
    {
      throw std::runtime_error(
        "acceleration_scale must be in (0, 1]");
    }

    if (eef_step <= 0.0)
    {
      throw std::runtime_error(
        "eef_step must be positive");
    }

    if (jump_threshold < 0.0)
    {
      throw std::runtime_error(
        "jump_threshold cannot be negative");
    }

    if (
      minimum_fraction <= 0.0 ||
      minimum_fraction > 1.0)
    {
      throw std::runtime_error(
        "minimum_fraction must be in (0, 1]");
    }

    if (maximum_joint_step <= 0.0)
    {
      throw std::runtime_error(
        "maximum_joint_step must be positive");
    }

    if (maximum_boundary_jump < 0.0)
    {
      throw std::runtime_error(
        "maximum_boundary_jump cannot be negative");
    }

    if (
      !std::isfinite(flow_ml_per_min) ||
      flow_ml_per_min <= 0.0)
    {
      throw std::runtime_error(
        "flow_ml_per_min must be finite and positive");
    }

    if (
      !std::isfinite(maximum_line_extrusion_seconds) ||
      maximum_line_extrusion_seconds <= 0.0 ||
      !std::isfinite(maximum_line_volume_ml) ||
      maximum_line_volume_ml <= 0.0 ||
      !std::isfinite(maximum_total_volume_ml) ||
      maximum_total_volume_ml <= 0.0)
    {
      throw std::runtime_error(
        "Extrusion duration and volume limits must be "
        "finite and positive");
    }

    if (extrude && (!execute || simulation))
    {
      throw std::runtime_error(
        "Extrusion requires physical execution");
    }

    if (execute && !simulation && !hardware_confirmed)
    {
      throw std::runtime_error(
        "Physical execution requires hardware_confirmed:=true");
    }

    if (extrude && !extrusion_confirmed)
    {
      throw std::runtime_error(
        "Extrusion requires extrusion_confirmed:=true");
    }

    if (execute && simulation && !confirm_mock_hardware)
    {
      throw std::runtime_error(
        "Execution requested, but confirm_mock_hardware "
        "is false. Set it to true only after confirming "
        "that execution is safe.");
    }


    if (execute)
    {
      requireControllerMode(node, simulation);
    }

    std::map<int, std::vector<CsvToolpathPoint>>
      object_lines =
        loadToolpathCsv(csv_path);

    RCLCPP_INFO(
      logger,
      "Loaded %zu lines from %s",
      object_lines.size(),
      csv_path.c_str());

    std::vector<int> lines_to_plan;

    if (selected_line_id >= 0)
    {
      const int line_id =
        static_cast<int>(selected_line_id);

      if (object_lines.count(line_id) == 0)
      {
        std::stringstream available;

        for (const auto& entry : object_lines)
        {
          available << entry.first << " ";
        }

        throw std::runtime_error(
          "Requested line_id " +
          std::to_string(line_id) +
          " does not exist. Available line IDs: " +
          available.str());
      }

      lines_to_plan.push_back(line_id);
    }
    else
    {
      for (const auto& entry : object_lines)
      {
        lines_to_plan.push_back(entry.first);
      }
    }

    tf2::Quaternion object_orientation;

    object_orientation.setRPY(
      degreesToRadians(object_roll_degrees),
      degreesToRadians(object_pitch_degrees),
      degreesToRadians(object_yaw_degrees));

    object_orientation.normalize();

    const tf2::Transform object_transform(
      object_orientation,
      tf2::Vector3(
        object_x,
        object_y,
        object_z));

    RCLCPP_INFO(
      logger,
      "Object pose in %s: "
      "xyz=[%.3f, %.3f, %.3f], "
      "rpy_deg=[%.1f, %.1f, %.1f]",
      planning_frame.c_str(),
      object_x,
      object_y,
      object_z,
      object_roll_degrees,
      object_pitch_degrees,
      object_yaw_degrees);

    moveit::planning_interface::MoveGroupInterface
      move_group(
        node,
        planning_group);

    move_group.setEndEffectorLink(tcp_link);
    move_group.setPoseReferenceFrame(planning_frame);
    move_group.setPlanningTime(planning_time);

    move_group.setNumPlanningAttempts(
      static_cast<int>(planning_attempts));

    move_group.setMaxVelocityScalingFactor(
      velocity_scale);

    move_group.setMaxAccelerationScalingFactor(
      acceleration_scale);

    rclcpp::QoS display_qos(1);
    display_qos.reliable();
    display_qos.transient_local();

    auto display_publisher =
      node->create_publisher<
        moveit_msgs::msg::DisplayTrajectory>(
          "/display_planned_path",
          display_qos);

    moveit_msgs::msg::DisplayTrajectory
      display_trajectory;

    display_trajectory.model_id =
      move_group.getRobotModel()->getName();

    std::shared_ptr<moveit::core::RobotState>
      planned_state;

    const std::vector<std::string> active_joint_names =
      move_group.getJointNames();

    std::size_t successful_lines = 0;

    for (const int line_id : lines_to_plan)
    {
      const auto& object_points =
        object_lines.at(line_id);

      if (object_points.size() < 2)
      {
        RCLCPP_ERROR(
          logger,
          "Line %d contains fewer than two points",
          line_id);

        break;
      }

      const std::vector<geometry_msgs::msg::Pose>
        world_waypoints =
          transformLineToWorld(
            object_points,
            object_transform);

      const geometry_msgs::msg::Pose&
        approach_target =
          world_waypoints.front();

      RCLCPP_INFO(
        logger,
        "==================================================");

      RCLCPP_INFO(
        logger,
        "Planning line %d with %zu path points",
        line_id,
        world_waypoints.size());

      printPose(
        logger,
        "Approach target",
        approach_target);

      moveit::planning_interface::
        MoveGroupInterface::Plan approach_plan;

      RCLCPP_INFO(
        logger,
        "Planning collision-aware OMPL transition "
        "to line %d...",
        line_id);

      bool approach_succeeded = false;
      for (std::int64_t retry = 1;
           retry <= approach_retries;
           ++retry)
      {
        std::shared_ptr<moveit::core::RobotState>
          approach_start_state;

        if (planned_state)
        {
          approach_start_state =
            std::make_shared<moveit::core::RobotState>(
              *planned_state);
        }
        else
        {
          approach_start_state =
            move_group.getCurrentState(10.0);

          if (!approach_start_state)
          {
            throw std::runtime_error(
              "Could not obtain the current robot state");
          }
        }

        move_group.setStartState(*approach_start_state);

        const auto joint_targets =
          resolveCollisionFreeIk(
            node,
            planning_group,
            planning_frame,
            tcp_link,
            approach_target,
            *approach_start_state,
            active_joint_names,
            std::min(5.0, planning_time),
            line_id);

        if (joint_targets.empty())
        {
          RCLCPP_WARN(
            logger,
            "Approach IK attempt %ld of %ld failed "
            "for line %d",
            static_cast<long>(retry),
            static_cast<long>(approach_retries),
            line_id);
          continue;
        }

        if (!move_group.setJointValueTarget(joint_targets))
        {
          RCLCPP_WARN(
            logger,
            "MoveIt rejected the IK joint target for line %d",
            line_id);
          continue;
        }

        const auto approach_result = move_group.plan(approach_plan);
        if (approach_result == moveit::core::MoveItErrorCode::SUCCESS)
        {
          approach_succeeded = true;
          break;
        }
        RCLCPP_WARN(
          logger,
          "Approach attempt %ld of %ld failed for line %d",
          static_cast<long>(retry),
          static_cast<long>(approach_retries),
          line_id);
      }

      if (!approach_succeeded)
      {
        RCLCPP_ERROR(
          logger,
          "Approach planning failed for line %d",
          line_id);

        break;
      }

      if (
        approach_plan.trajectory
        .joint_trajectory.points.empty())
      {
        RCLCPP_ERROR(
          logger,
          "Approach trajectory for line %d is empty",
          line_id);

        break;
      }

      RCLCPP_INFO(
        logger,
        "Approach succeeded with %zu trajectory points",
        approach_plan.trajectory
          .joint_trajectory.points.size());

      if (display_trajectory.trajectory.empty())
      {
        display_trajectory.trajectory_start =
          approach_plan.start_state;
      }

      display_trajectory.trajectory.push_back(
        approach_plan.trajectory);

      std::shared_ptr<moveit::core::RobotState>
        approach_end_state =
          trajectoryEndState(
            move_group.getRobotModel(),
            planned_state,
            approach_plan.trajectory);

      move_group.setStartState(
        *approach_end_state);

      std::vector<geometry_msgs::msg::Pose>
        cartesian_waypoints;

      cartesian_waypoints.reserve(
        world_waypoints.size() - 1);

      for (
        std::size_t index = 1;
        index < world_waypoints.size();
        ++index)
      {
        cartesian_waypoints.push_back(
          world_waypoints[index]);
      }

      moveit_msgs::msg::RobotTrajectory
        cartesian_trajectory;

      RCLCPP_INFO(
        logger,
        "Computing collision-aware Cartesian path "
        "for line %d with %zu waypoints...",
        line_id,
        cartesian_waypoints.size());

      const double fraction =
        move_group.computeCartesianPath(
          cartesian_waypoints,
          eef_step,
          jump_threshold,
          cartesian_trajectory,
          true);

      RCLCPP_INFO(
        logger,
        "Line %d Cartesian fraction: %.6f",
        line_id,
        fraction);

      RCLCPP_INFO(
        logger,
        "Line %d Cartesian trajectory points: %zu",
        line_id,
        cartesian_trajectory
          .joint_trajectory.points.size());

      if (
        !cartesian_trajectory
          .joint_trajectory.points.empty())
      {
        display_trajectory.trajectory.push_back(
          cartesian_trajectory);
      }

      if (fraction < minimum_fraction)
      {
        RCLCPP_ERROR(
          logger,
          "Line %d was incomplete: %.2f%%. "
          "Stopping without execution.",
          line_id,
          fraction * 100.0);

        break;
      }

      planned_state =
        trajectoryEndState(
          move_group.getRobotModel(),
          approach_end_state,
          cartesian_trajectory);

      ++successful_lines;

      RCLCPP_INFO(
        logger,
        "Line %d planned successfully",
        line_id);
    }

    if (display_trajectory.trajectory.empty())
    {
      throw std::runtime_error(
        "Planning produced no trajectory segments");
    }

    bool trajectories_valid = true;

    RCLCPP_INFO(
      logger,
      "==================================================");

    RCLCPP_INFO(
      logger,
      "Validating %zu trajectory segments...",
      display_trajectory.trajectory.size());

    for (
      std::size_t segment_index = 0;
      segment_index <
        display_trajectory.trajectory.size();
      ++segment_index)
    {
      const bool segment_valid =
        validateTrajectorySegment(
          logger,
          display_trajectory.trajectory[
            segment_index],
          segment_index,
          maximum_joint_step);

      if (!segment_valid)
      {
        trajectories_valid = false;
        break;
      }

      if (segment_index > 0)
      {
        const bool boundary_valid =
          validateSegmentBoundary(
            logger,
            display_trajectory.trajectory[
              segment_index - 1],
            display_trajectory.trajectory[
              segment_index],
            segment_index - 1,
            maximum_boundary_jump);

        if (!boundary_valid)
        {
          trajectories_valid = false;
          break;
        }
      }
    }

    std::this_thread::sleep_for(500ms);

    display_publisher->publish(
      display_trajectory);

    RCLCPP_INFO(
      logger,
      "Published %zu trajectory segments "
      "on /display_planned_path",
      display_trajectory.trajectory.size());

    std::this_thread::sleep_for(2s);

    RCLCPP_INFO(
      logger,
      "==================================================");

    RCLCPP_INFO(
      logger,
      "Successfully planned %zu of %zu requested lines",
      successful_lines,
      lines_to_plan.size());

    if (successful_lines != lines_to_plan.size())
    {
      RCLCPP_ERROR(
        logger,
        "Not all requested lines were planned. "
        "Execution is prohibited.");

      rclcpp::shutdown();
      return 1;
    }

    if (!trajectories_valid)
    {
      RCLCPP_ERROR(
        logger,
        "Trajectory validation failed. "
        "Execution is prohibited.");

      rclcpp::shutdown();
      return 1;
    }

    RCLCPP_INFO(
      logger,
      "All trajectory segments passed validation");

    if (!execute)
    {
      RCLCPP_INFO(
        logger,
        "PLAN-ONLY: no robot motion was executed");

      rclcpp::shutdown();
      return 0;
    }

    if (simulation)
    {
      RCLCPP_WARN(
        logger,
        "MOCK SIMULATION EXECUTION ENABLED");
    }
    else
    {
      RCLCPP_WARN(
        logger,
        "PHYSICAL EXECUTION ENABLED");
      RCLCPP_WARN(
        logger,
        extrude
          ? "Extrusion will run only during measured Cartesian "
            "surface motion"
          : "Extrusion is disabled");
    }

    SafeSegmentExecutor segment_executor(
      !simulation,
      maximum_line_extrusion_seconds,
      maximum_line_volume_ml,
      maximum_total_volume_ml);

    segment_executor.waitUntilReady(extrude);
    segment_executor.establishStopped();

    if (!simulation && extrude)
    {
      const double speed_fraction =
        segment_executor.speedScalingFraction();
      double estimated_total_volume = 0.0;

      for (std::size_t segment_index = 1;
           segment_index <
             display_trajectory.trajectory.size();
           segment_index += 2)
      {
        const auto& points =
          display_trajectory.trajectory[segment_index]
            .joint_trajectory.points;
        if (points.empty())
        {
          throw std::runtime_error(
            "Surface trace contains no trajectory points");
        }

        const double nominal_seconds =
          durationToSeconds(
            points.back().time_from_start);
        const double estimated_seconds =
          nominal_seconds / speed_fraction;
        const double estimated_volume =
          estimated_seconds * flow_ml_per_min / 60.0;

        if (
          estimated_seconds >
            maximum_line_extrusion_seconds ||
          estimated_volume > maximum_line_volume_ml)
        {
          throw std::runtime_error(
            "A surface line exceeds the configured extrusion "
            "duration or volume budget");
        }
        estimated_total_volume += estimated_volume;
      }

      if (estimated_total_volume > maximum_total_volume_ml)
      {
        throw std::runtime_error(
          "The full saddle exceeds max_total_volume_ml");
      }

      RCLCPP_WARN(
        logger,
        "Estimated extrusion at %.1f%% speed scaling: "
        "%.3f mL (limit %.3f mL)",
        speed_fraction * 100.0,
        estimated_total_volume,
        maximum_total_volume_ml);
    }

    RCLCPP_WARN(
      logger,
      "Executing %zu trajectory segments sequentially",
      display_trajectory.trajectory.size());

    for (
      std::size_t segment_index = 0;
      segment_index <
        display_trajectory.trajectory.size();
      ++segment_index)
    {
      const auto& segment =
        display_trajectory.trajectory[
          segment_index];

      const auto& segment_points =
        segment.joint_trajectory.points;

      const double segment_duration =
        segment_points.empty()
        ? 0.0
        : durationToSeconds(
            segment_points.back().time_from_start);

      /*
       * Do not submit a one-point, zero-duration trajectory to
       * the controller. It only means the robot is already at
       * that segment's target.
       */
      if (
        segment_points.size() == 1 &&
        segment_duration <= 0.0)
      {
        RCLCPP_INFO(
          logger,
          "Skipping segment %zu because it is a "
          "one-point no-op trajectory",
          segment_index);

        continue;
      }

      const bool is_transition =
        segment_index % 2 == 0;

      RCLCPP_WARN(
        logger,
        "Executing segment %zu of %zu: %s",
        segment_index + 1,
        display_trajectory.trajectory.size(),
        is_transition
          ? "OMPL transition"
          : "Cartesian surface trace");

      try
      {
        segment_executor.execute(
          segment,
          !is_transition && extrude,
          flow_ml_per_min,
          segment_index);
      }
      catch (...)
      {
        segment_executor.establishStopped();
        throw;
      }

      RCLCPP_INFO(
        logger,
        "Segment %zu executed successfully",
        segment_index);
    }

    RCLCPP_INFO(
      logger,
      "All trajectory segments executed successfully; "
      "estimated extruded volume %.4f mL",
      segment_executor.totalExtrudedVolume());
  }
  catch (const std::exception& error)
  {
    RCLCPP_FATAL(
      logger,
      "%s",
      error.what());

    rclcpp::shutdown();
    return 1;
  }

  rclcpp::shutdown();
  return 0;
}

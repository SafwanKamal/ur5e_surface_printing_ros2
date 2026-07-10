#include <chrono>
#include <fstream>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include <geometry_msgs/msg/pose.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit/robot_state/robot_state.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <moveit_msgs/srv/apply_planning_scene.hpp>
#include <rclcpp/rclcpp.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>

namespace
{

struct PathPoint
{
  geometry_msgs::msg::Pose pose;
  double nx{};
  double ny{};
  double nz{};
};

std::vector<PathPoint> loadPathCsv(const std::string & csv_path)
{
  std::ifstream input(csv_path);

  if (!input.is_open())
  {
    throw std::runtime_error("Could not open CSV file: " + csv_path);
  }

  std::string line;
  std::getline(input, line);  // Skip header row.

  std::vector<PathPoint> points;

  while (std::getline(input, line))
  {
    if (line.empty())
    {
      continue;
    }

    std::stringstream stream(line);
    std::string token;
    std::vector<double> values;

    while (std::getline(stream, token, ','))
    {
      values.push_back(std::stod(token));
    }

    if (values.size() < 10)
    {
      throw std::runtime_error(
        "Malformed CSV row in " + csv_path + ": " + line);
    }

    PathPoint point;

    point.pose.position.x = values[0];
    point.pose.position.y = values[1];
    point.pose.position.z = values[2];

    point.pose.orientation.x = values[3];
    point.pose.orientation.y = values[4];
    point.pose.orientation.z = values[5];
    point.pose.orientation.w = values[6];

    point.nx = values[7];
    point.ny = values[8];
    point.nz = values[9];

    points.push_back(point);
  }

  return points;
}

void printPose(
  const rclcpp::Logger & logger,
  const std::string & label,
  const geometry_msgs::msg::Pose & pose)
{
  RCLCPP_INFO(
    logger,
    "%s position: x=%.6f y=%.6f z=%.6f",
    label.c_str(),
    pose.position.x,
    pose.position.y,
    pose.position.z);

  RCLCPP_INFO(
    logger,
    "%s orientation: x=%.6f y=%.6f z=%.6f w=%.6f",
    label.c_str(),
    pose.orientation.x,
    pose.orientation.y,
    pose.orientation.z,
    pose.orientation.w);
}

}  // namespace

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto node = rclcpp::Node::make_shared("hemisphere_surface_demo");
  const auto logger = node->get_logger();

  constexpr char kPlanningGroup[] = "ur_manipulator";
  constexpr char kTcpLink[] = "probe_tcp";
  constexpr char kPlanningFrame[] = "world";

  const std::string csv_path =
    node->declare_parameter<std::string>(
      "csv_path",
      "/home/desktop/ur5e_ws/paths/hemisphere_ring_start_00.csv");

  const bool execute =
    node->declare_parameter<bool>("execute", false);

  // Cartesian interpolation spacing in meters.
  const double eef_step =
    node->declare_parameter<double>("eef_step", 0.002);

  // 0.0 disables joint-space jump detection for now.
  const double jump_threshold =
    node->declare_parameter<double>("jump_threshold", 0.0);

  // ------------------------------------------------------------
  // Load ring CSV.
  // ------------------------------------------------------------

  std::vector<PathPoint> path_points;

  try
  {
    path_points = loadPathCsv(csv_path);
  }
  catch (const std::exception & exception)
  {
    RCLCPP_ERROR(logger, "%s", exception.what());
    rclcpp::shutdown();
    return 1;
  }

  if (path_points.size() < 2)
  {
    RCLCPP_ERROR(
      logger,
      "CSV needs at least two waypoints for Cartesian path planning.");

    rclcpp::shutdown();
    return 1;
  }

  const geometry_msgs::msg::Pose approach_target =
    path_points.front().pose;

  RCLCPP_INFO(
    logger,
    "Loaded %zu CSV waypoints from: %s",
    path_points.size(),
    csv_path.c_str());

  printPose(logger, "Start 00 TCP target", approach_target);

  // ------------------------------------------------------------
  // Add sphere collision object.
  // Must match Python generator.
  // ------------------------------------------------------------

  constexpr double sphere_center_x = 0.55;
  constexpr double sphere_center_y = 0.00;
  constexpr double sphere_center_z = 0.25;
  constexpr double sphere_radius = 0.075;

  auto scene_client =
    node->create_client<moveit_msgs::srv::ApplyPlanningScene>(
      "/apply_planning_scene");

  if (!scene_client->wait_for_service(std::chrono::seconds(5)))
  {
    RCLCPP_ERROR(
      logger,
      "/apply_planning_scene is unavailable. Keep demo.launch.py running.");

    rclcpp::shutdown();
    return 1;
  }

  moveit_msgs::msg::CollisionObject sphere;
  sphere.header.frame_id = kPlanningFrame;
  sphere.id = "hemisphere_demo_sphere";

  shape_msgs::msg::SolidPrimitive primitive;
  primitive.type = shape_msgs::msg::SolidPrimitive::SPHERE;
  primitive.dimensions.resize(1);

  primitive.dimensions[
    shape_msgs::msg::SolidPrimitive::SPHERE_RADIUS] = sphere_radius;

  geometry_msgs::msg::Pose sphere_pose;
  sphere_pose.orientation.w = 1.0;
  sphere_pose.position.x = sphere_center_x;
  sphere_pose.position.y = sphere_center_y;
  sphere_pose.position.z = sphere_center_z;

  sphere.primitives.push_back(primitive);
  sphere.primitive_poses.push_back(sphere_pose);
  sphere.operation = moveit_msgs::msg::CollisionObject::ADD;

  auto scene_request =
    std::make_shared<moveit_msgs::srv::ApplyPlanningScene::Request>();

  scene_request->scene.is_diff = true;
  scene_request->scene.world.collision_objects.push_back(sphere);

  auto scene_future = scene_client->async_send_request(scene_request);

  if (
    rclcpp::spin_until_future_complete(
      node,
      scene_future,
      std::chrono::seconds(5)) != rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_ERROR(logger, "Timed out adding sphere to planning scene.");
    rclcpp::shutdown();
    return 1;
  }

  if (!scene_future.get()->success)
  {
    RCLCPP_ERROR(logger, "MoveIt rejected the sphere collision object.");
    rclcpp::shutdown();
    return 1;
  }

  RCLCPP_INFO(
    logger,
    "Sphere enabled: center=(%.3f, %.3f, %.3f), radius=%.3f m",
    sphere_center_x,
    sphere_center_y,
    sphere_center_z,
    sphere_radius);

  std::this_thread::sleep_for(std::chrono::milliseconds(750));

  // ------------------------------------------------------------
  // Plan normal collision-aware approach:
  // current robot state -> first ring point.
  // ------------------------------------------------------------

  moveit::planning_interface::MoveGroupInterface move_group(
    node,
    kPlanningGroup);

  move_group.setEndEffectorLink(kTcpLink);

  move_group.setPlanningTime(15.0);
  move_group.setNumPlanningAttempts(20);

  move_group.setMaxVelocityScalingFactor(0.10);
  move_group.setMaxAccelerationScalingFactor(0.10);

  move_group.setStartStateToCurrentState();
  move_group.setPoseTarget(approach_target, kTcpLink);

  moveit::planning_interface::MoveGroupInterface::Plan approach_plan;

  RCLCPP_INFO(
    logger,
    "Planning collision-aware OMPL approach to Start 00...");

  const auto approach_result = move_group.plan(approach_plan);

  move_group.clearPoseTargets();

  if (approach_result != moveit::core::MoveItErrorCode::SUCCESS)
  {
    RCLCPP_ERROR(
      logger,
      "FAILED: could not plan collision-aware approach to Start 00.");

    rclcpp::shutdown();
    return 1;
  }

  RCLCPP_INFO(
    logger,
    "Approach plan succeeded: %zu trajectory points.",
    approach_plan.trajectory.joint_trajectory.points.size());

  const auto & approach_trajectory =
    approach_plan.trajectory.joint_trajectory;

  if (approach_trajectory.points.empty())
  {
    RCLCPP_ERROR(
      logger,
      "Approach plan contains no trajectory points.");

    rclcpp::shutdown();
    return 1;
  }

  const auto & approach_final_point =
    approach_trajectory.points.back();

  if (
    approach_trajectory.joint_names.size() !=
    approach_final_point.positions.size())
  {
    RCLCPP_ERROR(
      logger,
      "Approach trajectory has mismatched joint names and positions.");

    rclcpp::shutdown();
    return 1;
  }

  // ------------------------------------------------------------
  // Build Cartesian start state from the final approach trajectory
  // point. Do NOT call getCurrentState() here: mock joint states
  // have timestamp 0 and MoveIt rejects them as stale.
  // ------------------------------------------------------------

  moveit::core::RobotState approach_end_state(
    move_group.getRobotModel());

  approach_end_state.setToDefaultValues();

  approach_end_state.setVariablePositions(
    approach_trajectory.joint_names,
    approach_final_point.positions);

  approach_end_state.update();

  move_group.setStartState(approach_end_state);

  // ------------------------------------------------------------
  // Create ring path.
  //
  // First CSV pose is excluded because approach_end_state already
  // represents that pose.
  // ------------------------------------------------------------

  std::vector<geometry_msgs::msg::Pose> ring_waypoints;
  ring_waypoints.reserve(path_points.size() - 1);

  for (std::size_t index = 1; index < path_points.size(); ++index)
  {
    ring_waypoints.push_back(path_points[index].pose);
  }

  moveit_msgs::msg::RobotTrajectory cartesian_trajectory;

  RCLCPP_INFO(
    logger,
    "Computing collision-aware Cartesian ring path with %zu waypoints...",
    ring_waypoints.size());

  const double fraction = move_group.computeCartesianPath(
    ring_waypoints,
    eef_step,
    jump_threshold,
    cartesian_trajectory,
    true);

  RCLCPP_INFO(
    logger,
    "Cartesian completion fraction: %.3f",
    fraction);

  RCLCPP_INFO(
    logger,
    "Cartesian trajectory points: %zu",
    cartesian_trajectory.joint_trajectory.points.size());

  if (fraction < 0.999)
  {
    RCLCPP_ERROR(
      logger,
      "Cartesian ring was incomplete. No execution performed.");

    rclcpp::shutdown();
    return 1;
  }

  RCLCPP_INFO(
    logger,
    "SUCCESS: full collision-aware Cartesian ring path found.");

  if (!execute)
  {
    RCLCPP_INFO(
      logger,
      "Planning-only test finished. "
      "Neither approach nor Cartesian path was executed.");

    rclcpp::shutdown();
    return 0;
  }

  // ------------------------------------------------------------
  // Execution remains opt-in.
  // ------------------------------------------------------------

  RCLCPP_WARN(
    logger,
    "Execution requested. Executing approach trajectory first.");

  const auto approach_execute_result =
    move_group.execute(approach_plan);

  if (approach_execute_result != moveit::core::MoveItErrorCode::SUCCESS)
  {
    RCLCPP_ERROR(logger, "Approach execution failed.");
    rclcpp::shutdown();
    return 1;
  }

  RCLCPP_WARN(
    logger,
    "Executing Cartesian ring trajectory.");

  const auto ring_execute_result =
    move_group.execute(cartesian_trajectory);

  if (ring_execute_result != moveit::core::MoveItErrorCode::SUCCESS)
  {
    RCLCPP_ERROR(logger, "Cartesian ring execution failed.");
    rclcpp::shutdown();
    return 1;
  }

  RCLCPP_INFO(
    logger,
    "Approach and Cartesian ring execution completed.");

  rclcpp::shutdown();
  return 0;
}
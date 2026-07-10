#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>

struct CandidateMotion
{
  std::string name;
  double dx;
  double dy;
  double dz;
};

struct TrajectoryMetrics
{
  bool valid = false;
  double total_duration = 0.0;
  double joint_space_distance = 0.0;
  size_t point_count = 0;
};

double durationToSeconds(const builtin_interfaces::msg::Duration& duration)
{
  return static_cast<double>(duration.sec) +
         static_cast<double>(duration.nanosec) * 1e-9;
}

TrajectoryMetrics analyzePlan(const moveit::planning_interface::MoveGroupInterface::Plan& plan)
{
  TrajectoryMetrics metrics;

  // In this user's ROS 2 Jazzy + MoveIt setup, use plan.trajectory, not plan.trajectory_.
  const auto& trajectory = plan.trajectory.joint_trajectory;
  const auto& points = trajectory.points;

  if (points.empty())
  {
    return metrics;
  }

  metrics.valid = true;
  metrics.point_count = points.size();
  metrics.total_duration = durationToSeconds(points.back().time_from_start);

  for (size_t i = 1; i < points.size(); ++i)
  {
    const auto& prev = points[i - 1];
    const auto& curr = points[i];

    double step_distance_squared = 0.0;

    for (size_t j = 0; j < curr.positions.size(); ++j)
    {
      const double delta = curr.positions[j] - prev.positions[j];
      step_distance_squared += delta * delta;
    }

    metrics.joint_space_distance += std::sqrt(step_distance_squared);
  }

  return metrics;
}

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<rclcpp::Node>(
      "compare_pose_candidates",
      rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

  auto logger = rclcpp::get_logger("compare_pose_candidates");

  // MoveIt needs this node to spin so it can receive /joint_states,
  // TF, planning scene updates, and action responses.
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);

  std::thread spinner([&executor]() {
    executor.spin();
  });

  static const std::string PLANNING_GROUP = "ur_manipulator";
  static const std::string END_EFFECTOR_LINK = "tool0";

  moveit::planning_interface::MoveGroupInterface move_group(node, PLANNING_GROUP);

  move_group.setEndEffectorLink(END_EFFECTOR_LINK);

  // Give the planner enough time for each candidate.
  move_group.setPlanningTime(10.0);

  // Keep execution slower and more visible in RViz.
  move_group.setMaxVelocityScalingFactor(0.15);
  move_group.setMaxAccelerationScalingFactor(0.15);

  // Relax tolerances a little so pose targets are not overly strict.
  move_group.setGoalPositionTolerance(0.01);
  move_group.setGoalOrientationTolerance(0.15);
  move_group.setGoalJointTolerance(0.01);

  RCLCPP_INFO(logger, "Planning frame: %s", move_group.getPlanningFrame().c_str());
  RCLCPP_INFO(logger, "End effector link: %s", move_group.getEndEffectorLink().c_str());

  // Let MoveIt receive current joint states.
  rclcpp::sleep_for(std::chrono::seconds(2));

  move_group.setStartStateToCurrentState();

  geometry_msgs::msg::Pose current_pose = move_group.getCurrentPose(END_EFFECTOR_LINK).pose;

  RCLCPP_INFO(logger, "Current tool0 pose:");
  RCLCPP_INFO(logger, "  x = %f", current_pose.position.x);
  RCLCPP_INFO(logger, "  y = %f", current_pose.position.y);
  RCLCPP_INFO(logger, "  z = %f", current_pose.position.z);

  // Larger visible movements.
  //
  // 0.10 means 10 cm.
  // This is for mock hardware/RViz visualization.
  // For a real robot, this should be reduced and reviewed for safety.
  std::vector<CandidateMotion> candidates = {
      {"+X_10cm", 0.10, 0.00, 0.00},
      {"-X_10cm", -0.10, 0.00, 0.00},
      {"+Y_10cm", 0.00, 0.10, 0.00},
      {"-Y_10cm", 0.00, -0.10, 0.00},
      {"+Z_8cm", 0.00, 0.00, 0.08},
      {"-Z_8cm", 0.00, 0.00, -0.08},
  };

  bool found_valid_plan = false;
  double best_score = std::numeric_limits<double>::infinity();
  std::string best_candidate_name;
  moveit::planning_interface::MoveGroupInterface::Plan best_plan;

  for (const auto& candidate : candidates)
  {
    RCLCPP_INFO(logger, "----------------------------------------");
    RCLCPP_INFO(logger, "Testing candidate: %s", candidate.name.c_str());

    geometry_msgs::msg::Pose target_pose = current_pose;
    target_pose.position.x += candidate.dx;
    target_pose.position.y += candidate.dy;
    target_pose.position.z += candidate.dz;

    RCLCPP_INFO(logger, "Target pose:");
    RCLCPP_INFO(logger, "  x = %f", target_pose.position.x);
    RCLCPP_INFO(logger, "  y = %f", target_pose.position.y);
    RCLCPP_INFO(logger, "  z = %f", target_pose.position.z);

    move_group.setStartStateToCurrentState();
    move_group.clearPoseTargets();

    // This is more forgiving than setPoseTarget().
    // It asks IK to find an approximate joint target for this tool0 pose.
    bool target_set =
        move_group.setApproximateJointValueTarget(target_pose, END_EFFECTOR_LINK);

    if (!target_set)
    {
      RCLCPP_WARN(logger, "Could not set approximate IK target for %s", candidate.name.c_str());
      continue;
    }

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool success = static_cast<bool>(move_group.plan(plan));

    if (!success)
    {
      RCLCPP_WARN(logger, "Planning failed for %s", candidate.name.c_str());
      continue;
    }

    TrajectoryMetrics metrics = analyzePlan(plan);

    if (!metrics.valid)
    {
      RCLCPP_WARN(logger, "Plan for %s had no trajectory points.", candidate.name.c_str());
      continue;
    }

    RCLCPP_INFO(logger, "Candidate succeeded: %s", candidate.name.c_str());
    RCLCPP_INFO(logger, "  points = %zu", metrics.point_count);
    RCLCPP_INFO(logger, "  duration = %.6f sec", metrics.total_duration);
    RCLCPP_INFO(logger, "  joint_space_distance = %.6f rad", metrics.joint_space_distance);

    // Simple score:
    // choose the trajectory with the smallest joint-space distance.
    //
    // Later, this score can include collision clearance, smoothness,
    // distance from singularities, joint limits, or surgical-task constraints.
    double score = metrics.joint_space_distance;

    if (score < best_score)
    {
      best_score = score;
      best_candidate_name = candidate.name;
      best_plan = plan;
      found_valid_plan = true;
    }
  }

  RCLCPP_INFO(logger, "========================================");

  if (!found_valid_plan)
  {
    RCLCPP_ERROR(logger, "No valid candidate plan was found.");

    executor.cancel();
    if (spinner.joinable())
    {
      spinner.join();
    }

    rclcpp::shutdown();
    return 1;
  }

  RCLCPP_INFO(logger, "Best candidate: %s", best_candidate_name.c_str());
  RCLCPP_INFO(logger, "Best score, joint-space distance: %.6f rad", best_score);

  // Execute the best trajectory so the tool movement is visible in RViz.
  RCLCPP_INFO(logger, "Executing best candidate. Watch tool0 in RViz.");

  auto result = move_group.execute(best_plan);

  if (static_cast<bool>(result))
  {
    RCLCPP_INFO(logger, "Execution succeeded.");
  }
  else
  {
    RCLCPP_ERROR(logger, "Execution failed.");
  }

  move_group.clearPoseTargets();

  executor.cancel();

  if (spinner.joinable())
  {
    spinner.join();
  }

  rclcpp::shutdown();
  return 0;
}
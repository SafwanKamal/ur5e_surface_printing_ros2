#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <numeric>
#include <string>
#include <thread>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>

double durationToSeconds(const builtin_interfaces::msg::Duration& duration)
{
  return static_cast<double>(duration.sec) +
         static_cast<double>(duration.nanosec) * 1e-9;
}

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<rclcpp::Node>(
      "analyze_trajectory",
      rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

  auto logger = rclcpp::get_logger("analyze_trajectory");

  // MoveIt needs the node to spin so it can receive /joint_states,
  // TF, planning scene updates, and action responses.
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);

  std::thread spinner([&executor]() {
    executor.spin();
  });

  static const std::string PLANNING_GROUP = "ur_manipulator";

  moveit::planning_interface::MoveGroupInterface move_group(node, PLANNING_GROUP);

  // Give the planner enough time for this simple test.
  move_group.setPlanningTime(10.0);

  // Keep speeds low. This matters more later for real hardware.
  move_group.setMaxVelocityScalingFactor(0.2);
  move_group.setMaxAccelerationScalingFactor(0.2);

  RCLCPP_INFO(logger, "Planning frame: %s", move_group.getPlanningFrame().c_str());
  RCLCPP_INFO(logger, "End effector link: %s", move_group.getEndEffectorLink().c_str());

  // Give MoveIt time to receive the current state from /joint_states.
  rclcpp::sleep_for(std::chrono::seconds(2));

  move_group.setStartStateToCurrentState();

  std::vector<double> joint_goal = move_group.getCurrentJointValues();

  if (joint_goal.size() < 6)
  {
    RCLCPP_ERROR(logger, "Expected at least 6 joints, got %zu", joint_goal.size());

    executor.cancel();
    if (spinner.joinable())
    {
      spinner.join();
    }

    rclcpp::shutdown();
    return 1;
  }

  RCLCPP_INFO(logger, "Current joint values:");
  for (size_t i = 0; i < joint_goal.size(); ++i)
  {
    RCLCPP_INFO(logger, "  joint[%zu] = %f", i, joint_goal[i]);
  }

  // Test target:
  // Move joint 0 by 0.2 rad.
  //
  // This is only a planning target. This node does NOT execute the trajectory.
  joint_goal[0] += 0.2;

  move_group.setJointValueTarget(joint_goal);

  moveit::planning_interface::MoveGroupInterface::Plan plan;

  bool success = static_cast<bool>(move_group.plan(plan));

  if (!success)
  {
    RCLCPP_ERROR(logger, "Planning failed.");

    executor.cancel();
    if (spinner.joinable())
    {
      spinner.join();
    }

    rclcpp::shutdown();
    return 1;
  }

  RCLCPP_INFO(logger, "Planning succeeded.");
  RCLCPP_INFO(logger, "Analyzing trajectory. This node will NOT execute it.");

  // In your ROS 2 Jazzy + MoveIt setup, the correct member is:
  // plan.trajectory
  //
  // Do not use plan.trajectory_ in this environment.
  const auto& trajectory = plan.trajectory.joint_trajectory;

  const auto& joint_names = trajectory.joint_names;
  const auto& points = trajectory.points;

  if (points.empty())
  {
    RCLCPP_ERROR(logger, "Trajectory has no points.");

    executor.cancel();
    if (spinner.joinable())
    {
      spinner.join();
    }

    rclcpp::shutdown();
    return 1;
  }

  const size_t num_joints = joint_names.size();
  const size_t num_points = points.size();

  RCLCPP_INFO(logger, "Number of joints: %zu", num_joints);
  RCLCPP_INFO(logger, "Number of trajectory points: %zu", num_points);

  RCLCPP_INFO(logger, "Joint names:");
  for (const auto& name : joint_names)
  {
    RCLCPP_INFO(logger, "  %s", name.c_str());
  }

  const double total_duration = durationToSeconds(points.back().time_from_start);

  RCLCPP_INFO(logger, "Total trajectory duration: %.6f seconds", total_duration);

  // Metrics:
  // 1. Total joint-space path length:
  //    Sum of Euclidean distances between consecutive joint-position vectors.
  //
  // 2. Per-joint absolute movement:
  //    Sum of absolute changes for each joint over the whole trajectory.
  //
  // 3. Max velocity per joint:
  //    Largest absolute velocity value observed for each joint.
  //
  // 4. Max single-step joint change:
  //    Largest absolute position jump between neighboring trajectory points.

  double total_joint_space_distance = 0.0;

  std::vector<double> per_joint_abs_movement(num_joints, 0.0);
  std::vector<double> max_abs_velocity(num_joints, 0.0);
  std::vector<double> max_single_step_change(num_joints, 0.0);

  for (size_t i = 1; i < num_points; ++i)
  {
    const auto& prev = points[i - 1];
    const auto& curr = points[i];

    double step_distance_squared = 0.0;

    for (size_t j = 0; j < num_joints; ++j)
    {
      const double delta = curr.positions[j] - prev.positions[j];
      const double abs_delta = std::abs(delta);

      step_distance_squared += delta * delta;

      per_joint_abs_movement[j] += abs_delta;
      max_single_step_change[j] = std::max(max_single_step_change[j], abs_delta);
    }

    total_joint_space_distance += std::sqrt(step_distance_squared);
  }

  for (const auto& point : points)
  {
    if (point.velocities.size() == num_joints)
    {
      for (size_t j = 0; j < num_joints; ++j)
      {
        max_abs_velocity[j] = std::max(max_abs_velocity[j], std::abs(point.velocities[j]));
      }
    }
  }

  RCLCPP_INFO(logger, "========== Trajectory Metrics ==========");
  RCLCPP_INFO(logger, "Total duration: %.6f seconds", total_duration);
  RCLCPP_INFO(logger, "Total joint-space distance: %.6f rad", total_joint_space_distance);

  if (total_duration > 0.0)
  {
    RCLCPP_INFO(logger,
                "Average joint-space speed: %.6f rad/s",
                total_joint_space_distance / total_duration);
  }

  RCLCPP_INFO(logger, "Per-joint movement summary:");

  for (size_t j = 0; j < num_joints; ++j)
  {
    RCLCPP_INFO(logger,
                "  %s: total_abs_movement=%.6f rad, max_abs_velocity=%.6f rad/s, max_step_change=%.6f rad",
                joint_names[j].c_str(),
                per_joint_abs_movement[j],
                max_abs_velocity[j],
                max_single_step_change[j]);
  }

  RCLCPP_INFO(logger, "First point time: %.6f sec", durationToSeconds(points.front().time_from_start));
  RCLCPP_INFO(logger, "Last point time: %.6f sec", durationToSeconds(points.back().time_from_start));

  RCLCPP_INFO(logger, "Trajectory analysis complete. No execution was performed.");

  executor.cancel();

  if (spinner.joinable())
  {
    spinner.join();
  }

  rclcpp::shutdown();
  return 0;
}
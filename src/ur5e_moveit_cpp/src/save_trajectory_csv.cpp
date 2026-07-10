#include <chrono>
#include <fstream>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<rclcpp::Node>(
      "save_trajectory_csv",
      rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

  auto logger = rclcpp::get_logger("save_trajectory_csv");

  // MoveIt needs the node to spin so it can receive /joint_states,
  // TF, planning scene updates, and action responses.
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);

  std::thread spinner([&executor]() {
    executor.spin();
  });

  static const std::string PLANNING_GROUP = "ur_manipulator";

  moveit::planning_interface::MoveGroupInterface move_group(node, PLANNING_GROUP);

  move_group.setPlanningTime(10.0);
  move_group.setMaxVelocityScalingFactor(0.2);
  move_group.setMaxAccelerationScalingFactor(0.2);

  RCLCPP_INFO(logger, "Planning frame: %s", move_group.getPlanningFrame().c_str());
  RCLCPP_INFO(logger, "End effector link: %s", move_group.getEndEffectorLink().c_str());

  // Give MoveIt time to receive current robot state.
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

  // Small test target. This plans only; it does not execute.
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

  RCLCPP_INFO(logger, "Planning succeeded. Saving trajectory to CSV...");

  // In your MoveIt/Jazzy environment, the correct member is `trajectory`,
  // not `trajectory_`.
  const auto& trajectory = plan.trajectory.joint_trajectory;

  if (trajectory.points.empty())
  {
    RCLCPP_ERROR(logger, "Trajectory has no points. Nothing to save.");

    executor.cancel();
    if (spinner.joinable())
    {
      spinner.join();
    }

    rclcpp::shutdown();
    return 1;
  }

  // Save to the home directory for easy access.
  const std::string output_path = "/home/safwan/ur5e_planned_trajectory.csv";

  std::ofstream csv_file(output_path);

  if (!csv_file.is_open())
  {
    RCLCPP_ERROR(logger, "Failed to open CSV file for writing: %s", output_path.c_str());

    executor.cancel();
    if (spinner.joinable())
    {
      spinner.join();
    }

    rclcpp::shutdown();
    return 1;
  }

  // Header row.
  csv_file << "point_index,time_from_start_sec";

  for (const auto& joint_name : trajectory.joint_names)
  {
    csv_file << "," << joint_name << "_position";
  }

  for (const auto& joint_name : trajectory.joint_names)
  {
    csv_file << "," << joint_name << "_velocity";
  }

  csv_file << "\n";

  // Data rows.
  for (size_t i = 0; i < trajectory.points.size(); ++i)
  {
    const auto& point = trajectory.points[i];

    double time_sec =
        static_cast<double>(point.time_from_start.sec) +
        static_cast<double>(point.time_from_start.nanosec) * 1e-9;

    csv_file << i << "," << time_sec;

    for (double position : point.positions)
    {
      csv_file << "," << position;
    }

    for (double velocity : point.velocities)
    {
      csv_file << "," << velocity;
    }

    csv_file << "\n";
  }

  csv_file.close();

  RCLCPP_INFO(logger, "Saved %zu trajectory points to:", trajectory.points.size());
  RCLCPP_INFO(logger, "%s", output_path.c_str());

  RCLCPP_INFO(logger, "This node did not execute the trajectory.");

  executor.cancel();

  if (spinner.joinable())
  {
    spinner.join();
  }

  rclcpp::shutdown();
  return 0;
}
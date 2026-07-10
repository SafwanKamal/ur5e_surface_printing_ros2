#include <chrono>
#include <memory>
#include <thread>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<rclcpp::Node>(
      "plan_only_trajectory",
      rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

  auto logger = rclcpp::get_logger("plan_only_trajectory");

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

  RCLCPP_INFO(logger, "Current joint values:");
  for (size_t i = 0; i < joint_goal.size(); ++i)
  {
    RCLCPP_INFO(logger, "  joint[%zu] = %f", i, joint_goal[i]);
  }

  // Create a small joint-space target.
  // This is plan-only: the robot will NOT move.
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
  RCLCPP_INFO(logger, "This node will NOT execute the trajectory.");

  const auto& trajectory = plan.trajectory.joint_trajectory;

  RCLCPP_INFO(logger, "Trajectory joint names:");
  for (const auto& name : trajectory.joint_names)
  {
    RCLCPP_INFO(logger, "  %s", name.c_str());
  }

  RCLCPP_INFO(logger, "Number of trajectory points: %zu", trajectory.points.size());

  for (size_t i = 0; i < trajectory.points.size(); ++i)
  {
    const auto& point = trajectory.points[i];

    double time_sec =
        static_cast<double>(point.time_from_start.sec) +
        static_cast<double>(point.time_from_start.nanosec) * 1e-9;

    RCLCPP_INFO(logger, "Point %zu:", i);
    RCLCPP_INFO(logger, "  time_from_start = %.6f seconds", time_sec);

    RCLCPP_INFO(logger, "  positions:");
    for (size_t j = 0; j < point.positions.size(); ++j)
    {
      RCLCPP_INFO(logger, "    %s = %f",
                  trajectory.joint_names[j].c_str(),
                  point.positions[j]);
    }

    if (!point.velocities.empty())
    {
      RCLCPP_INFO(logger, "  velocities:");
      for (size_t j = 0; j < point.velocities.size(); ++j)
      {
        RCLCPP_INFO(logger, "    %s = %f",
                    trajectory.joint_names[j].c_str(),
                    point.velocities[j]);
      }
    }
  }

  executor.cancel();

  if (spinner.joinable())
  {
    spinner.join();
  }

  rclcpp::shutdown();
  return 0;
}
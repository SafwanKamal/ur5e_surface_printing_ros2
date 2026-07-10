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
      "simple_joint_motion",
      rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

  auto logger = rclcpp::get_logger("simple_joint_motion");



  // Important: MoveGroupInterface needs the node to spin so it can receive
  // /joint_states, TF, planning scene updates, etc.
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);

  std::thread spinner([&executor]() {
    executor.spin();
  });

  static const std::string PLANNING_GROUP = "ur_manipulator";

  moveit::planning_interface::MoveGroupInterface move_group(node, PLANNING_GROUP);

  move_group.setMaxVelocityScalingFactor(0.1);
  move_group.setMaxAccelerationScalingFactor(0.1);

  RCLCPP_INFO(logger, "Planning frame: %s", move_group.getPlanningFrame().c_str());
  RCLCPP_INFO(logger, "End effector link: %s", move_group.getEndEffectorLink().c_str());

  // Give MoveIt time to receive the current robot state from /joint_states.
  rclcpp::sleep_for(std::chrono::seconds(2));

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

  // Small mock-hardware test motion.
  joint_goal[0] += 0.8;

  move_group.setStartStateToCurrentState();
  move_group.setJointValueTarget(joint_goal);

  moveit::planning_interface::MoveGroupInterface::Plan plan;

  bool success = static_cast<bool>(move_group.plan(plan));

  if (success)
  {
    RCLCPP_INFO(logger, "Planning succeeded. Executing...");
    move_group.execute(plan);
  }
  else
  {
    RCLCPP_ERROR(logger, "Planning failed.");
  }

  executor.cancel();

  if (spinner.joinable())
  {
    spinner.join();
  }

  rclcpp::shutdown();
  return 0;
}
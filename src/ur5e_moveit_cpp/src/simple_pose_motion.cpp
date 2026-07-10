#include <chrono>
#include <memory>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<rclcpp::Node>(
      "simple_pose_motion",
      rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

  auto logger = rclcpp::get_logger("simple_pose_motion");

  // MoveGroupInterface needs the node to spin so it can receive joint states,
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
  move_group.setPlanningTime(10.0);
  move_group.setMaxVelocityScalingFactor(0.2);
  move_group.setMaxAccelerationScalingFactor(0.2);

  RCLCPP_INFO(logger, "Planning frame: %s", move_group.getPlanningFrame().c_str());
  RCLCPP_INFO(logger, "End effector link: %s", move_group.getEndEffectorLink().c_str());

  // Give MoveIt time to receive /joint_states.
  rclcpp::sleep_for(std::chrono::seconds(2));

  move_group.setStartStateToCurrentState();

  geometry_msgs::msg::Pose current_pose = move_group.getCurrentPose(END_EFFECTOR_LINK).pose;

  RCLCPP_INFO(logger, "Current tool0 pose:");
  RCLCPP_INFO(logger, "  position.x = %f", current_pose.position.x);
  RCLCPP_INFO(logger, "  position.y = %f", current_pose.position.y);
  RCLCPP_INFO(logger, "  position.z = %f", current_pose.position.z);
  RCLCPP_INFO(logger, "  orientation.x = %f", current_pose.orientation.x);
  RCLCPP_INFO(logger, "  orientation.y = %f", current_pose.orientation.y);
  RCLCPP_INFO(logger, "  orientation.z = %f", current_pose.orientation.z);
  RCLCPP_INFO(logger, "  orientation.w = %f", current_pose.orientation.w);

  geometry_msgs::msg::Pose target_pose = current_pose;

  // Small test motion: move tool0 upward by 5 cm.
  // Keep this small while testing.
  target_pose.position.x += 0.02;

  RCLCPP_INFO(logger, "Target tool0 pose:");
  RCLCPP_INFO(logger, "  position.x = %f", target_pose.position.x);
  RCLCPP_INFO(logger, "  position.y = %f", target_pose.position.y);
  RCLCPP_INFO(logger, "  position.z = %f", target_pose.position.z);

move_group.setGoalPositionTolerance(0.01);
move_group.setGoalOrientationTolerance(0.10);
move_group.setGoalJointTolerance(0.01);

// More forgiving than setPoseTarget() for this first test.
// It asks IK for a nearby joint solution for the desired tool0 pose,
// then plans to that joint-space target.
bool target_set = move_group.setApproximateJointValueTarget(target_pose, END_EFFECTOR_LINK);

if (!target_set)
{
  RCLCPP_ERROR(logger, "Failed to find an approximate IK joint target for the requested pose.");

  executor.cancel();
  if (spinner.joinable())
  {
    spinner.join();
  }

  rclcpp::shutdown();
  return 1;
}

  moveit::planning_interface::MoveGroupInterface::Plan plan;

  bool success = static_cast<bool>(move_group.plan(plan));

  if (success)
  {
    RCLCPP_INFO(logger, "Pose planning succeeded. Executing...");
    move_group.execute(plan);
  }
  else
  {
    RCLCPP_ERROR(logger, "Pose planning failed.");
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
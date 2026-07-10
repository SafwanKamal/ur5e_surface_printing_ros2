#include <chrono>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.hpp>

#include <moveit_msgs/msg/collision_object.hpp>
#include <moveit_msgs/msg/planning_scene.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <geometry_msgs/msg/pose.hpp>

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<rclcpp::Node>(
      "collision_object_demo",
      rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

  auto logger = rclcpp::get_logger("collision_object_demo");

  // MoveIt needs this node to spin so it can receive /joint_states,
  // TF, planning scene updates, and action responses.
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);

  std::thread spinner([&executor]() {
    executor.spin();
  });

  static const std::string PLANNING_GROUP = "ur_manipulator";

  moveit::planning_interface::MoveGroupInterface move_group(node, PLANNING_GROUP);
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

  move_group.setPlanningTime(10.0);
  move_group.setMaxVelocityScalingFactor(0.2);
  move_group.setMaxAccelerationScalingFactor(0.2);

  const std::string planning_frame = move_group.getPlanningFrame();

  RCLCPP_INFO(logger, "Planning frame: %s", planning_frame.c_str());
  RCLCPP_INFO(logger, "End effector link: %s", move_group.getEndEffectorLink().c_str());

  // Publisher used for the RViz visualization fix.
  //
  // In our setup, MoveIt knew the object existed, but RViz did not show it
  // until we also published a PlanningScene diff directly to this topic.
  auto rviz_scene_publisher =
      node->create_publisher<moveit_msgs::msg::PlanningScene>(
          "/monitored_planning_scene", 10);

  // Give MoveIt/RViz time to connect.
  rclcpp::sleep_for(std::chrono::seconds(2));

  const std::string box_id = "demo_box_obstacle";

  // ------------------------------------------------------------
  // 1. Remove old copy of the object if it exists
  // ------------------------------------------------------------

  RCLCPP_INFO(logger, "Removing old object if it exists: %s", box_id.c_str());

  planning_scene_interface.removeCollisionObjects({box_id});

  rclcpp::sleep_for(std::chrono::seconds(1));

  // ------------------------------------------------------------
  // 2. Create a visible collision box
  // ------------------------------------------------------------

  moveit_msgs::msg::CollisionObject box;
  box.header.frame_id = planning_frame;
  box.id = box_id;

  shape_msgs::msg::SolidPrimitive primitive;
  primitive.type = shape_msgs::msg::SolidPrimitive::BOX;

  // Big enough to be obvious in RViz.
  primitive.dimensions.resize(3);
  primitive.dimensions[shape_msgs::msg::SolidPrimitive::BOX_X] = 0.60;
  primitive.dimensions[shape_msgs::msg::SolidPrimitive::BOX_Y] = 0.60;
  primitive.dimensions[shape_msgs::msg::SolidPrimitive::BOX_Z] = 0.60;

  geometry_msgs::msg::Pose box_pose;
  box_pose.orientation.w = 1.0;

  // z = 0.30 with height = 0.60 means the box bottom is near z = 0.0.
  // Adjust x/y later depending on where you want the obstacle.
  box_pose.position.x = 0.70;
  box_pose.position.y = 0.00;
  box_pose.position.z = 0.30;

  box.primitives.push_back(primitive);
  box.primitive_poses.push_back(box_pose);
  box.operation = moveit_msgs::msg::CollisionObject::ADD;

  RCLCPP_INFO(logger, "Adding collision object: %s", box.id.c_str());
  RCLCPP_INFO(logger, "Box frame: %s", box.header.frame_id.c_str());
  RCLCPP_INFO(logger, "Box size: 0.60 x 0.60 x 0.60 m");
  RCLCPP_INFO(logger,
              "Box position: x=%f y=%f z=%f",
              box_pose.position.x,
              box_pose.position.y,
              box_pose.position.z);

  // ------------------------------------------------------------
  // 3. Add object to MoveIt's planning scene
  // ------------------------------------------------------------

  std::vector<moveit_msgs::msg::CollisionObject> collision_objects;
  collision_objects.push_back(box);

  planning_scene_interface.addCollisionObjects(collision_objects);

  RCLCPP_INFO(logger, "Box add request sent to MoveIt.");

  rclcpp::sleep_for(std::chrono::seconds(2));

  auto known_objects = planning_scene_interface.getKnownObjectNames();

  RCLCPP_INFO(logger, "Known collision objects after adding box:");
  for (const auto& object_name : known_objects)
  {
    RCLCPP_INFO(logger, "  %s", object_name.c_str());
  }

  // ------------------------------------------------------------
  // 4. RViz visualization fix
  // ------------------------------------------------------------
  //
  // This was the part that made the box visible in your setup.
  // MoveIt knew the object existed, but RViz did not display it until
  // we directly published a PlanningScene diff to /monitored_planning_scene.

  moveit_msgs::msg::PlanningScene scene_diff;
  scene_diff.is_diff = true;
  scene_diff.world.collision_objects.push_back(box);

  RCLCPP_INFO(logger, "Publishing scene diff directly to /monitored_planning_scene for RViz.");

  for (int i = 0; i < 40; ++i)
  {
    rviz_scene_publisher->publish(scene_diff);
    rclcpp::sleep_for(std::chrono::milliseconds(250));
  }

  RCLCPP_INFO(logger, "Published RViz scene diff.");
  RCLCPP_INFO(logger, "Waiting 5 seconds so you can see the box before planning.");

  rclcpp::sleep_for(std::chrono::seconds(5));

  // ------------------------------------------------------------
  // 5. Plan a joint-space motion while the box exists
  // ------------------------------------------------------------

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

  // Visible mock-hardware test motion.
  //
  // Since this is a joint-space goal, this node does not need kinematics.yaml.
  // For real hardware later, reduce motion size and follow lab safety rules.
  joint_goal[0] += 0.4;
  joint_goal[1] += 0.2;

  move_group.setJointValueTarget(joint_goal);

  moveit::planning_interface::MoveGroupInterface::Plan plan;

  bool success = static_cast<bool>(move_group.plan(plan));

  if (success)
  {
    RCLCPP_INFO(logger, "Planning succeeded with collision object present.");
    RCLCPP_INFO(logger, "Executing trajectory. Watch RViz.");

    auto result = move_group.execute(plan);

    if (static_cast<bool>(result))
    {
      RCLCPP_INFO(logger, "Execution succeeded.");
    }
    else
    {
      RCLCPP_ERROR(logger, "Execution failed.");
    }
  }
  else
  {
    RCLCPP_ERROR(logger, "Planning failed with collision object present.");
  }

  RCLCPP_INFO(logger, "Leaving collision object in the scene for inspection.");
  RCLCPP_INFO(logger, "Waiting 60 seconds.");

  rclcpp::sleep_for(std::chrono::seconds(60));

  executor.cancel();

  if (spinner.joinable())
  {
    spinner.join();
  }

  rclcpp::shutdown();
  return 0;
}
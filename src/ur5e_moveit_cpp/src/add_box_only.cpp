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
      "add_box_only",
      rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

  auto logger = rclcpp::get_logger("add_box_only");

  // Spin the node so MoveIt/RViz communication can happen.
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);

  std::thread spinner([&executor]() {
    executor.spin();
  });

  static const std::string PLANNING_GROUP = "ur_manipulator";

  moveit::planning_interface::MoveGroupInterface move_group(node, PLANNING_GROUP);
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

  const std::string planning_frame = move_group.getPlanningFrame();

  RCLCPP_INFO(logger, "Planning frame: %s", planning_frame.c_str());

  // This publisher sends the planning scene diff directly to RViz's topic.
  // Your RViz MotionPlanning display is subscribed to /monitored_planning_scene.
  auto rviz_scene_publisher =
      node->create_publisher<moveit_msgs::msg::PlanningScene>(
          "/monitored_planning_scene", 10);

  // Give MoveIt and RViz time to connect.
  rclcpp::sleep_for(std::chrono::seconds(2));

  const std::string box_id = "BIG_VISIBLE_TEST_BOX";

  // ------------------------------------------------------------
  // Remove old object if it exists
  // ------------------------------------------------------------

  RCLCPP_INFO(logger, "Removing old object if it exists: %s", box_id.c_str());

  planning_scene_interface.removeCollisionObjects({box_id});

  rclcpp::sleep_for(std::chrono::seconds(1));

  // ------------------------------------------------------------
  // Create a huge visible box
  // ------------------------------------------------------------

  moveit_msgs::msg::CollisionObject box;
  box.header.frame_id = planning_frame;
  box.id = box_id;

  shape_msgs::msg::SolidPrimitive primitive;
  primitive.type = shape_msgs::msg::SolidPrimitive::BOX;

  // Very large box: 80 cm cube.
  primitive.dimensions.resize(3);
  primitive.dimensions[shape_msgs::msg::SolidPrimitive::BOX_X] = 0.20;
  primitive.dimensions[shape_msgs::msg::SolidPrimitive::BOX_Y] = 0.20;
  primitive.dimensions[shape_msgs::msg::SolidPrimitive::BOX_Z] = 0.20;

  geometry_msgs::msg::Pose box_pose;
  box_pose.orientation.w = 1.0;

  // Since height is 0.80 m and z is 0.40 m,
  // the bottom sits at z = 0.0 and the top at z = 0.8.
  box_pose.position.x = 0.8;
  box_pose.position.y = 0.20;
  box_pose.position.z = 0.40;

  box.primitives.push_back(primitive);
  box.primitive_poses.push_back(box_pose);
  box.operation = moveit_msgs::msg::CollisionObject::ADD;

  RCLCPP_INFO(logger, "Adding collision object: %s", box.id.c_str());
  RCLCPP_INFO(logger, "Box frame: %s", box.header.frame_id.c_str());
  RCLCPP_INFO(logger, "Box size: 0.80 x 0.80 x 0.80 m");
  RCLCPP_INFO(logger,
              "Box position: x=%f y=%f z=%f",
              box_pose.position.x,
              box_pose.position.y,
              box_pose.position.z);

  // ------------------------------------------------------------
  // Part 1: Add object to MoveIt's planning scene
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
  // Part 2: Also publish the planning scene diff directly to RViz
  // ------------------------------------------------------------

  moveit_msgs::msg::PlanningScene scene_diff;
  scene_diff.is_diff = true;
  scene_diff.world.collision_objects.push_back(box);

  RCLCPP_INFO(logger, "Publishing scene diff directly to /monitored_planning_scene for RViz.");

  // Publish repeatedly so RViz definitely receives the update.
  for (int i = 0; i < 40; ++i)
  {
    rviz_scene_publisher->publish(scene_diff);
    rclcpp::sleep_for(std::chrono::milliseconds(250));
  }

  RCLCPP_INFO(logger, "Published RViz scene diff.");
  RCLCPP_INFO(logger, "Waiting 60 seconds. Check RViz now.");

  rclcpp::sleep_for(std::chrono::seconds(60));

  executor.cancel();

  if (spinner.joinable())
  {
    spinner.join();
  }

  rclcpp::shutdown();
  return 0;
}
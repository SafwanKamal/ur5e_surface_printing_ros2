#include <chrono>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <moveit_msgs/msg/planning_scene.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);

//   auto node = std::make_shared<rclcpp::Node>(
//       "parameterized_shape_adder",
//       rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));
  auto node = std::make_shared<rclcpp::Node>("parameterized_shape_adder");
  auto logger = rclcpp::get_logger("parameterized_shape_adder");

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);

  std::thread spinner([&executor]() {
    executor.spin();
  });

  const std::string shape_type = node->declare_parameter<std::string>("shape_type", "box");
  const std::string object_id = node->declare_parameter<std::string>("object_id", "terminal_shape");
  const std::string frame_id = node->declare_parameter<std::string>("frame_id", "base_link");

  const double center_x = node->declare_parameter<double>("center_x", 0.50);
  const double center_y = node->declare_parameter<double>("center_y", 0.00);
  const double center_z = node->declare_parameter<double>("center_z", 0.30);

  const double box_x = node->declare_parameter<double>("box_x", 0.20);
  const double box_y = node->declare_parameter<double>("box_y", 0.20);
  const double box_z = node->declare_parameter<double>("box_z", 0.20);

  const double radius = node->declare_parameter<double>("radius", 0.10);
  const double height = node->declare_parameter<double>("height", 0.30);

  shape_msgs::msg::SolidPrimitive primitive;

  if (shape_type == "box")
  {
    primitive.type = shape_msgs::msg::SolidPrimitive::BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[shape_msgs::msg::SolidPrimitive::BOX_X] = box_x;
    primitive.dimensions[shape_msgs::msg::SolidPrimitive::BOX_Y] = box_y;
    primitive.dimensions[shape_msgs::msg::SolidPrimitive::BOX_Z] = box_z;
  }
  else if (shape_type == "cylinder")
  {
    primitive.type = shape_msgs::msg::SolidPrimitive::CYLINDER;
    primitive.dimensions.resize(2);
    primitive.dimensions[shape_msgs::msg::SolidPrimitive::CYLINDER_HEIGHT] = height;
    primitive.dimensions[shape_msgs::msg::SolidPrimitive::CYLINDER_RADIUS] = radius;
  }
  else if (shape_type == "sphere")
  {
    primitive.type = shape_msgs::msg::SolidPrimitive::SPHERE;
    primitive.dimensions.resize(1);
    primitive.dimensions[shape_msgs::msg::SolidPrimitive::SPHERE_RADIUS] = radius;
  }
  else
  {
    RCLCPP_ERROR(logger, "Unsupported shape_type: %s", shape_type.c_str());
    RCLCPP_ERROR(logger, "Use shape_type:=box, shape_type:=cylinder, or shape_type:=sphere");
    executor.cancel();
    if (spinner.joinable()) spinner.join();
    rclcpp::shutdown();
    return 1;
  }

  geometry_msgs::msg::Pose pose;
  pose.orientation.w = 1.0;
  pose.position.x = center_x;
  pose.position.y = center_y;
  pose.position.z = center_z;

  moveit_msgs::msg::CollisionObject object;
  object.header.frame_id = frame_id;
  object.id = object_id;
  object.primitives.push_back(primitive);
  object.primitive_poses.push_back(pose);
  object.operation = moveit_msgs::msg::CollisionObject::ADD;

  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

  auto rviz_scene_publisher =
      node->create_publisher<moveit_msgs::msg::PlanningScene>(
          "/monitored_planning_scene", 10);

  rclcpp::sleep_for(std::chrono::seconds(2));

  planning_scene_interface.applyCollisionObjects({object});

  moveit_msgs::msg::PlanningScene scene_diff;
  scene_diff.is_diff = true;
  scene_diff.world.collision_objects.push_back(object);

  for (int i = 0; i < 5; ++i)
  {
    rviz_scene_publisher->publish(scene_diff);
    rclcpp::sleep_for(std::chrono::milliseconds(200));
  }

  RCLCPP_INFO(logger, "Added %s object '%s' in frame '%s'.", shape_type.c_str(), object_id.c_str(), frame_id.c_str());
  RCLCPP_INFO(logger, "Center: x=%f, y=%f, z=%f", center_x, center_y, center_z);

  executor.cancel();
  if (spinner.joinable()) spinner.join();
  rclcpp::shutdown();
  return 0;
}
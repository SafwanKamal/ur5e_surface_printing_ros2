#include <chrono>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <moveit_msgs/msg/planning_scene.hpp>

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<rclcpp::Node>(
      "remove_all_objects",
      rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

  auto logger = rclcpp::get_logger("remove_all_objects");

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);

  std::thread spinner([&executor]() {
    executor.spin();
  });

  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

  auto rviz_scene_publisher =
      node->create_publisher<moveit_msgs::msg::PlanningScene>(
          "/monitored_planning_scene", 10);

  rclcpp::sleep_for(std::chrono::seconds(2));

  std::vector<std::string> object_ids = planning_scene_interface.getKnownObjectNames();

  if (object_ids.empty())
  {
    RCLCPP_INFO(logger, "No known collision objects found in MoveIt.");
  }
  else
  {
    RCLCPP_INFO(logger, "Removing %zu known collision object(s):", object_ids.size());
    for (const auto& id : object_ids)
    {
      RCLCPP_INFO(logger, "  %s", id.c_str());
    }

    // Remove objects from MoveIt's planning scene.
    planning_scene_interface.removeCollisionObjects(object_ids);

    rclcpp::sleep_for(std::chrono::seconds(1));

    // Also publish REMOVE diffs so RViz clears the scene geometry.
    // This is useful when RViz does not visually update from the normal interface alone.
    moveit_msgs::msg::PlanningScene scene_diff;
    scene_diff.is_diff = true;

    for (const auto& id : object_ids)
    {
      moveit_msgs::msg::CollisionObject remove_object;
      remove_object.id = id;
      remove_object.operation = moveit_msgs::msg::CollisionObject::REMOVE;
      scene_diff.world.collision_objects.push_back(remove_object);
    }

    for (int i = 0; i < 5; ++i)
    {
      rviz_scene_publisher->publish(scene_diff);
      rclcpp::sleep_for(std::chrono::milliseconds(200));
    }

    RCLCPP_INFO(logger, "Cleanup message published. MoveIt and RViz should now be clear of added objects.");
  }

  executor.cancel();
  if (spinner.joinable()) spinner.join();
  rclcpp::shutdown();
  return 0;
}
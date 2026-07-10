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
#include <moveit/planning_scene_interface/planning_scene_interface.hpp>

#include <moveit_msgs/msg/collision_object.hpp>
#include <moveit_msgs/msg/planning_scene.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>

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

  // In our ROS 2 Jazzy + MoveIt setup, use plan.trajectory, not plan.trajectory_.
  const auto& trajectory = plan.trajectory.joint_trajectory;
  const auto& points = trajectory.points;

  if (points.empty())
  {
    return metrics;
  }

  metrics.valid = true;
  metrics.point_count = points.size();
  metrics.total_duration = durationToSeconds(points.back().time_from_start);

  // Total joint-space distance:
  // Sum of Euclidean distances between neighboring joint-position vectors.
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

void publishSceneDiffForRviz(
    const rclcpp::Publisher<moveit_msgs::msg::PlanningScene>::SharedPtr& publisher,
    const moveit_msgs::msg::CollisionObject& object)
{
  moveit_msgs::msg::PlanningScene scene_diff;
  scene_diff.is_diff = true;
  scene_diff.world.collision_objects.push_back(object);

  // In a clean setup, one publish should be enough.
  // In our RViz setup, publishing a few times made visualization reliable.
  rclcpp::sleep_for(std::chrono::seconds(1));

  for (int i = 0; i < 5; ++i)
  {
    publisher->publish(scene_diff);
    rclcpp::sleep_for(std::chrono::milliseconds(200));
  }
}

moveit_msgs::msg::CollisionObject makeBoxObstacle(
    const std::string& planning_frame,
    const std::string& box_id)
{
  moveit_msgs::msg::CollisionObject box;
  box.header.frame_id = planning_frame;
  box.id = box_id;

  shape_msgs::msg::SolidPrimitive primitive;
  primitive.type = shape_msgs::msg::SolidPrimitive::BOX;

  // Large visible box.
  // Dimensions are in meters.
  primitive.dimensions.resize(3);
  primitive.dimensions[shape_msgs::msg::SolidPrimitive::BOX_X] = 0.30;
  primitive.dimensions[shape_msgs::msg::SolidPrimitive::BOX_Y] = 0.30;
  primitive.dimensions[shape_msgs::msg::SolidPrimitive::BOX_Z] = 0.30;

  geometry_msgs::msg::Pose box_pose;
  box_pose.orientation.w = 1.0;

  // Since z = 0.30 and height = 0.60, the bottom is around z = 0.0.
  // Adjust this later when building more realistic workspaces.
  box_pose.position.x = 0.80;
  box_pose.position.y = 0.00;
  box_pose.position.z = 0.30;

  box.primitives.push_back(primitive);
  box.primitive_poses.push_back(box_pose);
  box.operation = moveit_msgs::msg::CollisionObject::ADD;

  return box;
}

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<rclcpp::Node>(
      "collision_aware_candidate_planner",
      rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

  auto logger = rclcpp::get_logger("collision_aware_candidate_planner");

  // MoveIt needs this node to spin so it can receive:
  // /joint_states, TF, planning scene updates, and action responses.
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);

  std::thread spinner([&executor]() {
    executor.spin();
  });

  static const std::string PLANNING_GROUP = "ur_manipulator";
  static const std::string END_EFFECTOR_LINK = "tool0";

  moveit::planning_interface::MoveGroupInterface move_group(node, PLANNING_GROUP);
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

  move_group.setEndEffectorLink(END_EFFECTOR_LINK);

  // Planning settings.
  move_group.setPlanningTime(10.0);
  move_group.setMaxVelocityScalingFactor(0.15);
  move_group.setMaxAccelerationScalingFactor(0.15);

  // Pose target tolerances.
  move_group.setGoalPositionTolerance(0.01);
  move_group.setGoalOrientationTolerance(0.15);
  move_group.setGoalJointTolerance(0.01);

  const std::string planning_frame = move_group.getPlanningFrame();

  RCLCPP_INFO(logger, "Planning frame: %s", planning_frame.c_str());
  RCLCPP_INFO(logger, "End effector link: %s", move_group.getEndEffectorLink().c_str());

  // Publisher for the RViz visualization fix.
  //
  // In our setup, MoveIt knew the collision object existed, but RViz did not
  // show it until we also published a PlanningScene diff directly to
  // /monitored_planning_scene.
  auto rviz_scene_publisher =
      node->create_publisher<moveit_msgs::msg::PlanningScene>(
          "/monitored_planning_scene", 10);

  // Let MoveIt/RViz connections settle.
  rclcpp::sleep_for(std::chrono::seconds(2));

  // ------------------------------------------------------------
  // 1. Add collision object
  // ------------------------------------------------------------

  const std::string box_id = "candidate_planner_box";

  RCLCPP_INFO(logger, "Removing old object if it exists: %s", box_id.c_str());
  planning_scene_interface.removeCollisionObjects({box_id});
  rclcpp::sleep_for(std::chrono::seconds(1));

  moveit_msgs::msg::CollisionObject box = makeBoxObstacle(planning_frame, box_id);

  RCLCPP_INFO(logger, "Adding collision object: %s", box.id.c_str());
  RCLCPP_INFO(logger, "Box frame: %s", box.header.frame_id.c_str());

  if (!box.primitive_poses.empty())
  {
    const auto& pose = box.primitive_poses.front();
    RCLCPP_INFO(logger,
                "Box position: x=%f y=%f z=%f",
                pose.position.x,
                pose.position.y,
                pose.position.z);
  }

  std::vector<moveit_msgs::msg::CollisionObject> collision_objects;
  collision_objects.push_back(box);

  // Add the object to MoveIt's planning scene so planning/collision checking sees it.
  planning_scene_interface.addCollisionObjects(collision_objects);

  rclcpp::sleep_for(std::chrono::seconds(2));

  auto known_objects = planning_scene_interface.getKnownObjectNames();

  RCLCPP_INFO(logger, "Known collision objects after adding box:");
  for (const auto& object_name : known_objects)
  {
    RCLCPP_INFO(logger, "  %s", object_name.c_str());
  }

  // Also publish directly to RViz so the box is visible.
  RCLCPP_INFO(logger, "Publishing scene diff to RViz.");
  publishSceneDiffForRviz(rviz_scene_publisher, box);

  RCLCPP_INFO(logger, "Waiting 3 seconds so you can see the box in RViz...");
  rclcpp::sleep_for(std::chrono::seconds(3));

  // ------------------------------------------------------------
  // 2. Read current tool pose
  // ------------------------------------------------------------

  move_group.setStartStateToCurrentState();

  geometry_msgs::msg::Pose current_pose = move_group.getCurrentPose(END_EFFECTOR_LINK).pose;

  RCLCPP_INFO(logger, "Current tool0 pose:");
  RCLCPP_INFO(logger, "  x = %f", current_pose.position.x);
  RCLCPP_INFO(logger, "  y = %f", current_pose.position.y);
  RCLCPP_INFO(logger, "  z = %f", current_pose.position.z);

  // ------------------------------------------------------------
  // 3. Generate candidate tool motions
  // ------------------------------------------------------------
  //
  // These are visible 8-12 cm movements.
  // Some may fail because of IK, collision, or planning constraints.
  // That is expected; the node only chooses among successful plans.

  std::vector<CandidateMotion> candidates = {
      {"+X_12cm", 0.12, 0.00, 0.00},
      {"-X_12cm", -0.12, 0.00, 0.00},
      {"+Y_12cm", 0.00, 0.12, 0.00},
      {"-Y_12cm", 0.00, -0.12, 0.00},
      {"+Z_10cm", 0.00, 0.00, 0.10},
      {"-Z_10cm", 0.00, 0.00, -0.10},
  };

  bool found_valid_plan = false;
  double best_score = std::numeric_limits<double>::infinity();
  std::string best_candidate_name;
  TrajectoryMetrics best_metrics;
  moveit::planning_interface::MoveGroupInterface::Plan best_plan;

  // ------------------------------------------------------------
  // 4. Plan each candidate and score successful ones
  // ------------------------------------------------------------

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

    // Pose goals require IK.
    //
    // This uses the KDL IK plugin loaded from kinematics.yaml.
    // It is more forgiving than setPoseTarget() for this beginner workflow.
    bool target_set =
        move_group.setApproximateJointValueTarget(target_pose, END_EFFECTOR_LINK);

    if (!target_set)
    {
      RCLCPP_WARN(logger,
                  "Could not find approximate IK target for %s",
                  candidate.name.c_str());
      continue;
    }

    moveit::planning_interface::MoveGroupInterface::Plan plan;

    bool success = static_cast<bool>(move_group.plan(plan));

    if (!success)
    {
      RCLCPP_WARN(logger,
                  "Planning failed for %s. It may be blocked by collision, IK, or constraints.",
                  candidate.name.c_str());
      continue;
    }

    TrajectoryMetrics metrics = analyzePlan(plan);

    if (!metrics.valid)
    {
      RCLCPP_WARN(logger,
                  "Plan for %s had no trajectory points.",
                  candidate.name.c_str());
      continue;
    }

    RCLCPP_INFO(logger, "Candidate succeeded: %s", candidate.name.c_str());
    RCLCPP_INFO(logger, "  points = %zu", metrics.point_count);
    RCLCPP_INFO(logger, "  duration = %.6f sec", metrics.total_duration);
    RCLCPP_INFO(logger, "  joint_space_distance = %.6f rad", metrics.joint_space_distance);

    // Current simple score:
    // choose the valid trajectory with the smallest total joint-space distance.
    //
    // Later, this can include:
    // - obstacle clearance
    // - smoothness / jerk
    // - distance from joint limits
    // - distance from singularities
    // - duration
    // - task-specific surgical constraints
    double score = metrics.joint_space_distance;

    if (score < best_score)
    {
      best_score = score;
      best_candidate_name = candidate.name;
      best_metrics = metrics;
      best_plan = plan;
      found_valid_plan = true;
    }
  }

  // ------------------------------------------------------------
  // 5. Execute best plan
  // ------------------------------------------------------------

  RCLCPP_INFO(logger, "========================================");

  if (!found_valid_plan)
  {
    RCLCPP_ERROR(logger, "No valid collision-aware candidate plan was found.");

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
  RCLCPP_INFO(logger, "Best duration: %.6f sec", best_metrics.total_duration);
  RCLCPP_INFO(logger, "Best point count: %zu", best_metrics.point_count);

  RCLCPP_INFO(logger, "Executing best collision-aware candidate. Watch RViz.");

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

  RCLCPP_INFO(logger, "Leaving the collision object visible for 30 seconds.");
  rclcpp::sleep_for(std::chrono::seconds(30));

  executor.cancel();

  if (spinner.joinable())
  {
    spinner.join();
  }

  rclcpp::shutdown();
  return 0;
}
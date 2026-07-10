#include <ament_index_cpp/get_package_share_directory.hpp>

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_array.hpp>

#include <rclcpp/rclcpp.hpp>

#include <tf2/LinearMath/Quaternion.hpp>
#include <tf2/LinearMath/Transform.hpp>
#include <tf2/LinearMath/Vector3.hpp>

#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <chrono>
#include <cmath>
#include <cstddef>
#include <fstream>
#include <functional>
#include <map>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

using namespace std::chrono_literals;

namespace
{

constexpr double kMillimetersToMeters = 0.001;
constexpr double kPi = 3.14159265358979323846;

struct CsvToolpathPoint
{
  int line_id;
  int point_index;

  tf2::Vector3 tcp_object_m;
  tf2::Quaternion tool_orientation_object;
};

std::vector<std::string> splitCsvLine(const std::string& line)
{
  std::vector<std::string> values;
  std::stringstream stream(line);
  std::string value;

  while (std::getline(stream, value, ','))
  {
    values.push_back(value);
  }

  return values;
}

double degreesToRadians(double degrees)
{
  return degrees * kPi / 180.0;
}

geometry_msgs::msg::Pose transformToPose(
  const tf2::Transform& transform)
{
  geometry_msgs::msg::Pose pose;

  const tf2::Vector3& translation = transform.getOrigin();
  tf2::Quaternion rotation = transform.getRotation();
  rotation.normalize();

  pose.position.x = translation.x();
  pose.position.y = translation.y();
  pose.position.z = translation.z();

  pose.orientation.x = rotation.x();
  pose.orientation.y = rotation.y();
  pose.orientation.z = rotation.z();
  pose.orientation.w = rotation.w();

  return pose;
}

geometry_msgs::msg::Point vectorToPoint(
  const tf2::Vector3& vector)
{
  geometry_msgs::msg::Point point;

  point.x = vector.x();
  point.y = vector.y();
  point.z = vector.z();

  return point;
}

}  // namespace


class SurfaceToolpathVisualizer : public rclcpp::Node
{
public:
  SurfaceToolpathVisualizer()
  : Node("surface_toolpath_visualizer")
  {
    const std::string package_share =
      ament_index_cpp::get_package_share_directory(
        "ur5e_moveit_cpp");

    const std::string default_csv_path =
      package_share + "/data/toolpaths/front_raster.csv";

    declare_parameter<std::string>(
      "csv_path",
      default_csv_path);

    declare_parameter<std::string>(
      "frame_id",
      "world");

    declare_parameter<std::string>(
      "mesh_resource",
      "package://ur5e_moveit_cpp/meshes/"
      "apple_surface_z_up.stl");

    // Mock placement of the apple in the MoveIt world frame.
    declare_parameter<double>("object_x", 0.60);
    declare_parameter<double>("object_y", 0.00);
    declare_parameter<double>("object_z", 0.10);

    declare_parameter<double>("object_roll_deg", 0.0);
    declare_parameter<double>("object_pitch_deg", 0.0);
    declare_parameter<double>("object_yaw_deg", 90.0);

    declare_parameter<double>("mesh_scale", 0.001);
    declare_parameter<double>("path_width", 0.002);
    declare_parameter<double>("axis_length", 0.025);
    declare_parameter<int>("orientation_stride", 10);

    csv_path_ = get_parameter("csv_path").as_string();
    frame_id_ = get_parameter("frame_id").as_string();
    mesh_resource_ =
      get_parameter("mesh_resource").as_string();

    mesh_scale_ = get_parameter("mesh_scale").as_double();
    path_width_ = get_parameter("path_width").as_double();
    axis_length_ = get_parameter("axis_length").as_double();
    orientation_stride_ =
      get_parameter("orientation_stride").as_int();

    if (mesh_scale_ <= 0.0)
    {
      throw std::runtime_error(
        "mesh_scale must be positive");
    }

    if (path_width_ <= 0.0)
    {
      throw std::runtime_error(
        "path_width must be positive");
    }

    if (axis_length_ <= 0.0)
    {
      throw std::runtime_error(
        "axis_length must be positive");
    }

    if (orientation_stride_ < 1)
    {
      throw std::runtime_error(
        "orientation_stride must be at least 1");
    }

    createObjectTransform();
    loadToolpathCsv();
    transformToolpathToWorld();
    createVisualizationMessages();

    rclcpp::QoS durable_qos(1);
    durable_qos.reliable();
    durable_qos.transient_local();

    marker_publisher_ =
      create_publisher<
        visualization_msgs::msg::MarkerArray>(
          "/surface_toolpath/markers",
          durable_qos);

    pose_publisher_ =
      create_publisher<geometry_msgs::msg::PoseArray>(
        "/surface_toolpath/poses",
        durable_qos);

    timer_ = create_wall_timer(
      1s,
      std::bind(
        &SurfaceToolpathVisualizer::publishVisualization,
        this));

    publishVisualization();

    RCLCPP_INFO(
      get_logger(),
      "Loaded %zu poses across %zu raster lines",
      world_poses_.size(),
      poses_by_line_.size());

    RCLCPP_INFO(
      get_logger(),
      "Publishing markers on /surface_toolpath/markers");

    RCLCPP_INFO(
      get_logger(),
      "Publishing poses on /surface_toolpath/poses");

    RCLCPP_INFO(
      get_logger(),
      "This node only visualizes; it does not plan or execute motion");
  }

private:
  void createObjectTransform()
  {
    const double x =
      get_parameter("object_x").as_double();
    const double y =
      get_parameter("object_y").as_double();
    const double z =
      get_parameter("object_z").as_double();

    const double roll = degreesToRadians(
      get_parameter("object_roll_deg").as_double());

    const double pitch = degreesToRadians(
      get_parameter("object_pitch_deg").as_double());

    const double yaw = degreesToRadians(
      get_parameter("object_yaw_deg").as_double());

    tf2::Quaternion orientation;
    orientation.setRPY(roll, pitch, yaw);
    orientation.normalize();

    object_transform_ = tf2::Transform(
      orientation,
      tf2::Vector3(x, y, z));

    object_pose_ = transformToPose(
      object_transform_);

    RCLCPP_INFO(
      get_logger(),
      "Object pose in %s: xyz=[%.3f, %.3f, %.3f], "
      "rpy_deg=[%.1f, %.1f, %.1f]",
      frame_id_.c_str(),
      x,
      y,
      z,
      get_parameter("object_roll_deg").as_double(),
      get_parameter("object_pitch_deg").as_double(),
      get_parameter("object_yaw_deg").as_double());
  }

  void loadToolpathCsv()
  {
    std::ifstream input(csv_path_);

    if (!input.is_open())
    {
      throw std::runtime_error(
        "Could not open toolpath CSV: " + csv_path_);
    }

    std::string header_line;

    if (!std::getline(input, header_line))
    {
      throw std::runtime_error(
        "Toolpath CSV is empty: " + csv_path_);
    }

    if (
      !header_line.empty() &&
      header_line.back() == '\r')
    {
      header_line.pop_back();
    }

    const std::vector<std::string> headers =
      splitCsvLine(header_line);

    std::unordered_map<std::string, std::size_t>
      header_indices;

    for (std::size_t index = 0;
         index < headers.size();
         ++index)
    {
      header_indices[headers[index]] = index;
    }

    const std::vector<std::string> required_headers = {
      "line_id",
      "point_index",
      "tcp_x_mm",
      "tcp_y_mm",
      "tcp_z_mm",
      "qx",
      "qy",
      "qz",
      "qw"
    };

    for (const std::string& header : required_headers)
    {
      if (header_indices.count(header) == 0)
      {
        throw std::runtime_error(
          "CSV is missing required column: " + header);
      }
    }

    std::string line;
    std::size_t csv_row = 1;

    while (std::getline(input, line))
    {
      ++csv_row;

      if (
        !line.empty() &&
        line.back() == '\r')
      {
        line.pop_back();
      }

      if (line.empty())
      {
        continue;
      }

      const std::vector<std::string> values =
        splitCsvLine(line);

      if (values.size() != headers.size())
      {
        throw std::runtime_error(
          "CSV row " + std::to_string(csv_row) +
          " has an unexpected number of fields");
      }

      auto value =
        [&values, &header_indices](
          const std::string& name) -> const std::string&
        {
          return values.at(header_indices.at(name));
        };

      CsvToolpathPoint point;

      point.line_id = std::stoi(value("line_id"));
      point.point_index =
        std::stoi(value("point_index"));

      point.tcp_object_m = tf2::Vector3(
        std::stod(value("tcp_x_mm")) *
          kMillimetersToMeters,
        std::stod(value("tcp_y_mm")) *
          kMillimetersToMeters,
        std::stod(value("tcp_z_mm")) *
          kMillimetersToMeters);

      point.tool_orientation_object = tf2::Quaternion(
        std::stod(value("qx")),
        std::stod(value("qy")),
        std::stod(value("qz")),
        std::stod(value("qw")));

      point.tool_orientation_object.normalize();

      csv_points_.push_back(point);
    }

    if (csv_points_.empty())
    {
      throw std::runtime_error(
        "Toolpath CSV contains no data points");
    }
  }

  void transformToolpathToWorld()
  {
    world_poses_.clear();
    poses_by_line_.clear();
    world_tool_transforms_.clear();

    for (const CsvToolpathPoint& point : csv_points_)
    {
      const tf2::Transform object_tool_transform(
        point.tool_orientation_object,
        point.tcp_object_m);

      const tf2::Transform world_tool_transform =
        object_transform_ * object_tool_transform;

      const geometry_msgs::msg::Pose world_pose =
        transformToPose(world_tool_transform);

      world_poses_.push_back(world_pose);
      poses_by_line_[point.line_id].push_back(
        world_pose);
      world_tool_transforms_.push_back(
        world_tool_transform);
    }
  }

  void createVisualizationMessages()
  {
    pose_array_.header.frame_id = frame_id_;
    pose_array_.poses = world_poses_;

    marker_array_.markers.clear();

    createMeshMarker();
    createPathMarkers();
    createOrientationMarkers();
  }

  void createMeshMarker()
  {
    visualization_msgs::msg::Marker mesh_marker;

    mesh_marker.header.frame_id = frame_id_;
    mesh_marker.ns = "surface_object";
    mesh_marker.id = 0;
    mesh_marker.type =
      visualization_msgs::msg::Marker::MESH_RESOURCE;
    mesh_marker.action =
      visualization_msgs::msg::Marker::ADD;

    mesh_marker.pose = object_pose_;

    mesh_marker.scale.x = mesh_scale_;
    mesh_marker.scale.y = mesh_scale_;
    mesh_marker.scale.z = mesh_scale_;

    mesh_marker.color.r = 0.70F;
    mesh_marker.color.g = 0.72F;
    mesh_marker.color.b = 0.75F;
    mesh_marker.color.a = 0.65F;

    mesh_marker.mesh_resource = mesh_resource_;
    mesh_marker.mesh_use_embedded_materials = false;

    marker_array_.markers.push_back(mesh_marker);
  }

  void createPathMarkers()
  {
    int marker_id = 100;

    for (const auto& entry : poses_by_line_)
    {
      const int line_id = entry.first;
      const std::vector<geometry_msgs::msg::Pose>& poses =
        entry.second;

      visualization_msgs::msg::Marker path_marker;

      path_marker.header.frame_id = frame_id_;
      path_marker.ns = "surface_path";
      path_marker.id = marker_id++;
      path_marker.type =
        visualization_msgs::msg::Marker::LINE_STRIP;
      path_marker.action =
        visualization_msgs::msg::Marker::ADD;

      path_marker.pose.orientation.w = 1.0;
      path_marker.scale.x = path_width_;

      const bool even_line = line_id % 2 == 0;

      path_marker.color.r =
        even_line ? 1.0F : 1.0F;
      path_marker.color.g =
        even_line ? 0.10F : 0.55F;
      path_marker.color.b =
        even_line ? 0.05F : 0.05F;
      path_marker.color.a = 1.0F;

      for (const geometry_msgs::msg::Pose& pose : poses)
      {
        path_marker.points.push_back(
          vectorToPoint(
            tf2::Vector3(
              pose.position.x,
              pose.position.y,
              pose.position.z)));
      }

      marker_array_.markers.push_back(path_marker);
    }
  }

  void createOrientationMarkers()
  {
    int marker_id = 1000;

    for (std::size_t index = 0;
         index < world_tool_transforms_.size();
         index += static_cast<std::size_t>(
           orientation_stride_))
    {
      const tf2::Transform& tool_transform =
        world_tool_transforms_[index];

      const tf2::Vector3 start =
        tool_transform.getOrigin();

      // Tool +Z points from the nozzle toward the surface.
      const tf2::Vector3 tool_z_world =
        tool_transform.getBasis() *
        tf2::Vector3(0.0, 0.0, 1.0);

      const tf2::Vector3 end =
        start + axis_length_ * tool_z_world;

      visualization_msgs::msg::Marker arrow;

      arrow.header.frame_id = frame_id_;
      arrow.ns = "tool_z_axes";
      arrow.id = marker_id++;
      arrow.type =
        visualization_msgs::msg::Marker::ARROW;
      arrow.action =
        visualization_msgs::msg::Marker::ADD;

      arrow.pose.orientation.w = 1.0;

      arrow.points.push_back(
        vectorToPoint(start));
      arrow.points.push_back(
        vectorToPoint(end));

      arrow.scale.x = 0.0015;
      arrow.scale.y = 0.0035;
      arrow.scale.z = 0.0050;

      arrow.color.r = 0.10F;
      arrow.color.g = 0.35F;
      arrow.color.b = 1.00F;
      arrow.color.a = 1.00F;

      marker_array_.markers.push_back(arrow);
    }
  }

  void publishVisualization()
  {
    const rclcpp::Time timestamp = now();

    pose_array_.header.stamp = timestamp;

    for (visualization_msgs::msg::Marker& marker :
         marker_array_.markers)
    {
      marker.header.stamp = timestamp;
    }

    marker_publisher_->publish(marker_array_);
    pose_publisher_->publish(pose_array_);
  }

  std::string csv_path_;
  std::string frame_id_;
  std::string mesh_resource_;

  double mesh_scale_;
  double path_width_;
  double axis_length_;
  int orientation_stride_;

  tf2::Transform object_transform_;
  geometry_msgs::msg::Pose object_pose_;

  std::vector<CsvToolpathPoint> csv_points_;
  std::vector<geometry_msgs::msg::Pose> world_poses_;
  std::vector<tf2::Transform> world_tool_transforms_;

  std::map<
    int,
    std::vector<geometry_msgs::msg::Pose>>
    poses_by_line_;

  geometry_msgs::msg::PoseArray pose_array_;
  visualization_msgs::msg::MarkerArray marker_array_;

  rclcpp::Publisher<
    visualization_msgs::msg::MarkerArray>::SharedPtr
    marker_publisher_;

  rclcpp::Publisher<
    geometry_msgs::msg::PoseArray>::SharedPtr
    pose_publisher_;

  rclcpp::TimerBase::SharedPtr timer_;
};


int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);

  try
  {
    auto node =
      std::make_shared<SurfaceToolpathVisualizer>();

    rclcpp::spin(node);
  }
  catch (const std::exception& error)
  {
    RCLCPP_FATAL(
      rclcpp::get_logger(
        "surface_toolpath_visualizer"),
      "%s",
      error.what());

    rclcpp::shutdown();
    return 1;
  }

  rclcpp::shutdown();
  return 0;
}
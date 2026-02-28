#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "nav_msgs/msg/occupancy_grid.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "std_msgs/msg/header.hpp"

class ElevationMappingNode : public rclcpp::Node {
public:
  ElevationMappingNode() : Node("elevation_mapping_ros2") {
    pointcloud_topic_ = this->declare_parameter<std::string>("pointcloud_topic", "/rslidar_points");
    map_frame_ = this->declare_parameter<std::string>("map_frame", "map");
    resolution_ = this->declare_parameter<double>("resolution", 0.1);
    map_length_x_ = this->declare_parameter<double>("map_length_x", 40.0);
    map_length_y_ = this->declare_parameter<double>("map_length_y", 40.0);
    min_height_ = this->declare_parameter<double>("min_height", -3.0);
    max_height_ = this->declare_parameter<double>("max_height", 5.0);
    aggregation_method_ = this->declare_parameter<std::string>("aggregation_method", "max");
    min_points_per_cell_ = this->declare_parameter<int>("min_points_per_cell", 2);
    smoothing_factor_ = this->declare_parameter<double>("smoothing_factor", 0.7);
    auto_contrast_ = this->declare_parameter<bool>("auto_contrast", true);
    visualization_min_height_ = this->declare_parameter<double>("visualization_min_height", min_height_);
    visualization_max_height_ = this->declare_parameter<double>("visualization_max_height", max_height_);
    map_frame_from_cloud_ = this->declare_parameter<bool>("map_frame_from_cloud", false);

    smoothing_factor_ = std::clamp(smoothing_factor_, 0.0, 1.0);

    width_ = static_cast<unsigned int>(std::max(1.0, std::round(map_length_x_ / resolution_)));
    height_ = static_cast<unsigned int>(std::max(1.0, std::round(map_length_y_ / resolution_)));

    elevations_.assign(width_ * height_, std::numeric_limits<float>::quiet_NaN());

    occupancy_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("elevation_map", 1);
    cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("elevation_map_points", 1);

    auto qos = rclcpp::SensorDataQoS();
    points_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      pointcloud_topic_, qos,
      std::bind(&ElevationMappingNode::pointCloudCallback, this, std::placeholders::_1));

    if (aggregation_method_ != "mean" && aggregation_method_ != "max") {
      RCLCPP_WARN(this->get_logger(), "Unknown aggregation_method='%s', fallback to 'max'.", aggregation_method_.c_str());
      aggregation_method_ = "max";
    }

    RCLCPP_INFO(this->get_logger(), "Subscribed to %s, map size %ux%u @ %.3fm, aggregation=%s, output_frame=%s",
      pointcloud_topic_.c_str(), width_, height_, resolution_, aggregation_method_.c_str(),
      (map_frame_from_cloud_ ? "<cloud_frame>" : map_frame_.c_str()));
  }

private:
  inline bool worldToGrid(const float x, const float y, unsigned int & gx, unsigned int & gy) const {
    const double origin_x = -map_length_x_ / 2.0;
    const double origin_y = -map_length_y_ / 2.0;

    const double mx = (static_cast<double>(x) - origin_x) / resolution_;
    const double my = (static_cast<double>(y) - origin_y) / resolution_;

    if (mx < 0.0 || my < 0.0 || mx >= static_cast<double>(width_) || my >= static_cast<double>(height_)) {
      return false;
    }

    gx = static_cast<unsigned int>(mx);
    gy = static_cast<unsigned int>(my);
    return true;
  }

  void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    std::vector<float> cell_sum(width_ * height_, 0.0f);
    std::vector<float> cell_max(width_ * height_, -std::numeric_limits<float>::infinity());
    std::vector<int> cell_count(width_ * height_, 0);

    sensor_msgs::PointCloud2ConstIterator<float> iter_x(*msg, "x");
    sensor_msgs::PointCloud2ConstIterator<float> iter_y(*msg, "y");
    sensor_msgs::PointCloud2ConstIterator<float> iter_z(*msg, "z");

    for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
      const float x = *iter_x;
      const float y = *iter_y;
      const float z = *iter_z;

      if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
        continue;
      }
      if (z < min_height_ || z > max_height_) {
        continue;
      }

      unsigned int gx = 0;
      unsigned int gy = 0;
      if (!worldToGrid(x, y, gx, gy)) {
        continue;
      }

      const size_t index = gy * width_ + gx;
      cell_sum[index] += z;
      cell_max[index] = std::max(cell_max[index], z);
      cell_count[index] += 1;
    }

    for (size_t i = 0; i < elevations_.size(); ++i) {
      if (cell_count[i] < min_points_per_cell_) {
        continue;
      }
      const float measured = (aggregation_method_ == "mean")
                               ? (cell_sum[i] / static_cast<float>(cell_count[i]))
                               : cell_max[i];
      if (std::isnan(elevations_[i])) {
        elevations_[i] = measured;
      } else {
        elevations_[i] = static_cast<float>(smoothing_factor_) * elevations_[i] +
                         static_cast<float>(1.0 - smoothing_factor_) * measured;
      }
    }

    publishOccupancyGrid(msg->header);
    publishElevationCloud(msg->header.stamp, msg->header.frame_id);
  }

  void publishOccupancyGrid(const std_msgs::msg::Header & source_header) {
    nav_msgs::msg::OccupancyGrid map;
    map.header.stamp = source_header.stamp;
    map.header.frame_id = map_frame_from_cloud_ ? source_header.frame_id : map_frame_;

    map.info.resolution = static_cast<float>(resolution_);
    map.info.width = width_;
    map.info.height = height_;
    map.info.origin.position.x = -map_length_x_ / 2.0;
    map.info.origin.position.y = -map_length_y_ / 2.0;
    map.info.origin.position.z = 0.0;
    map.info.origin.orientation.w = 1.0;

    map.data.resize(width_ * height_);

    double vis_min = visualization_min_height_;
    double vis_max = visualization_max_height_;
    if (auto_contrast_) {
      vis_min = std::numeric_limits<double>::infinity();
      vis_max = -std::numeric_limits<double>::infinity();
      for (const auto & h : elevations_) {
        if (!std::isfinite(h)) {
          continue;
        }
        vis_min = std::min(vis_min, static_cast<double>(h));
        vis_max = std::max(vis_max, static_cast<double>(h));
      }
      if (!std::isfinite(vis_min) || !std::isfinite(vis_max)) {
        vis_min = min_height_;
        vis_max = max_height_;
      }
    }

    const double denom = std::max(1e-6, vis_max - vis_min);
    for (size_t i = 0; i < elevations_.size(); ++i) {
      const float h = elevations_[i];
      if (std::isnan(h)) {
        map.data[i] = -1;
        continue;
      }
      const double norm = std::clamp((static_cast<double>(h) - vis_min) / denom, 0.0, 1.0);
      map.data[i] = static_cast<int8_t>(std::round(norm * 100.0));
    }

    occupancy_pub_->publish(map);
  }

  void publishElevationCloud(const rclcpp::Time & stamp, const std::string & source_frame) {
    sensor_msgs::msg::PointCloud2 cloud;
    cloud.header.stamp = stamp;
    cloud.header.frame_id = map_frame_from_cloud_ ? source_frame : map_frame_;
    cloud.height = 1;
    cloud.is_bigendian = false;
    cloud.is_dense = false;

    size_t valid = 0;
    for (const auto & e : elevations_) {
      if (std::isfinite(e)) {
        ++valid;
      }
    }

    sensor_msgs::PointCloud2Modifier modifier(cloud);
    modifier.setPointCloud2FieldsByString(1, "xyz");
    modifier.resize(valid);

    sensor_msgs::PointCloud2Iterator<float> iter_x(cloud, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(cloud, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(cloud, "z");

    const double origin_x = -map_length_x_ / 2.0;
    const double origin_y = -map_length_y_ / 2.0;

    for (unsigned int gy = 0; gy < height_; ++gy) {
      for (unsigned int gx = 0; gx < width_; ++gx) {
        const size_t i = gy * width_ + gx;
        const float h = elevations_[i];
        if (!std::isfinite(h)) {
          continue;
        }

        *iter_x = static_cast<float>(origin_x + (static_cast<double>(gx) + 0.5) * resolution_);
        *iter_y = static_cast<float>(origin_y + (static_cast<double>(gy) + 0.5) * resolution_);
        *iter_z = h;
        ++iter_x;
        ++iter_y;
        ++iter_z;
      }
    }

    cloud_pub_->publish(cloud);
  }

  std::string pointcloud_topic_;
  std::string map_frame_;
  std::string aggregation_method_;
  double resolution_;
  double map_length_x_;
  double map_length_y_;
  double min_height_;
  double max_height_;
  bool auto_contrast_;
  double visualization_min_height_;
  double visualization_max_height_;
  bool map_frame_from_cloud_;
  int min_points_per_cell_;
  double smoothing_factor_;

  unsigned int width_;
  unsigned int height_;

  std::vector<float> elevations_;

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr points_sub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr occupancy_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ElevationMappingNode>());
  rclcpp::shutdown();
  return 0;
}

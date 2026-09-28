#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <stdexcept>
#include <string>
#include <unordered_map>

#include <builtin_interfaces/msg/time.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

namespace
{
struct VoxelKey
{
  std::int64_t x;
  std::int64_t y;
  std::int64_t z;

  bool operator==(const VoxelKey & other) const
  {
    return x == other.x && y == other.y && z == other.z;
  }
};

struct VoxelKeyHash
{
  std::size_t operator()(const VoxelKey & key) const
  {
    std::size_t seed = std::hash<std::int64_t>{}(key.x);
    seed ^= std::hash<std::int64_t>{}(key.y) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
    seed ^= std::hash<std::int64_t>{}(key.z) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
    return seed;
  }
};
}  // namespace

class HistoryCloudAccumulator : public rclcpp::Node
{
public:
  HistoryCloudAccumulator()
  : Node("history_cloud_accumulator")
  {
    input_topic_ = declare_parameter<std::string>("input_topic", "/lio/cloud_world");
    output_topic_ = declare_parameter<std::string>("output_topic", "/lio/history_map");
    frame_id_ = declare_parameter<std::string>("frame_id", "world");
    voxel_size_ = declare_parameter<double>("voxel_size", 0.15);
    publish_period_ = declare_parameter<double>("publish_period", 2.0);
    max_voxels_ = declare_parameter<int>("max_voxels", 1000000);

    if (voxel_size_ <= 0.0) {
      throw std::runtime_error("voxel_size must be positive");
    }
    if (publish_period_ <= 0.0) {
      throw std::runtime_error("publish_period must be positive");
    }

    voxels_.reserve(static_cast<std::size_t>(std::min(max_voxels_, 200000)));

    cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      input_topic_, rclcpp::SensorDataQoS(),
      std::bind(&HistoryCloudAccumulator::cloudCallback, this, std::placeholders::_1));
    cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      output_topic_, rclcpp::QoS(1).reliable().transient_local());
    publish_timer_ = create_wall_timer(
      std::chrono::duration<double>(publish_period_),
      std::bind(&HistoryCloudAccumulator::publishMap, this));

    RCLCPP_INFO(
      get_logger(),
      "Accumulating %s into %s (voxel=%.3f m, period=%.1f s, max=%d)",
      input_topic_.c_str(), output_topic_.c_str(), voxel_size_, publish_period_, max_voxels_);
  }

private:
  void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    pcl::PointCloud<pcl::PointXYZI> cloud;
    pcl::fromROSMsg(*msg, cloud);

    for (const auto & point : cloud.points) {
      if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z)) {
        continue;
      }

      const VoxelKey key{
        static_cast<std::int64_t>(std::floor(point.x / voxel_size_)),
        static_cast<std::int64_t>(std::floor(point.y / voxel_size_)),
        static_cast<std::int64_t>(std::floor(point.z / voxel_size_))};

      auto existing = voxels_.find(key);
      if (existing != voxels_.end()) {
        existing->second = point;
        continue;
      }

      if (max_voxels_ > 0 && voxels_.size() >= static_cast<std::size_t>(max_voxels_)) {
        if (!limit_warned_) {
          RCLCPP_WARN(
            get_logger(), "History map reached max_voxels=%d; new voxels are being ignored",
            max_voxels_);
          limit_warned_ = true;
        }
        continue;
      }
      voxels_.emplace(key, point);
    }

    last_stamp_ = msg->header.stamp;
  }

  void publishMap()
  {
    if (voxels_.empty() || cloud_pub_->get_subscription_count() == 0) {
      return;
    }

    pcl::PointCloud<pcl::PointXYZI> map;
    map.points.reserve(voxels_.size());
    for (const auto & entry : voxels_) {
      map.points.push_back(entry.second);
    }
    map.width = static_cast<std::uint32_t>(map.points.size());
    map.height = 1;
    map.is_dense = false;

    sensor_msgs::msg::PointCloud2 output;
    pcl::toROSMsg(map, output);
    output.header.frame_id = frame_id_;
    output.header.stamp = last_stamp_;
    cloud_pub_->publish(output);
  }

  std::string input_topic_;
  std::string output_topic_;
  std::string frame_id_;
  double voxel_size_;
  double publish_period_;
  int max_voxels_;
  bool limit_warned_{false};
  builtin_interfaces::msg::Time last_stamp_;

  std::unordered_map<VoxelKey, pcl::PointXYZI, VoxelKeyHash> voxels_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;
  rclcpp::TimerBase::SharedPtr publish_timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<HistoryCloudAccumulator>());
  rclcpp::shutdown();
  return 0;
}

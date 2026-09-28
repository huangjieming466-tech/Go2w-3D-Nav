#include <algorithm>
#include <chrono>
#include <cstdio>
#include <filesystem>
#include <functional>
#include <mutex>
#include <string>
#include <stdexcept>

#include <Eigen/Dense>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/registration/icp.h>
#include <pcl_conversions/pcl_conversions.h>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/float32.hpp>
#include <tf2_ros/transform_broadcaster.h>

class NativeLocalizer final : public rclcpp::Node {
public:
  NativeLocalizer() : Node("open3d_global_localization") {
    map_file_ = declare_parameter<std::string>("map_file", "");
    cloud_topic_ = declare_parameter<std::string>("cloud_topic", "/cloud_registered_body_1");
    odom_topic_ = declare_parameter<std::string>("odom_topic", "/Odometry_loc");
    map_frame_ = declare_parameter<std::string>("map_frame", "map");
    base_frame_ = declare_parameter<std::string>("base_frame", "base_link");
    voxel_size_ = declare_parameter<double>("voxel_size", 0.15);
    max_corr_ = declare_parameter<double>("registration_distance", 1.5);
    period_ = declare_parameter<double>("update_period", 0.10);

    if (std::filesystem::is_directory(map_file_)) {
      for (const auto &entry : std::filesystem::directory_iterator(map_file_)) {
        if (entry.path().extension() == ".pcd") {
          if (entry.path().filename().string().find("oriented_normals") != std::string::npos) {
            map_file_ = entry.path().string();
            break;
          }
          if (map_file_.back() != '/') {
            map_file_ = entry.path().string();
          }
        }
      }
    }
    if (map_file_.empty() || !std::filesystem::is_regular_file(map_file_)) {
      throw std::runtime_error("map_file does not exist: " + map_file_);
    }
    pcl::PointCloud<pcl::PointXYZ>::Ptr raw(new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(map_file_, *raw) < 0 || raw->empty()) {
      throw std::runtime_error("failed to load map: " + map_file_);
    }
    pcl::VoxelGrid<pcl::PointXYZ> voxel;
    voxel.setInputCloud(raw);
    voxel.setLeafSize(voxel_size_, voxel_size_, voxel_size_);
    voxel.filter(*map_);
    RCLCPP_INFO(get_logger(), "loaded map %s (%zu points)", map_file_.c_str(), map_->size());

    cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      cloud_topic_, rclcpp::SensorDataQoS(),
      std::bind(&NativeLocalizer::cloudCallback, this, std::placeholders::_1));
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      odom_topic_, 20,
      std::bind(&NativeLocalizer::odomCallback, this, std::placeholders::_1));
    pose_pub_ = create_publisher<nav_msgs::msg::Odometry>("/baselink2map_kalman", 20);
    pose_raw_pub_ = create_publisher<nav_msgs::msg::Odometry>("/baselink2map", 20);
    odom_map_pub_ = create_publisher<nav_msgs::msg::Odometry>("/odom2map_kalman", 20);
    localization_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("/localization_3d", 20);
    confidence_pub_ = create_publisher<std_msgs::msg::Float32>("/localization_3d_confidence", 20);
    tf_pub_ = std::make_shared<tf2_ros::TransformBroadcaster>(*this);
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(period_)),
      std::bind(&NativeLocalizer::update, this));
  }

private:
  static Eigen::Matrix4d poseMatrix(const geometry_msgs::msg::Pose &pose) {
    Eigen::Quaterniond q(pose.orientation.w, pose.orientation.x,
                        pose.orientation.y, pose.orientation.z);
    Eigen::Matrix4d result = Eigen::Matrix4d::Identity();
    result.block<3, 3>(0, 0) = q.normalized().toRotationMatrix();
    result(0, 3) = pose.position.x;
    result(1, 3) = pose.position.y;
    result(2, 3) = pose.position.z;
    return result;
  }

  static void setPose(const Eigen::Matrix4d &matrix, geometry_msgs::msg::Pose &pose) {
    Eigen::Quaterniond q(matrix.block<3, 3>(0, 0));
    q.normalize();
    pose.position.x = matrix(0, 3);
    pose.position.y = matrix(1, 3);
    pose.position.z = matrix(2, 3);
    pose.orientation.x = q.x();
    pose.orientation.y = q.y();
    pose.orientation.z = q.z();
    pose.orientation.w = q.w();
  }

  void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    auto cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*msg, *cloud);
    if (cloud->size() < 30) return;
    std::lock_guard<std::mutex> lock(mutex_);
    latest_cloud_.reset(cloud);
  }

  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(mutex_);
    latest_odom_ = msg;
  }

  void update() {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
    nav_msgs::msg::Odometry::SharedPtr odom;
    Eigen::Matrix4d map_odom;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      cloud = latest_cloud_;
      odom = latest_odom_;
      map_odom = this->map_odom_;
    }
    if (!cloud || !odom) return;

    pcl::VoxelGrid<pcl::PointXYZ> voxel;
    pcl::PointCloud<pcl::PointXYZ>::Ptr scan(new pcl::PointCloud<pcl::PointXYZ>);
    voxel.setInputCloud(cloud);
    voxel.setLeafSize(0.10f, 0.10f, 0.10f);
    voxel.filter(*scan);
    if (scan->size() < 30) return;

    const Eigen::Matrix4d odom_body = poseMatrix(odom->pose.pose);
    const Eigen::Matrix4d predicted = map_odom * odom_body;
    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    icp.setInputSource(scan);
    icp.setInputTarget(map_);
    icp.setMaxCorrespondenceDistance(max_corr_);
    icp.setMaximumIterations(20);
    pcl::PointCloud<pcl::PointXYZ> aligned;
    icp.align(aligned, predicted.cast<float>());
    if (!icp.hasConverged()) return;

    const Eigen::Matrix4d result = icp.getFinalTransformation().cast<double>();
    const Eigen::Matrix4d new_map_odom = result * odom_body.inverse();
    {
      std::lock_guard<std::mutex> lock(mutex_);
      map_odom_ = new_map_odom;
    }
    publish(odom->header, new_map_odom * odom_body, new_map_odom, icp.getFitnessScore());
  }

  void publish(const std_msgs::msg::Header &header, const Eigen::Matrix4d &map_body,
               const Eigen::Matrix4d &map_odom, double fitness) {
    nav_msgs::msg::Odometry pose;
    pose.header = header;
    pose.header.frame_id = map_frame_;
    pose.child_frame_id = base_frame_;
    setPose(map_body, pose.pose.pose);
    pose_pub_->publish(pose);
    pose_raw_pub_->publish(pose);

    nav_msgs::msg::Odometry map_odom_msg;
    map_odom_msg.header = pose.header;
    map_odom_msg.child_frame_id = "odom";
    setPose(map_odom, map_odom_msg.pose.pose);
    odom_map_pub_->publish(map_odom_msg);

    geometry_msgs::msg::PoseStamped pose_stamped;
    pose_stamped.header = pose.header;
    pose_stamped.pose = pose.pose.pose;
    localization_pub_->publish(pose_stamped);
    std_msgs::msg::Float32 confidence;
    confidence.data = static_cast<float>(fitness);
    confidence_pub_->publish(confidence);

    geometry_msgs::msg::TransformStamped tf;
    tf.header = pose.header;
    tf.child_frame_id = base_frame_;
    tf.transform.translation.x = pose.pose.pose.position.x;
    tf.transform.translation.y = pose.pose.pose.position.y;
    tf.transform.translation.z = pose.pose.pose.position.z;
    tf.transform.rotation = pose.pose.pose.orientation;
    tf_pub_->sendTransform(tf);
  }

  std::string map_file_, cloud_topic_, odom_topic_, map_frame_, base_frame_;
  double voxel_size_{0.15}, max_corr_{1.5}, period_{0.10};
  pcl::PointCloud<pcl::PointXYZ>::Ptr map_{new pcl::PointCloud<pcl::PointXYZ>};
  pcl::PointCloud<pcl::PointXYZ>::Ptr latest_cloud_;
  nav_msgs::msg::Odometry::SharedPtr latest_odom_;
  Eigen::Matrix4d map_odom_{Eigen::Matrix4d::Identity()};
  std::mutex mutex_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pose_pub_, pose_raw_pub_, odom_map_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr localization_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr confidence_pub_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<NativeLocalizer>());
  } catch (const std::exception &e) {
    fprintf(stderr, "%s\n", e.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}

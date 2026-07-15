#include <openssl/sha.h>

#include <cmath>
#include <fstream>
#include <iomanip>
#include <mutex>
#include <sstream>
#include <string>

#include <diagnostic_msgs/DiagnosticArray.h>
#include <diagnostic_msgs/DiagnosticStatus.h>
#include <diagnostic_msgs/KeyValue.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <pcl/common/point_tests.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include "static_livox_localization/registration.hpp"

namespace {

std::string sha256_file(const std::string& path) {
  std::ifstream input(path, std::ios::binary);
  if (!input) throw std::runtime_error("cannot open map for SHA-256: " + path);
  SHA256_CTX context;
  SHA256_Init(&context);
  char buffer[1 << 20];
  while (input.good()) {
    input.read(buffer, sizeof(buffer));
    if (input.gcount() > 0) SHA256_Update(&context, buffer, input.gcount());
  }
  unsigned char digest[SHA256_DIGEST_LENGTH];
  SHA256_Final(digest, &context);
  std::ostringstream out;
  for (unsigned char byte : digest) out << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(byte);
  return out.str();
}

diagnostic_msgs::KeyValue key_value(const std::string& key, const std::string& value) {
  diagnostic_msgs::KeyValue item; item.key = key; item.value = value; return item;
}

Eigen::Isometry3d pose_to_eigen(const geometry_msgs::Pose& pose) {
  Eigen::Quaterniond q(pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z);
  if (!std::isfinite(q.norm()) || q.norm() < 1e-6) throw std::runtime_error("invalid initial pose quaternion");
  q.normalize();
  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
  transform.linear() = q.toRotationMatrix();
  transform.translation() = Eigen::Vector3d(pose.position.x, pose.position.y, pose.position.z);
  return transform;
}

geometry_msgs::Pose eigen_to_pose(const Eigen::Isometry3d& transform) {
  geometry_msgs::Pose pose;
  pose.position.x = transform.translation().x(); pose.position.y = transform.translation().y(); pose.position.z = transform.translation().z();
  const Eigen::Quaterniond q(transform.rotation());
  pose.orientation.x = q.x(); pose.orientation.y = q.y(); pose.orientation.z = q.z(); pose.orientation.w = q.w();
  return pose;
}

}  // namespace

class StaticIcpLocalizer {
 public:
  StaticIcpLocalizer() : private_nh_("~"), map_(new pcl::PointCloud<pcl::PointXYZI>), accumulator_(new pcl::PointCloud<pcl::PointXYZI>) {
    private_nh_.param<std::string>("map_path", map_path_, "");
    private_nh_.param<std::string>("map_sha256", map_sha256_, "");
    private_nh_.param<std::string>("map_id", map_id_, "livox_raw_20260707");
    private_nh_.param<std::string>("map_frame", map_frame_, "map");
    private_nh_.param<std::string>("base_frame", base_frame_, "base_link");
    private_nh_.param<std::string>("cloud_topic", cloud_topic_, "/cloud_registered_body");
    private_nh_.param<std::string>("seed_topic", seed_topic_, "/fast_lio_icp/initialpose");
    private_nh_.param("window_s", window_s_, 5.0);
    private_nh_.param("voxel_resolution", config_.voxel_resolution, 0.20);
    private_nh_.param("roi_radius", config_.roi_radius, 20.0);
    private_nh_.param("roi_z_half_extent", config_.roi_z_half_extent, 5.0);
    private_nh_.param("max_correspondence", config_.max_correspondence, 1.0);
    private_nh_.param("max_iterations", config_.max_iterations, 64);
    private_nh_.param("min_points", config_.min_points, 500);
    private_nh_.param("max_fitness", config_.max_fitness, 0.20);
    private_nh_.param("max_seed_translation", config_.max_seed_translation, 3.0);
    double max_seed_rotation_deg = 30.0;
    private_nh_.param("max_seed_rotation_deg", max_seed_rotation_deg, 30.0);
    config_.max_seed_rotation_rad = max_seed_rotation_deg * M_PI / 180.0;
    if (map_path_.empty() || map_sha256_.size() != 64) throw std::runtime_error("map_path and 64-char map_sha256 are required");
    const std::string observed = sha256_file(map_path_);
    if (observed != map_sha256_) throw std::runtime_error("map SHA-256 mismatch: " + observed);
    if (pcl::io::loadPCDFile(map_path_, *map_) != 0 || map_->empty()) throw std::runtime_error("failed to load non-empty XYZI PCD map");

    pose_pub_ = nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>("/fast_lio_icp/pose", 10);
    diagnostics_pub_ = nh_.advertise<diagnostic_msgs::DiagnosticArray>("/fast_lio_icp/localization_diagnostics", 10, true);
    initial_pose_sub_ = nh_.subscribe(seed_topic_, 1, &StaticIcpLocalizer::initial_pose_callback, this);
    cloud_sub_ = nh_.subscribe(cloud_topic_, 5, &StaticIcpLocalizer::cloud_callback, this);
    publish_diagnostic(false, "WAITING_FOR_INITIALPOSE", static_livox_localization::RegistrationResult());
    ROS_INFO("Loaded map %s with %zu points; waiting for %s", map_id_.c_str(), map_->size(), seed_topic_.c_str());
  }

 private:
  void initial_pose_callback(const geometry_msgs::PoseWithCovarianceStampedConstPtr& message) {
    try {
      std::lock_guard<std::mutex> lock(mutex_);
      seed_ = pose_to_eigen(message->pose.pose);
      accumulator_->clear();
      window_start_ = ros::Time();
      has_seed_ = true;
      ++reset_count_;
      publish_diagnostic(false, "ACCUMULATING", static_livox_localization::RegistrationResult());
      ROS_INFO("Accepted initial pose seed #%d: %.3f %.3f %.3f", reset_count_, seed_.translation().x(), seed_.translation().y(), seed_.translation().z());
    } catch (const std::exception& error) {
      ROS_ERROR("Rejected /initialpose: %s", error.what());
    }
  }

  void cloud_callback(const sensor_msgs::PointCloud2ConstPtr& message) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!has_seed_) return;
    pcl::PointCloud<pcl::PointXYZI> cloud;
    pcl::fromROSMsg(*message, cloud);
    for (const auto& point : cloud.points) if (pcl::isFinite(point)) accumulator_->push_back(point);
    const ros::Time stamp = message->header.stamp.isZero() ? ros::Time::now() : message->header.stamp;
    if (window_start_.isZero()) window_start_ = stamp;
    if ((stamp - window_start_).toSec() < window_s_) return;
    pcl::PointCloud<pcl::PointXYZI>::Ptr scan(new pcl::PointCloud<pcl::PointXYZI>);
    scan.swap(accumulator_);
    accumulator_.reset(new pcl::PointCloud<pcl::PointXYZI>);
    window_start_ = stamp;
    const auto result = static_livox_localization::register_cloud(scan, map_, seed_, config_);
    if (result.converged) {
      seed_ = result.map_T_base;
      publish_pose(stamp, result);
      publish_diagnostic(true, "RAW_OK", result);
    } else {
      publish_diagnostic(false, "RAW_LOST", result);
    }
  }

  void publish_pose(const ros::Time& stamp, const static_livox_localization::RegistrationResult& result) {
    geometry_msgs::PoseWithCovarianceStamped message;
    message.header.stamp = stamp; message.header.frame_id = map_frame_;
    message.pose.pose = eigen_to_pose(result.map_T_base);
    const double position_variance = std::max(1e-4, result.fitness);
    const double angle_variance = std::max(1e-4, result.fitness * 0.25);
    message.pose.covariance.fill(0.0);
    message.pose.covariance[0] = message.pose.covariance[7] = message.pose.covariance[14] = position_variance;
    message.pose.covariance[21] = message.pose.covariance[28] = message.pose.covariance[35] = angle_variance;
    pose_pub_.publish(message);
  }

  void publish_diagnostic(bool ok, const std::string& state, const static_livox_localization::RegistrationResult& result) {
    diagnostic_msgs::DiagnosticArray array; array.header.stamp = ros::Time::now();
    diagnostic_msgs::DiagnosticStatus status;
    status.name = "fast_lio_icp"; status.hardware_id = map_id_;
    status.level = ok ? diagnostic_msgs::DiagnosticStatus::OK : diagnostic_msgs::DiagnosticStatus::WARN;
    status.message = state;
    status.values.push_back(key_value("raw_state", state));
    status.values.push_back(key_value("raw_score", std::to_string(result.fitness)));
    status.values.push_back(key_value("fitness", std::to_string(result.fitness)));
    status.values.push_back(key_value("inlier_ratio", std::to_string(result.inlier_ratio)));
    status.values.push_back(key_value("reset_count", std::to_string(reset_count_)));
    status.values.push_back(key_value("map_id", map_id_));
    status.values.push_back(key_value("map_sha256", map_sha256_));
    status.values.push_back(key_value("source_points", std::to_string(result.source_points)));
    status.values.push_back(key_value("target_points", std::to_string(result.target_points)));
    array.status.push_back(status); diagnostics_pub_.publish(array);
  }

  ros::NodeHandle nh_, private_nh_;
  ros::Publisher pose_pub_, diagnostics_pub_;
  ros::Subscriber initial_pose_sub_, cloud_sub_;
  pcl::PointCloud<pcl::PointXYZI>::Ptr map_, accumulator_;
  static_livox_localization::RegistrationConfig config_;
  Eigen::Isometry3d seed_ = Eigen::Isometry3d::Identity();
  std::mutex mutex_;
  ros::Time window_start_;
  bool has_seed_ = false;
  int reset_count_ = 0;
  double window_s_ = 5.0;
  std::string map_path_, map_sha256_, map_id_, map_frame_, base_frame_, cloud_topic_, seed_topic_;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "static_icp_localizer");
  try {
    StaticIcpLocalizer node;
    ros::spin();
  } catch (const std::exception& error) {
    ROS_FATAL("static localization startup rejected: %s", error.what());
    return 2;
  }
  return 0;
}



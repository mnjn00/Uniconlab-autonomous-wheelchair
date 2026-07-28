#include <openssl/sha.h>

#include <algorithm>
#include <atomic>
#include <cctype>
#include <cmath>
#include <cstdint>
#include <deque>
#include <fstream>
#include <iomanip>
#include <limits>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include <Eigen/Geometry>
#include <diagnostic_msgs/DiagnosticArray.h>
#include <diagnostic_msgs/DiagnosticStatus.h>
#include <diagnostic_msgs/KeyValue.h>
#include <fast_gicp/gicp/fast_vgicp.hpp>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <hdl_global_localization/QueryGlobalLocalization.h>
#include <hdl_global_localization/SetGlobalLocalizationEngine.h>
#include <hdl_global_localization/SetGlobalMap.h>
#include <nav_msgs/Odometry.h>
#include <pcl/common/point_tests.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include "wheelchair_lidar_localization/localization_core.hpp"

namespace wll = wheelchair_lidar_localization;

namespace {

using Point = pcl::PointXYZ;
using Cloud = pcl::PointCloud<Point>;
constexpr double kPi = 3.14159265358979323846;

std::string sha256File(const std::string& path) {
  std::ifstream stream(path, std::ios::binary);
  if (!stream) {
    throw std::runtime_error("cannot open map for SHA-256: " + path);
  }
  SHA256_CTX context;
  SHA256_Init(&context);
  char block[1 << 20];
  while (stream.good()) {
    stream.read(block, sizeof(block));
    if (stream.gcount() > 0) {
      SHA256_Update(&context, block,
                    static_cast<std::size_t>(stream.gcount()));
    }
  }
  unsigned char digest[SHA256_DIGEST_LENGTH];
  SHA256_Final(digest, &context);
  std::ostringstream output;
  for (const unsigned char byte : digest) {
    output << std::hex << std::setw(2) << std::setfill('0')
           << static_cast<int>(byte);
  }
  return output.str();
}

std::string lowerExtension(const std::string& path) {
  const std::size_t dot = path.find_last_of('.');
  if (dot == std::string::npos) {
    return "";
  }
  std::string extension = path.substr(dot);
  std::transform(extension.begin(), extension.end(), extension.begin(),
                 [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
  return extension;
}

Eigen::Isometry3d poseToEigen(const geometry_msgs::Pose& pose) {
  Eigen::Quaterniond rotation(pose.orientation.w, pose.orientation.x,
                              pose.orientation.y, pose.orientation.z);
  if (!std::isfinite(rotation.norm()) || rotation.norm() < 1e-9) {
    throw std::runtime_error("invalid pose quaternion");
  }
  rotation.normalize();
  Eigen::Isometry3d result = Eigen::Isometry3d::Identity();
  result.translation() =
      Eigen::Vector3d(pose.position.x, pose.position.y, pose.position.z);
  result.linear() = rotation.toRotationMatrix();
  if (!result.matrix().allFinite()) {
    throw std::runtime_error("non-finite pose");
  }
  return result;
}

geometry_msgs::Pose eigenToPose(const Eigen::Isometry3d& transform) {
  geometry_msgs::Pose pose;
  pose.position.x = transform.translation().x();
  pose.position.y = transform.translation().y();
  pose.position.z = transform.translation().z();
  Eigen::Quaterniond rotation(transform.rotation());
  rotation.normalize();
  pose.orientation.x = rotation.x();
  pose.orientation.y = rotation.y();
  pose.orientation.z = rotation.z();
  pose.orientation.w = rotation.w();
  return pose;
}

geometry_msgs::Transform eigenToTransform(const Eigen::Isometry3d& transform) {
  geometry_msgs::Transform message;
  message.translation.x = transform.translation().x();
  message.translation.y = transform.translation().y();
  message.translation.z = transform.translation().z();
  const geometry_msgs::Pose pose = eigenToPose(transform);
  message.rotation = pose.orientation;
  return message;
}

diagnostic_msgs::KeyValue keyValue(const std::string& key,
                                   const std::string& value) {
  diagnostic_msgs::KeyValue result;
  result.key = key;
  result.value = value;
  return result;
}

std::string asString(double value) {
  std::ostringstream output;
  output << std::setprecision(10) << value;
  return output.str();
}

Eigen::Isometry3d interpolate(const Eigen::Isometry3d& previous,
                             const Eigen::Isometry3d& measurement,
                             double alpha) {
  alpha = std::max(0.0, std::min(1.0, alpha));
  Eigen::Isometry3d result = Eigen::Isometry3d::Identity();
  result.translation() =
      (1.0 - alpha) * previous.translation() +
      alpha * measurement.translation();
  Eigen::Quaterniond a(previous.rotation());
  Eigen::Quaterniond b(measurement.rotation());
  if (a.dot(b) < 0.0) {
    b.coeffs() *= -1.0;
  }
  result.linear() = a.slerp(alpha, b).normalized().toRotationMatrix();
  return result;
}

class ProcessingFlag {
 public:
  explicit ProcessingFlag(std::atomic<bool>& flag) : flag_(flag) {}
  ~ProcessingFlag() { flag_.store(false); }

 private:
  std::atomic<bool>& flag_;
};

}  // namespace

class WheelchairMapLocalizer {
 public:
  WheelchairMapLocalizer()
      : private_nh_("~"),
        tf_listener_(tf_buffer_),
        map_(new Cloud),
        consensus_(3, 0.25, 4.0 * kPi / 180.0) {
    loadParameters();
    consensus_ = wll::PoseConsensus(
        static_cast<std::size_t>(global_consensus_count_),
        global_consensus_translation_m_,
        global_consensus_rotation_deg_ * kPi / 180.0);
    loadMap();

    pose_pub_ = nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>(
        "/fast_lio_icp/pose", 10);
    diagnostics_pub_ = nh_.advertise<diagnostic_msgs::DiagnosticArray>(
        "/fast_lio_icp/localization_diagnostics", 10, true);
    canonical_odom_pub_ = nh_.advertise<nav_msgs::Odometry>("/odom", 50);
    aligned_pub_ =
        nh_.advertise<sensor_msgs::PointCloud2>("/fast_lio_icp/aligned", 2);

    odom_sub_ = nh_.subscribe(fastlio_odom_topic_, 200,
                              &WheelchairMapLocalizer::odomCallback, this);
    cloud_sub_ = nh_.subscribe(fastlio_body_cloud_topic_, 20,
                               &WheelchairMapLocalizer::cloudCallback, this);
    initial_pose_sub_ = nh_.subscribe(
        "/initialpose", 1, &WheelchairMapLocalizer::initialPoseCallback, this);

    set_engine_client_ =
        nh_.serviceClient<hdl_global_localization::SetGlobalLocalizationEngine>(
            "/hdl_global_localization/set_engine", true);
    set_map_client_ =
        nh_.serviceClient<hdl_global_localization::SetGlobalMap>(
            "/hdl_global_localization/set_global_map", true);
    global_query_client_ =
        nh_.serviceClient<hdl_global_localization::QueryGlobalLocalization>(
            "/hdl_global_localization/query", true);

    timer_ = nh_.createTimer(ros::Duration(0.05),
                             &WheelchairMapLocalizer::timerCallback, this);
    publishDiagnostic("WAITING_FOR_FAST_LIO", wll::RegistrationMetrics());
    ROS_INFO_STREAM("Loaded immutable map " << map_id_ << " with "
                                            << map_->size() << " points");
  }

 private:
  struct OdomSample {
    ros::Time stamp;
    Eigen::Isometry3d odom_T_body = Eigen::Isometry3d::Identity();
  };

  struct CloudFrame {
    ros::Time stamp;
    Eigen::Isometry3d odom_T_body = Eigen::Isometry3d::Identity();
    Cloud::Ptr cloud;
  };

  struct SubmapSnapshot {
    ros::Time stamp;
    Eigen::Isometry3d odom_T_body = Eigen::Isometry3d::Identity();
    Cloud::Ptr cloud;
    std::uint64_t state_epoch = 0;
    std::uint32_t reset_count = 0;
  };

  struct RegistrationResult {
    Eigen::Isometry3d map_T_body = Eigen::Isometry3d::Identity();
    wll::RegistrationMetrics metrics;
    Cloud::Ptr aligned;
  };

  void loadParameters() {
    private_nh_.param<std::string>("map_path", map_path_, "");
    private_nh_.param<std::string>("map_sha256", map_sha256_, "");
    private_nh_.param<std::string>("map_id", map_id_, "");
    private_nh_.param<std::string>("map_frame", map_frame_, "map");
    private_nh_.param<std::string>("odom_frame", odom_frame_, "odom");
    private_nh_.param<std::string>("body_frame", body_frame_, "body");
    private_nh_.param<std::string>("base_frame", base_frame_,
                                   "base_footprint");
    private_nh_.param<std::string>("fastlio_odom_topic",
                                   fastlio_odom_topic_, "/Odometry");
    private_nh_.param<std::string>("fastlio_body_cloud_topic",
                                   fastlio_body_cloud_topic_,
                                   "/cloud_registered_body");
    private_nh_.param<std::string>("global_engine", global_engine_,
                                   "FPFH_TEASER");

    private_nh_.param("automatic_initialization",
                      automatic_initialization_, true);
    private_nh_.param("rolling_window_s", rolling_window_s_, 1.5);
    private_nh_.param("max_cloud_frames", max_cloud_frames_, 15);
    private_nh_.param("max_cloud_odom_skew_s", max_cloud_odom_skew_s_, 0.08);
    private_nh_.param("submap_voxel_m", submap_voxel_m_, 0.20);
    private_nh_.param("min_submap_points", min_submap_points_, 700);
    private_nh_.param("stationary_linear_mps", stationary_linear_mps_, 0.03);
    private_nh_.param("stationary_angular_radps",
                      stationary_angular_radps_, 0.05);
    private_nh_.param("stationary_hold_s", stationary_hold_s_, 2.0);

    private_nh_.param("tracking_period_s", tracking_period_s_, 0.50);
    private_nh_.param("global_period_s", global_period_s_, 1.0);
    private_nh_.param("local_map_radius_m", local_map_radius_m_, 18.0);
    private_nh_.param("local_map_z_half_extent_m",
                      local_map_z_half_extent_m_, 4.0);
    private_nh_.param("vgicp_voxel_m", vgicp_voxel_m_, 0.30);
    private_nh_.param("max_correspondence_m", max_correspondence_m_, 1.0);
    private_nh_.param("max_iterations", max_iterations_, 64);
    private_nh_.param("num_threads", num_threads_, 4);
    private_nh_.param("inlier_distance_m", inlier_distance_m_, 0.30);
    private_nh_.param("correction_alpha", correction_alpha_, 0.25);

    private_nh_.param("tracking/max_fitness",
                      tracking_limits_.max_fitness, 0.20);
    private_nh_.param("tracking/min_inlier_ratio",
                      tracking_limits_.min_inlier_ratio, 0.35);
    private_nh_.param("tracking/max_correction_translation_m",
                      tracking_limits_.max_translation_correction_m, 0.35);
    double tracking_rotation_deg = 6.0;
    private_nh_.param("tracking/max_correction_rotation_deg",
                      tracking_rotation_deg, tracking_rotation_deg);
    tracking_limits_.max_rotation_correction_rad =
        tracking_rotation_deg * kPi / 180.0;
    tracking_limits_.min_source_points =
        static_cast<std::size_t>(min_submap_points_);
    tracking_limits_.min_target_points = 1000;

    private_nh_.param("global/max_fitness",
                      global_limits_.max_fitness, 0.30);
    private_nh_.param("global/min_inlier_ratio",
                      global_limits_.min_inlier_ratio, 0.25);
    private_nh_.param("global/max_refinement_translation_m",
                      global_limits_.max_translation_correction_m, 2.0);
    double global_refinement_rotation_deg = 30.0;
    private_nh_.param("global/max_refinement_rotation_deg",
                      global_refinement_rotation_deg,
                      global_refinement_rotation_deg);
    global_limits_.max_rotation_correction_rad =
        global_refinement_rotation_deg * kPi / 180.0;
    global_limits_.min_source_points =
        static_cast<std::size_t>(min_submap_points_);
    global_limits_.min_target_points = 1000;
    private_nh_.param("global/consensus_count",
                      global_consensus_count_, 3);
    private_nh_.param("global/consensus_translation_m",
                      global_consensus_translation_m_, 0.25);
    private_nh_.param("global/consensus_rotation_deg",
                      global_consensus_rotation_deg_, 4.0);
    private_nh_.param("global/max_map_to_odom_tilt_deg",
                      max_map_to_odom_tilt_deg_, 5.0);
    private_nh_.param("global/max_candidates", max_global_candidates_, 5);

    private_nh_.param("degraded_after_failures",
                      degraded_after_failures_, 1);
    private_nh_.param("lost_after_failures", lost_after_failures_, 6);
    private_nh_.param("recovery_confirmations",
                      recovery_confirmations_, 3);

    const bool valid_sha256 =
        map_sha256_.size() == 64 &&
        std::all_of(map_sha256_.begin(), map_sha256_.end(),
                    [](unsigned char c) { return std::isxdigit(c) != 0; });
    if (map_path_.empty() || map_id_.empty() || !valid_sha256) {
      throw std::runtime_error(
          "map_path, map_id, and exact hexadecimal map_sha256 are required");
    }
    std::transform(map_sha256_.begin(), map_sha256_.end(),
                   map_sha256_.begin(), [](unsigned char c) {
                     return static_cast<char>(std::tolower(c));
                   });
    if (map_frame_ != "map" || odom_frame_ != "odom" ||
        body_frame_ != "body" || base_frame_ != "base_footprint" ||
        fastlio_odom_topic_ != "/Odometry" ||
        fastlio_body_cloud_topic_ != "/cloud_registered_body") {
      throw std::runtime_error(
          "canonical map/odom/body/base_footprint frames and upstream "
          "FAST-LIO topics are required");
    }
    if (global_engine_ != "FPFH_TEASER") {
      throw std::runtime_error(
          "global_engine must be FPFH_TEASER; no silent weak fallback");
    }
    if (global_consensus_count_ < 2 ||
        global_consensus_translation_m_ <= 0.0 ||
        global_consensus_rotation_deg_ <= 0.0 ||
        max_global_candidates_ < 1 || max_cloud_frames_ < 2 ||
        min_submap_points_ < 100 || rolling_window_s_ <= 0.0 ||
        max_cloud_odom_skew_s_ <= 0.0 || submap_voxel_m_ <= 0.0 ||
        stationary_linear_mps_ <= 0.0 ||
        stationary_angular_radps_ <= 0.0 || stationary_hold_s_ <= 0.0 ||
        tracking_period_s_ <= 0.0 || global_period_s_ <= 0.0 ||
        local_map_radius_m_ <= 0.0 ||
        local_map_z_half_extent_m_ <= 0.0 || vgicp_voxel_m_ <= 0.0 ||
        max_correspondence_m_ <= 0.0 || max_iterations_ < 1 ||
        num_threads_ < 1 || inlier_distance_m_ <= 0.0 ||
        correction_alpha_ <= 0.0 ||
        correction_alpha_ > 1.0 ||
        tracking_limits_.max_fitness <= 0.0 ||
        tracking_limits_.min_inlier_ratio <= 0.0 ||
        tracking_limits_.min_inlier_ratio > 1.0 ||
        tracking_limits_.max_translation_correction_m <= 0.0 ||
        tracking_limits_.max_rotation_correction_rad <= 0.0 ||
        global_limits_.max_fitness <= 0.0 ||
        global_limits_.min_inlier_ratio <= 0.0 ||
        global_limits_.min_inlier_ratio > 1.0 ||
        global_limits_.max_translation_correction_m <= 0.0 ||
        global_limits_.max_rotation_correction_rad <= 0.0 ||
        max_map_to_odom_tilt_deg_ <= 0.0 ||
        max_map_to_odom_tilt_deg_ >= 90.0 ||
        degraded_after_failures_ < 1 ||
        lost_after_failures_ <= degraded_after_failures_ ||
        recovery_confirmations_ < 1) {
      throw std::runtime_error("invalid localization configuration");
    }
  }

  void loadMap() {
    const std::string observed_sha256 = sha256File(map_path_);
    if (observed_sha256 != map_sha256_) {
      throw std::runtime_error("map SHA-256 mismatch; observed " +
                               observed_sha256);
    }

    const std::string extension = lowerExtension(map_path_);
    int result = -1;
    if (extension == ".pcd") {
      result = pcl::io::loadPCDFile<Point>(map_path_, *map_);
    } else if (extension == ".ply") {
      result = pcl::io::loadPLYFile<Point>(map_path_, *map_);
    } else {
      throw std::runtime_error("map must be .pcd or .ply");
    }
    if (result != 0 || map_->empty()) {
      throw std::runtime_error("failed to load non-empty point cloud map");
    }
    Cloud::Ptr finite(new Cloud);
    finite->reserve(map_->size());
    for (const auto& point : *map_) {
      if (pcl::isFinite(point)) {
        finite->push_back(point);
      }
    }
    map_ = finite;
    if (map_->size() < 1000) {
      throw std::runtime_error("map has fewer than 1000 finite points");
    }
  }

  bool lookupBodyToBase(const ros::Time& stamp,
                        Eigen::Isometry3d* body_T_base) {
    try {
      const auto transform = tf_buffer_.lookupTransform(
          body_frame_, base_frame_, stamp, ros::Duration(0.02));
      *body_T_base = tf2::transformToEigen(transform);
      return body_T_base->matrix().allFinite();
    } catch (const std::exception& error) {
      ROS_WARN_STREAM_THROTTLE(
          2.0, "Measured " << body_frame_ << "<-" << base_frame_
                           << " TF is required: " << error.what());
      return false;
    }
  }

  void odomCallback(const nav_msgs::OdometryConstPtr& message) {
    const ros::Time now = ros::Time::now();
    if (message->header.stamp.isZero() ||
        (!now.isZero() &&
         (message->header.stamp - now).toSec() > 0.05)) {
      ROS_ERROR_THROTTLE(2.0, "Rejected zero or future FAST-LIO odometry stamp");
      return;
    }
    if (message->header.frame_id != "camera_init" &&
        message->header.frame_id != odom_frame_) {
      ROS_ERROR_STREAM_THROTTLE(
          2.0, "Unexpected FAST-LIO odom frame "
                   << message->header.frame_id);
      return;
    }
    if (message->child_frame_id != body_frame_) {
      ROS_ERROR_STREAM_THROTTLE(
          2.0, "Unexpected FAST-LIO child frame "
                   << message->child_frame_id << ", expected " << body_frame_);
      return;
    }

    Eigen::Isometry3d odom_T_body;
    try {
      odom_T_body = poseToEigen(message->pose.pose);
    } catch (const std::exception& error) {
      ROS_ERROR_STREAM_THROTTLE(2.0, error.what());
      return;
    }

    double linear_speed = 0.0;
    double angular_speed = 0.0;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (!odom_history_.empty() &&
          message->header.stamp <= odom_history_.back().stamp) {
        odom_history_.clear();
        cloud_frames_.clear();
        consensus_.clear();
        ++state_epoch_;
        last_motion_stamp_ = message->header.stamp;
        last_global_attempt_ = ros::Time();
        last_tracking_attempt_ = ros::Time();
        initialized_ = false;
        raw_state_ =
            ever_initialized_ ? wll::RawState::LOST
                              : wll::RawState::UNINITIALIZED;
        publishDiagnosticLocked("ODOMETRY_TIME_REGRESSION",
                                wll::RegistrationMetrics());
        return;
      }
      if (!odom_history_.empty()) {
        const OdomSample& previous = odom_history_.back();
        const double dt = (message->header.stamp - previous.stamp).toSec();
        if (dt > 1e-4 && dt < 1.0) {
          const Eigen::Isometry3d previous_body_T_body =
              previous.odom_T_body.inverse() * odom_T_body;
          linear_speed = previous_body_T_body.translation().x() / dt;
          const double planar_speed =
              previous_body_T_body.translation().head<2>().norm() / dt;
          angular_speed =
              wll::wrapAngle(wll::yawOf(odom_T_body) -
                             wll::yawOf(previous.odom_T_body)) /
              dt;
          if (planar_speed > stationary_linear_mps_ ||
              std::abs(angular_speed) > stationary_angular_radps_) {
            last_motion_stamp_ = message->header.stamp;
          }
        } else {
          last_motion_stamp_ = message->header.stamp;
        }
      } else {
        last_motion_stamp_ = message->header.stamp;
      }
      odom_history_.push_back({message->header.stamp, odom_T_body});
      while (odom_history_.size() > 400) {
        odom_history_.pop_front();
      }
      latest_linear_speed_ = linear_speed;
      latest_angular_speed_ = angular_speed;
    }

    Eigen::Isometry3d body_T_base;
    if (!lookupBodyToBase(message->header.stamp, &body_T_base)) {
      return;
    }
    const Eigen::Isometry3d odom_T_base = odom_T_body * body_T_base;

    nav_msgs::Odometry canonical = *message;
    canonical.header.frame_id = odom_frame_;
    canonical.child_frame_id = base_frame_;
    canonical.pose.pose = eigenToPose(odom_T_base);
    canonical.twist.twist.linear.x = linear_speed;
    canonical.twist.twist.angular.z = angular_speed;
    canonical_odom_pub_.publish(canonical);

    geometry_msgs::TransformStamped transform;
    transform.header = canonical.header;
    transform.child_frame_id = base_frame_;
    transform.transform = eigenToTransform(odom_T_base);
    odom_tf_broadcaster_.sendTransform(transform);
  }

  void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& message) {
    const ros::Time now = ros::Time::now();
    if (message->header.stamp.isZero() ||
        (!now.isZero() &&
         (message->header.stamp - now).toSec() > 0.05)) {
      ROS_ERROR_THROTTLE(2.0, "Rejected zero or future body-cloud stamp");
      return;
    }
    if (message->header.frame_id != body_frame_) {
      ROS_ERROR_STREAM_THROTTLE(
          2.0, "Body cloud frame " << message->header.frame_id
                                    << " does not match " << body_frame_);
      return;
    }
    Cloud::Ptr cloud(new Cloud);
    pcl::fromROSMsg(*message, *cloud);
    Cloud::Ptr finite(new Cloud);
    finite->reserve(cloud->size());
    for (const auto& point : *cloud) {
      if (pcl::isFinite(point)) {
        finite->push_back(point);
      }
    }
    if (finite->empty()) {
      return;
    }

    std::lock_guard<std::mutex> lock(mutex_);
    if (odom_history_.empty()) {
      return;
    }
    if (!cloud_frames_.empty() &&
        message->header.stamp <= cloud_frames_.back().stamp) {
      cloud_frames_.clear();
      consensus_.clear();
      ++state_epoch_;
      last_global_attempt_ = ros::Time();
      last_tracking_attempt_ = ros::Time();
      initialized_ = false;
      raw_state_ =
          ever_initialized_ ? wll::RawState::LOST
                            : wll::RawState::UNINITIALIZED;
      publishDiagnosticLocked("CLOUD_TIME_REGRESSION",
                              wll::RegistrationMetrics());
      return;
    }
    const OdomSample* nearest = nullptr;
    double nearest_delta = std::numeric_limits<double>::infinity();
    for (auto iterator = odom_history_.rbegin();
         iterator != odom_history_.rend(); ++iterator) {
      const double delta =
          std::abs((message->header.stamp - iterator->stamp).toSec());
      if (delta < nearest_delta) {
        nearest = &*iterator;
        nearest_delta = delta;
      }
      if (iterator->stamp + ros::Duration(max_cloud_odom_skew_s_) <
          message->header.stamp) {
        break;
      }
    }
    if (nearest == nullptr || nearest_delta > max_cloud_odom_skew_s_) {
      ROS_WARN_STREAM_THROTTLE(2.0, "Cloud/odometry skew " << nearest_delta);
      return;
    }
    cloud_frames_.push_back(
        {message->header.stamp, nearest->odom_T_body, finite});
    while (static_cast<int>(cloud_frames_.size()) > max_cloud_frames_) {
      cloud_frames_.pop_front();
    }
    while (!cloud_frames_.empty() &&
           (message->header.stamp - cloud_frames_.front().stamp).toSec() >
               rolling_window_s_) {
      cloud_frames_.pop_front();
    }
  }

  bool isStationaryLocked(const ros::Time& stamp) const {
    return !last_motion_stamp_.isZero() &&
           (stamp - last_motion_stamp_).toSec() >= stationary_hold_s_ &&
           std::abs(latest_linear_speed_) <= stationary_linear_mps_ &&
           std::abs(latest_angular_speed_) <= stationary_angular_radps_;
  }

  SubmapSnapshot buildSubmapLocked() const {
    SubmapSnapshot result;
    result.cloud.reset(new Cloud);
    if (cloud_frames_.empty()) {
      return result;
    }
    const CloudFrame& latest = cloud_frames_.back();
    result.stamp = latest.stamp;
    result.odom_T_body = latest.odom_T_body;
    result.state_epoch = state_epoch_;
    result.reset_count = reset_count_;
    for (const auto& frame : cloud_frames_) {
      Cloud transformed;
      const Eigen::Isometry3d latest_body_T_frame_body =
          latest.odom_T_body.inverse() * frame.odom_T_body;
      pcl::transformPointCloud(*frame.cloud, transformed,
                               latest_body_T_frame_body.matrix().cast<float>());
      *result.cloud += transformed;
    }
    pcl::VoxelGrid<Point> voxel;
    voxel.setLeafSize(submap_voxel_m_, submap_voxel_m_, submap_voxel_m_);
    voxel.setInputCloud(result.cloud);
    Cloud::Ptr filtered(new Cloud);
    voxel.filter(*filtered);
    result.cloud = filtered;
    return result;
  }

  Cloud::Ptr cropTarget(const Eigen::Vector3d& center) const {
    pcl::CropBox<Point> crop;
    crop.setInputCloud(map_);
    crop.setMin(Eigen::Vector4f(
        center.x() - local_map_radius_m_,
        center.y() - local_map_radius_m_,
        center.z() - local_map_z_half_extent_m_, 1.0f));
    crop.setMax(Eigen::Vector4f(
        center.x() + local_map_radius_m_,
        center.y() + local_map_radius_m_,
        center.z() + local_map_z_half_extent_m_, 1.0f));
    Cloud::Ptr result(new Cloud);
    crop.filter(*result);
    return result;
  }

  double inlierRatio(const Cloud::ConstPtr& aligned,
                     const Cloud::ConstPtr& target) const {
    if (aligned->empty() || target->empty()) {
      return 0.0;
    }
    pcl::KdTreeFLANN<Point> tree;
    tree.setInputCloud(target);
    std::vector<int> index(1);
    std::vector<float> squared_distance(1);
    std::size_t inliers = 0;
    const double threshold_squared = inlier_distance_m_ * inlier_distance_m_;
    for (const auto& point : *aligned) {
      if (tree.nearestKSearch(point, 1, index, squared_distance) == 1 &&
          squared_distance[0] <= threshold_squared) {
        ++inliers;
      }
    }
    return static_cast<double>(inliers) /
           static_cast<double>(aligned->size());
  }

  RegistrationResult registerCloud(
      const Cloud::ConstPtr& source, const Eigen::Isometry3d& initial_guess,
      const wll::AcceptanceLimits& limits) const {
    RegistrationResult result;
    result.aligned.reset(new Cloud);
    Cloud::Ptr target = cropTarget(initial_guess.translation());
    result.metrics.source_points = source->size();
    result.metrics.target_points = target->size();
    if (source->size() < limits.min_source_points ||
        target->size() < limits.min_target_points) {
      return result;
    }

    fast_gicp::FastVGICP<Point, Point> registration;
    registration.setNumThreads(std::max(1, num_threads_));
    registration.setResolution(vgicp_voxel_m_);
    registration.setMaximumIterations(std::max(1, max_iterations_));
    registration.setMaxCorrespondenceDistance(max_correspondence_m_);
    registration.setInputSource(source);
    registration.setInputTarget(target);
    registration.align(*result.aligned,
                       initial_guess.matrix().cast<float>());

    result.metrics.converged = registration.hasConverged();
    if (!result.metrics.converged) {
      return result;
    }
    result.map_T_body =
        Eigen::Isometry3d(registration.getFinalTransformation().cast<double>());
    result.metrics.fitness =
        registration.getFitnessScore(max_correspondence_m_);
    result.metrics.inlier_ratio = inlierRatio(result.aligned, target);
    const Eigen::Isometry3d correction =
        initial_guess.inverse() * result.map_T_body;
    result.metrics.translation_correction_m =
        correction.translation().norm();
    result.metrics.rotation_correction_rad =
        wll::rotationDistance(Eigen::Isometry3d::Identity(), correction);
    return result;
  }

  bool configureGlobalEngine() {
    if (global_engine_ready_) {
      return true;
    }
    if (!set_engine_client_.exists() || !set_map_client_.exists() ||
        !global_query_client_.exists()) {
      return false;
    }

    hdl_global_localization::SetGlobalLocalizationEngine engine;
    engine.request.engine_name.data = global_engine_;
    if (!set_engine_client_.call(engine)) {
      ROS_FATAL_STREAM(
          "FPFH_TEASER is unavailable. Build hdl_global_localization "
          "with TEASER++; refusing FPFH_RANSAC fallback.");
      global_engine_failed_ = true;
      publishDiagnostic("TEASER_ENGINE_UNAVAILABLE",
                        wll::RegistrationMetrics());
      return false;
    }

    hdl_global_localization::SetGlobalMap set_map;
    pcl::toROSMsg(*map_, set_map.request.global_map);
    set_map.request.global_map.header.frame_id = map_frame_;
    set_map.request.global_map.header.stamp = ros::Time::now();
    if (!set_map_client_.call(set_map)) {
      ROS_ERROR("Failed to load immutable map into global localizer");
      return false;
    }
    global_engine_ready_ = true;
    ROS_INFO("FPFH_TEASER global localization engine is ready");
    return true;
  }

  bool runGlobalInitialization(const SubmapSnapshot& snapshot) {
    hdl_global_localization::QueryGlobalLocalization query;
    query.request.max_num_candidates = max_global_candidates_;
    pcl::toROSMsg(*snapshot.cloud, query.request.cloud);
    query.request.cloud.header.frame_id = body_frame_;
    query.request.cloud.header.stamp = snapshot.stamp;
    if (!global_query_client_.call(query) ||
        query.response.poses.empty()) {
      {
        std::lock_guard<std::mutex> lock(mutex_);
        consensus_.clear();
      }
      publishDiagnostic("GLOBAL_QUERY_FAILED", wll::RegistrationMetrics());
      return false;
    }

    RegistrationResult best;
    bool found = false;
    for (const auto& candidate_pose : query.response.poses) {
      Eigen::Isometry3d candidate;
      try {
        candidate = poseToEigen(candidate_pose);
      } catch (const std::exception&) {
        continue;
      }
      RegistrationResult refined =
          registerCloud(snapshot.cloud, candidate, global_limits_);
      if (!wll::registrationAccepted(refined.metrics, global_limits_)) {
        continue;
      }
      if (!found ||
          refined.metrics.inlier_ratio > best.metrics.inlier_ratio ||
          (refined.metrics.inlier_ratio == best.metrics.inlier_ratio &&
           refined.metrics.fitness < best.metrics.fitness)) {
        best = refined;
        found = true;
      }
    }
    if (!found) {
      {
        std::lock_guard<std::mutex> lock(mutex_);
        consensus_.clear();
      }
      publishDiagnostic("GLOBAL_REFINEMENT_REJECTED", best.metrics);
      return false;
    }

    const Eigen::Isometry3d candidate_map_T_odom =
        best.map_T_body * snapshot.odom_T_body.inverse();
    if (wll::tiltAngle(candidate_map_T_odom) >
        max_map_to_odom_tilt_deg_ * kPi / 180.0) {
      {
        std::lock_guard<std::mutex> lock(mutex_);
        consensus_.clear();
      }
      publishDiagnostic("GLOBAL_GRAVITY_ALIGNMENT_REJECTED", best.metrics);
      return false;
    }
    bool consensus_ready = false;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (snapshot.state_epoch != state_epoch_) {
        return false;
      }
      consensus_ready = consensus_.add(candidate_map_T_odom);
    }
    if (!consensus_ready) {
      publishDiagnostic("GLOBAL_CONSENSUS", best.metrics);
      return false;
    }

    Eigen::Isometry3d accepted_map_T_odom;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (snapshot.state_epoch != state_epoch_) {
        return false;
      }
      map_T_odom_ = consensus_.mean();
      accepted_map_T_odom = map_T_odom_;
      initialized_ = true;
      ever_initialized_ = true;
      relocalization_requested_ = false;
      failures_ = 0;
      recoveries_ = 0;
      raw_state_ = wll::RawState::OK;
      last_good_stamp_ = snapshot.stamp;
    }
    const Eigen::Isometry3d accepted_map_T_body =
        accepted_map_T_odom * snapshot.odom_T_body;
    publishPose(snapshot, accepted_map_T_body, best.metrics, best.aligned);
    publishDiagnostic("TRACKING", best.metrics);
    return true;
  }

  void runTracking(const SubmapSnapshot& snapshot) {
    Eigen::Isometry3d predicted_map_T_body;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (snapshot.state_epoch != state_epoch_ || !initialized_) {
        return;
      }
      predicted_map_T_body = map_T_odom_ * snapshot.odom_T_body;
    }
    RegistrationResult result =
        registerCloud(snapshot.cloud, predicted_map_T_body, tracking_limits_);
    const Eigen::Isometry3d measured_map_T_odom =
        result.map_T_body * snapshot.odom_T_body.inverse();
    const bool gravity_aligned =
        wll::tiltAngle(measured_map_T_odom) <=
        max_map_to_odom_tilt_deg_ * kPi / 180.0;
    if (!wll::registrationAccepted(result.metrics, tracking_limits_) ||
        !gravity_aligned) {
      std::lock_guard<std::mutex> lock(mutex_);
      if (snapshot.state_epoch != state_epoch_) {
        return;
      }
      ++failures_;
      recoveries_ = 0;
      if (failures_ >= lost_after_failures_) {
        raw_state_ = wll::RawState::LOST;
        initialized_ = false;
      } else if (failures_ >= degraded_after_failures_) {
        raw_state_ = wll::RawState::DEGRADED;
      }
      publishDiagnosticLocked(raw_state_ == wll::RawState::LOST
                                  ? "LOST_EXPLICIT_RELOCALIZATION_REQUIRED"
                                  : (gravity_aligned
                                         ? "TRACKING_REJECTED"
                                         : "TRACKING_GRAVITY_ALIGNMENT_REJECTED"),
                              result.metrics);
      return;
    }

    Eigen::Isometry3d output_map_T_body;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (snapshot.state_epoch != state_epoch_ || !initialized_) {
        return;
      }
      map_T_odom_ =
          interpolate(map_T_odom_, measured_map_T_odom, correction_alpha_);
      output_map_T_body = map_T_odom_ * snapshot.odom_T_body;
      failures_ = 0;
      ++recoveries_;
      if (raw_state_ == wll::RawState::DEGRADED &&
          recoveries_ < recovery_confirmations_) {
        publishDiagnosticLocked("RECOVERY_CONFIRMING", result.metrics);
        return;
      }
      raw_state_ = wll::RawState::OK;
      last_good_stamp_ = snapshot.stamp;
    }
    publishPose(snapshot, output_map_T_body, result.metrics, result.aligned);
    publishDiagnostic("TRACKING", result.metrics);
  }

  void publishPose(const SubmapSnapshot& snapshot,
                   const Eigen::Isometry3d& map_T_body,
                   const wll::RegistrationMetrics& metrics,
                   const Cloud::ConstPtr& aligned) {
    Eigen::Isometry3d body_T_base;
    if (!lookupBodyToBase(snapshot.stamp, &body_T_base)) {
      publishDiagnostic("BASE_EXTRINSIC_MISSING", metrics);
      return;
    }
    const Eigen::Isometry3d map_T_base = map_T_body * body_T_base;
    geometry_msgs::PoseWithCovarianceStamped pose;
    pose.header.frame_id = map_frame_;
    pose.header.stamp = snapshot.stamp;
    pose.header.seq = snapshot.reset_count;
    pose.pose.pose = eigenToPose(map_T_base);
    std::fill(pose.pose.covariance.begin(), pose.pose.covariance.end(), 0.0);
    const double position_sigma =
        std::max(0.05, std::min(1.0, std::sqrt(metrics.fitness)));
    const double yaw_sigma =
        std::max(2.0 * kPi / 180.0,
                 std::min(20.0 * kPi / 180.0,
                          metrics.rotation_correction_rad +
                              2.0 * kPi / 180.0));
    pose.pose.covariance[0] = position_sigma * position_sigma;
    pose.pose.covariance[7] = position_sigma * position_sigma;
    pose.pose.covariance[14] =
        4.0 * position_sigma * position_sigma;
    pose.pose.covariance[21] = 0.25;
    pose.pose.covariance[28] = 0.25;
    pose.pose.covariance[35] = yaw_sigma * yaw_sigma;
    pose_pub_.publish(pose);

    if (aligned && !aligned->empty()) {
      sensor_msgs::PointCloud2 cloud_message;
      pcl::toROSMsg(*aligned, cloud_message);
      cloud_message.header.frame_id = map_frame_;
      cloud_message.header.stamp = snapshot.stamp;
      aligned_pub_.publish(cloud_message);
    }
  }

  void publishDiagnostic(const std::string& text,
                         const wll::RegistrationMetrics& metrics) {
    std::lock_guard<std::mutex> lock(mutex_);
    publishDiagnosticLocked(text, metrics);
  }

  void publishDiagnosticLocked(const std::string& text,
                               const wll::RegistrationMetrics& metrics) {
    diagnostic_msgs::DiagnosticArray array;
    array.header.stamp = ros::Time::now();
    diagnostic_msgs::DiagnosticStatus status;
    status.name = "wheelchair_lidar_localization";
    status.hardware_id = "livox_mid360_builtin_imu";
    status.message = text;
    status.level =
        raw_state_ == wll::RawState::OK
            ? diagnostic_msgs::DiagnosticStatus::OK
            : (raw_state_ == wll::RawState::DEGRADED
                   ? diagnostic_msgs::DiagnosticStatus::WARN
                   : diagnostic_msgs::DiagnosticStatus::ERROR);
    status.values.push_back(
        keyValue("raw_state",
                 std::to_string(static_cast<unsigned int>(raw_state_))));
    status.values.push_back(
        keyValue("reset_count", std::to_string(reset_count_)));
    status.values.push_back(keyValue("map_id", map_id_));
    status.values.push_back(keyValue("map_sha256", map_sha256_));
    status.values.push_back(keyValue("global_engine", global_engine_));
    status.values.push_back(
        keyValue("raw_score", asString(metrics.inlier_ratio)));
    status.values.push_back(
        keyValue("fitness", asString(metrics.fitness)));
    status.values.push_back(
        keyValue("scan_residual_m",
                 asString(std::sqrt(std::max(0.0, metrics.fitness)))));
    status.values.push_back(
        keyValue("inlier_ratio", asString(metrics.inlier_ratio)));
    status.values.push_back(
        keyValue("rotation_correction_rad",
                 asString(metrics.rotation_correction_rad)));
    status.values.push_back(
        keyValue("map_to_odom_tilt_rad",
                 asString(wll::tiltAngle(map_T_odom_))));
    status.values.push_back(
        keyValue("source_points", std::to_string(metrics.source_points)));
    status.values.push_back(
        keyValue("target_points", std::to_string(metrics.target_points)));
    status.values.push_back(
        keyValue("global_consensus_count",
                 std::to_string(consensus_.size())));
    status.values.push_back(
        keyValue("body_to_base_tf_required", "true"));
    status.values.push_back(
        keyValue("automatic_resume_after_lost", "false"));
    status.values.push_back(
        keyValue("pose_header_seq_binding", "reset_count"));
    array.status.push_back(status);
    diagnostics_pub_.publish(array);
  }

  void initialPoseCallback(
      const geometry_msgs::PoseWithCovarianceStampedConstPtr& message) {
    std::lock_guard<std::mutex> lock(mutex_);
    const ros::Time now = ros::Time::now();
    const double age_s = (now - message->header.stamp).toSec();
    if (message->header.frame_id != map_frame_ ||
        message->header.stamp.isZero() || age_s < -0.05 || age_s > 0.25) {
      ROS_WARN("Rejected stale or wrong-frame /initialpose request");
      return;
    }
    if (odom_history_.empty() || !isStationaryLocked(now)) {
      ROS_WARN("Rejected /initialpose request while chair is not stationary");
      return;
    }
    if (reset_count_ == std::numeric_limits<std::uint32_t>::max()) {
      initialized_ = false;
      raw_state_ = wll::RawState::LOST;
      publishDiagnosticLocked("RESET_COUNTER_EXHAUSTED",
                              wll::RegistrationMetrics());
      return;
    }
    ++reset_count_;
    ++state_epoch_;
    initialized_ = false;
    relocalization_requested_ = true;
    consensus_.clear();
    last_global_attempt_ = ros::Time();
    last_tracking_attempt_ = ros::Time();
    raw_state_ = wll::RawState::INITIALIZING;
    failures_ = 0;
    recoveries_ = 0;
    publishDiagnosticLocked("RELOCALIZING", wll::RegistrationMetrics());
    ROS_INFO("Explicit /initialpose relocalization accepted");
  }

  void timerCallback(const ros::TimerEvent&) {
    if (processing_.exchange(true)) {
      return;
    }
    ProcessingFlag guard(processing_);
    if (global_engine_failed_ || !configureGlobalEngine()) {
      return;
    }

    SubmapSnapshot snapshot;
    bool stationary = false;
    bool should_initialize = false;
    bool should_track = false;
    bool attempt_due = false;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      snapshot = buildSubmapLocked();
      if (snapshot.cloud->size() <
          static_cast<std::size_t>(min_submap_points_)) {
        return;
      }
      stationary = isStationaryLocked(snapshot.stamp);
      should_initialize =
          !initialized_ &&
          (relocalization_requested_ ||
           (automatic_initialization_ && !ever_initialized_));
      should_track = initialized_;
      if (should_initialize) {
        raw_state_ = wll::RawState::INITIALIZING;
        if (stationary &&
            (last_global_attempt_.isZero() ||
             (snapshot.stamp - last_global_attempt_).toSec() >=
                 global_period_s_)) {
          last_global_attempt_ = snapshot.stamp;
          attempt_due = true;
        }
      } else if (should_track &&
                 (last_tracking_attempt_.isZero() ||
                  (snapshot.stamp - last_tracking_attempt_).toSec() >=
                      tracking_period_s_)) {
        last_tracking_attempt_ = snapshot.stamp;
        attempt_due = true;
      }
    }

    if (should_initialize) {
      if (!stationary) {
        publishDiagnostic("INITIALIZATION_REQUIRES_STATIONARY",
                          wll::RegistrationMetrics());
        return;
      }
      if (!attempt_due) {
        return;
      }
      runGlobalInitialization(snapshot);
      return;
    }

    if (should_track) {
      if (!attempt_due) {
        return;
      }
      runTracking(snapshot);
    }
  }

  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  ros::Subscriber odom_sub_;
  ros::Subscriber cloud_sub_;
  ros::Subscriber initial_pose_sub_;
  ros::Publisher pose_pub_;
  ros::Publisher diagnostics_pub_;
  ros::Publisher canonical_odom_pub_;
  ros::Publisher aligned_pub_;
  ros::ServiceClient set_engine_client_;
  ros::ServiceClient set_map_client_;
  ros::ServiceClient global_query_client_;
  ros::Timer timer_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  tf2_ros::TransformBroadcaster odom_tf_broadcaster_;

  std::mutex mutex_;
  std::atomic<bool> processing_{false};
  std::deque<OdomSample> odom_history_;
  std::deque<CloudFrame> cloud_frames_;
  Cloud::Ptr map_;
  wll::PoseConsensus consensus_;
  wll::AcceptanceLimits tracking_limits_;
  wll::AcceptanceLimits global_limits_;
  Eigen::Isometry3d map_T_odom_ = Eigen::Isometry3d::Identity();
  wll::RawState raw_state_ = wll::RawState::UNINITIALIZED;

  std::string map_path_;
  std::string map_sha256_;
  std::string map_id_;
  std::string map_frame_;
  std::string odom_frame_;
  std::string body_frame_;
  std::string base_frame_;
  std::string fastlio_odom_topic_;
  std::string fastlio_body_cloud_topic_;
  std::string global_engine_;

  bool automatic_initialization_ = true;
  bool initialized_ = false;
  bool ever_initialized_ = false;
  bool relocalization_requested_ = false;
  bool global_engine_ready_ = false;
  bool global_engine_failed_ = false;
  double rolling_window_s_ = 1.5;
  int max_cloud_frames_ = 15;
  double max_cloud_odom_skew_s_ = 0.08;
  double submap_voxel_m_ = 0.20;
  int min_submap_points_ = 700;
  double stationary_linear_mps_ = 0.03;
  double stationary_angular_radps_ = 0.05;
  double stationary_hold_s_ = 2.0;
  double tracking_period_s_ = 0.50;
  double global_period_s_ = 1.0;
  double local_map_radius_m_ = 18.0;
  double local_map_z_half_extent_m_ = 4.0;
  double vgicp_voxel_m_ = 0.30;
  double max_correspondence_m_ = 1.0;
  int max_iterations_ = 64;
  int num_threads_ = 4;
  double inlier_distance_m_ = 0.30;
  double correction_alpha_ = 0.25;
  int global_consensus_count_ = 3;
  double global_consensus_translation_m_ = 0.25;
  double global_consensus_rotation_deg_ = 4.0;
  double max_map_to_odom_tilt_deg_ = 5.0;
  int max_global_candidates_ = 5;
  int degraded_after_failures_ = 1;
  int lost_after_failures_ = 6;
  int recovery_confirmations_ = 3;
  std::uint32_t reset_count_ = 0;
  std::uint64_t state_epoch_ = 0;
  int failures_ = 0;
  int recoveries_ = 0;
  double latest_linear_speed_ = 0.0;
  double latest_angular_speed_ = 0.0;
  ros::Time last_motion_stamp_;
  ros::Time last_good_stamp_;
  ros::Time last_global_attempt_;
  ros::Time last_tracking_attempt_;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "wheelchair_map_localizer");
  try {
    WheelchairMapLocalizer localizer;
    ros::AsyncSpinner spinner(3);
    spinner.start();
    ros::waitForShutdown();
  } catch (const std::exception& error) {
    ROS_FATAL_STREAM(error.what());
    return 2;
  }
  return 0;
}

#include <openssl/sha.h>

#include <algorithm>
#include <cmath>
#include <deque>
#include <fstream>
#include <iomanip>
#include <limits>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <string>

#include <diagnostic_msgs/DiagnosticArray.h>
#include <diagnostic_msgs/DiagnosticStatus.h>
#include <diagnostic_msgs/KeyValue.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <pcl/common/point_tests.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_srvs/SetBool.h>
#include <tf2_ros/transform_broadcaster.h>

#include "static_livox_localization/assisted_alignment.hpp"
#include "static_livox_localization/moving_tracker.hpp"
#include "static_livox_localization/registration.hpp"
#include "static_livox_localization/rolling_submap.hpp"

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
  for (unsigned char byte : digest) {
    out << std::hex << std::setw(2) << std::setfill('0')
        << static_cast<int>(byte);
  }
  return out.str();
}

diagnostic_msgs::KeyValue key_value(const std::string& key,
                                    const std::string& value) {
  diagnostic_msgs::KeyValue item;
  item.key = key;
  item.value = value;
  return item;
}

Eigen::Isometry3d pose_to_eigen(const geometry_msgs::Pose& pose) {
  Eigen::Quaterniond q(pose.orientation.w, pose.orientation.x,
                       pose.orientation.y, pose.orientation.z);
  if (!std::isfinite(q.norm()) || q.norm() < 1e-6) {
    throw std::runtime_error("invalid pose quaternion");
  }
  q.normalize();
  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
  transform.linear() = q.toRotationMatrix();
  transform.translation() =
      Eigen::Vector3d(pose.position.x, pose.position.y, pose.position.z);
  return transform;
}

geometry_msgs::Pose eigen_to_pose(const Eigen::Isometry3d& transform) {
  geometry_msgs::Pose pose;
  pose.position.x = transform.translation().x();
  pose.position.y = transform.translation().y();
  pose.position.z = transform.translation().z();
  Eigen::Quaterniond q(transform.rotation());
  q.normalize();
  pose.orientation.x = q.x();
  pose.orientation.y = q.y();
  pose.orientation.z = q.z();
  pose.orientation.w = q.w();
  return pose;
}

geometry_msgs::Transform eigen_to_transform(
    const Eigen::Isometry3d& transform) {
  geometry_msgs::Transform message;
  message.translation.x = transform.translation().x();
  message.translation.y = transform.translation().y();
  message.translation.z = transform.translation().z();
  Eigen::Quaterniond q(transform.rotation());
  q.normalize();
  message.rotation.x = q.x();
  message.rotation.y = q.y();
  message.rotation.z = q.z();
  message.rotation.w = q.w();
  return message;
}

}  // namespace

class MovingIcpLocalizer {
 public:
  using Cloud = pcl::PointCloud<pcl::PointXYZI>;
  using RegistrationResult = static_livox_localization::RegistrationResult;
  using CorrectionDecision = static_livox_localization::CorrectionDecision;
  using TrackingState = static_livox_localization::TrackingState;

  using ConsensusDecision = static_livox_localization::ConsensusDecision;
  MovingIcpLocalizer()
      : private_nh_("~"),
        map_(new Cloud),
        rolling_submap_(rolling_config_),
        state_machine_(tracking_config_),
        alignment_controller_(alignment_config_) {
    load_parameters();
    rolling_submap_ = static_livox_localization::RollingSubmap(rolling_config_);
    state_machine_ =
        static_livox_localization::TrackingStateMachine(tracking_config_);
    alignment_controller_ =
        static_livox_localization::AssistedAlignmentController(alignment_config_);
    load_fixed_map();

    pose_pub_ = nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>(
        "/fast_lio_icp/pose", 20);
    path_pub_ = nh_.advertise<nav_msgs::Path>("/fast_lio_icp/path", 2, true);
    diagnostics_pub_ = nh_.advertise<diagnostic_msgs::DiagnosticArray>(
        "/fast_lio_icp/localization_diagnostics", 10, true);
    seed_sub_ = nh_.subscribe(seed_topic_, 1,
                              &MovingIcpLocalizer::seed_callback, this);
    odom_sub_ = nh_.subscribe(odom_topic_, 100,
                              &MovingIcpLocalizer::odom_callback, this);
    cloud_sub_ = nh_.subscribe(cloud_topic_, 10,
                               &MovingIcpLocalizer::cloud_callback, this);

    auto_correction_service_ = nh_.advertiseService("/fast_lio_icp/enable_auto_correction",
                                                    &MovingIcpLocalizer::set_auto_correction,
                                                    this);
    path_.header.frame_id = map_frame_;
    std::lock_guard<std::mutex> lock(mutex_);
    publish_diagnostic_locked("WAITING_FOR_INITIALPOSE", RegistrationResult(),
                              CorrectionDecision());
    ROS_INFO("Loaded immutable map %s (%zu points); expecting %s -> %s",
             map_id_.c_str(), map_->size(), odom_frame_.c_str(),
             base_frame_.c_str());
  }

 private:
  struct OdomSample {
    ros::Time stamp;
    Eigen::Isometry3d odom_T_base = Eigen::Isometry3d::Identity();
    boost::array<double, 36> covariance{};
  };

  void load_parameters() {
    private_nh_.param<std::string>("map_path", map_path_, "");
    private_nh_.param<std::string>("map_sha256", map_sha256_, "");
    private_nh_.param<std::string>("map_id", map_id_, "livox_raw_20260707");
    private_nh_.param<std::string>("map_frame", map_frame_, "map");
    private_nh_.param<std::string>("odom_frame", odom_frame_, "camera_init");
    private_nh_.param<std::string>("base_frame", base_frame_, "body");
    private_nh_.param<std::string>("cloud_topic", cloud_topic_,
                                   "/cloud_registered_body");
    private_nh_.param<std::string>("odom_topic", odom_topic_, "/Odometry");
    private_nh_.param<std::string>("seed_topic", seed_topic_,
                                   "/fast_lio_icp/initialpose");
    private_nh_.param("initialization_window_s", initialization_window_s_, 3.0);
    private_nh_.param("correction_period_s", correction_period_s_, 1.0);

    private_nh_.param("voxel_resolution", registration_config_.voxel_resolution,
                      0.20);
    private_nh_.param("roi_radius", registration_config_.roi_radius, 20.0);
    private_nh_.param("roi_z_half_extent",
                      registration_config_.roi_z_half_extent, 5.0);
    private_nh_.param("max_correspondence",
                      registration_config_.max_correspondence, 1.0);
    private_nh_.param("max_iterations", registration_config_.max_iterations,
                      64);
    private_nh_.param("min_points", registration_config_.min_points, 500);
    private_nh_.param("max_fitness", registration_config_.max_fitness, 0.20);
    private_nh_.param("registration_max_seed_translation_m",
                      registration_config_.max_seed_translation, 3.0);
    double registration_rotation_deg = 30.0;
    private_nh_.param("registration_max_seed_rotation_deg",
                      registration_rotation_deg, 30.0);
    registration_config_.max_seed_rotation_rad =
        registration_rotation_deg * M_PI / 180.0;

    private_nh_.param("max_fitness", tracking_config_.max_fitness, 0.20);
    private_nh_.param("min_inlier_ratio", tracking_config_.min_inlier_ratio,
                      0.35);
    tracking_config_.min_source_points = registration_config_.min_points;
    tracking_config_.min_target_points = registration_config_.min_points;
    private_nh_.param("max_prediction_translation_m",
                      tracking_config_.max_prediction_translation_m, 1.0);
    double prediction_yaw_deg = 20.0;
    private_nh_.param("max_prediction_yaw_deg", prediction_yaw_deg, 20.0);
    tracking_config_.max_prediction_rotation_rad =
        prediction_yaw_deg * M_PI / 180.0;
    private_nh_.param("max_correction_translation_m",
                      tracking_config_.max_correction_translation_m, 0.30);
    double correction_yaw_deg = 5.0;
    private_nh_.param("max_correction_yaw_deg", correction_yaw_deg, 5.0);
    tracking_config_.max_correction_rotation_rad =
        correction_yaw_deg * M_PI / 180.0;
    private_nh_.param("degraded_after_failures",
                      tracking_config_.degraded_after_failures, 1);
    private_nh_.param("lost_after_s", tracking_config_.lost_after_s, 8.0);
    private_nh_.param("recovery_confirmations",
                      tracking_config_.recovery_confirmations, 2);
    private_nh_.param("auto_correction_on_start", auto_correction_on_start_,
                      false);
    private_nh_.param("required_consistent_candidates",
                      alignment_config_.required_consistent_candidates, 3);
    private_nh_.param("candidate_translation_tolerance_m",
                      alignment_config_.candidate_translation_tolerance_m, 0.20);
    double candidate_yaw_tolerance_deg = 3.0;
    private_nh_.param("candidate_yaw_tolerance_deg",
                      candidate_yaw_tolerance_deg, 3.0);
    alignment_config_.candidate_rotation_tolerance_rad =
        candidate_yaw_tolerance_deg * M_PI / 180.0;


    private_nh_.param("rolling_window_s", rolling_config_.window_s, 2.0);
    private_nh_.param("voxel_resolution", rolling_config_.voxel_resolution,
                      0.20);
    private_nh_.param("max_cloud_odom_skew_s",
                      rolling_config_.max_stamp_skew_s, 0.10);
    rolling_config_.expected_cloud_frame = base_frame_;
    int max_samples = 20;
    int max_points = 120000;
    private_nh_.param("max_cloud_samples", max_samples, 20);
    private_nh_.param("max_submap_points", max_points, 120000);
    rolling_config_.max_samples = static_cast<std::size_t>(std::max(1, max_samples));
    rolling_config_.max_stored_points =
        static_cast<std::size_t>(std::max(1, max_points));
    private_nh_.param("path_max_poses", path_max_poses_, 2000);
  }

  void load_fixed_map() {
    if (map_path_.empty() || map_sha256_.size() != 64) {
      throw std::runtime_error("map_path and 64-char map_sha256 are required");
    }
    const std::string observed = sha256_file(map_path_);
    if (observed != map_sha256_) {
      throw std::runtime_error("map SHA-256 mismatch: " + observed);
    }
    if (pcl::io::loadPCDFile(map_path_, *map_) != 0 || map_->empty()) {
      throw std::runtime_error("failed to load non-empty XYZI PCD map");
    }
  }

  bool set_auto_correction(std_srvs::SetBool::Request& request,
                           std_srvs::SetBool::Response& response) {
    std::lock_guard<std::mutex> lock(mutex_);
    response.success =
        alignment_controller_.set_auto_correction(request.data);
    if (response.success && request.data) {
      rolling_submap_.clear();
      correction_in_progress_ = false;
      initialization_start_stamp_s_ = ros::Time::now().toSec();
      last_correction_stamp_s_ = -1.0;
    }
    response.message =
        response.success
            ? static_livox_localization::alignment_state_name(
                  alignment_controller_.state())
            : "INITIAL_POSE_REQUIRED";
    publish_diagnostic_locked(response.message, RegistrationResult(),
                              CorrectionDecision());
    return true;
  }

  void seed_callback(
      const geometry_msgs::PoseWithCovarianceStampedConstPtr& message) {
    try {
      if (!message->header.frame_id.empty() &&
          message->header.frame_id != map_frame_) {
        throw std::runtime_error("initial pose must be expressed in map frame");
      }
      const Eigen::Isometry3d seed = pose_to_eigen(message->pose.pose);
      std::lock_guard<std::mutex> lock(mutex_);
      seed_map_T_base_ = seed;
      has_seed_ = true;
      alignment_controller_.on_seed();
      has_map_T_odom_ = false;
      has_seed_map_T_odom_guess_ = false;
      correction_in_progress_ = false;
      initialization_start_stamp_s_ = -1.0;
      last_correction_stamp_s_ = -1.0;
      rolling_submap_.clear();
      state_machine_ =
          static_livox_localization::TrackingStateMachine(tracking_config_);
      path_.poses.clear();
      ++reset_count_;
      if (has_latest_odom_) {
        seed_map_T_odom_guess_ =
            seed_map_T_base_ * latest_odom_.odom_T_base.inverse();
        has_seed_map_T_odom_guess_ = true;
        map_T_odom_ = seed_map_T_odom_guess_;
        has_map_T_odom_ = true;
        publish_pose_tf_path_locked(latest_odom_);
      }
      publish_diagnostic_locked("MANUAL_ALIGN",
                                RegistrationResult(), CorrectionDecision());
    } catch (const std::exception& error) {
      ROS_ERROR("Rejected initial pose: %s", error.what());
    }
  }

  void odom_callback(const nav_msgs::OdometryConstPtr& message) {
    if (message->header.frame_id != odom_frame_ ||
        message->child_frame_id != base_frame_) {
      std::lock_guard<std::mutex> lock(mutex_);
      publish_diagnostic_locked("ODOMETRY_FRAME_MISMATCH",
                                RegistrationResult(), CorrectionDecision());
      return;
    }
    OdomSample sample;
    sample.stamp = message->header.stamp.isZero() ? ros::Time::now()
                                                  : message->header.stamp;
    try {
      sample.odom_T_base = pose_to_eigen(message->pose.pose);
    } catch (const std::exception& error) {
      ROS_ERROR_THROTTLE(2.0, "Rejected odometry pose: %s", error.what());
      return;
    }
    sample.covariance = message->pose.covariance;

    std::lock_guard<std::mutex> lock(mutex_);
    odom_history_.push_back(sample);
    while (odom_history_.size() > 300 ||
           (!odom_history_.empty() &&
            (sample.stamp - odom_history_.front().stamp).toSec() > 5.0)) {
      odom_history_.pop_front();
    }
    latest_odom_ = sample;
    has_latest_odom_ = true;
    if (has_seed_ && !has_seed_map_T_odom_guess_) {
      seed_map_T_odom_guess_ =
          seed_map_T_base_ * sample.odom_T_base.inverse();
      has_seed_map_T_odom_guess_ = true;
      map_T_odom_ = seed_map_T_odom_guess_;
      has_map_T_odom_ = true;
    }
    if (has_map_T_odom_) publish_pose_tf_path_locked(sample);
  }

  bool nearest_odom_locked(const ros::Time& stamp, OdomSample* result) const {
    if (odom_history_.empty() || !result) return false;
    double best_skew = std::numeric_limits<double>::infinity();
    const OdomSample* best = nullptr;
    for (const OdomSample& sample : odom_history_) {
      const double skew = std::abs((sample.stamp - stamp).toSec());
      if (skew < best_skew) {
        best_skew = skew;
        best = &sample;
      }
    }
    if (!best || best_skew > rolling_config_.max_stamp_skew_s) return false;
    *result = *best;
    return true;
  }

  void cloud_callback(const sensor_msgs::PointCloud2ConstPtr& message) {
    const ros::Time stamp = message->header.stamp.isZero() ? ros::Time::now()
                                                           : message->header.stamp;
    Cloud::Ptr cloud(new Cloud);
    pcl::fromROSMsg(*message, *cloud);

    OdomSample odom;
    Cloud::Ptr submap;
    Eigen::Isometry3d predicted_map_T_base = Eigen::Isometry3d::Identity();
    bool initializing = false;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (!nearest_odom_locked(stamp, &odom)) {
        publish_diagnostic_locked("CLOUD_ODOMETRY_TIME_MISMATCH",
                                  RegistrationResult(), CorrectionDecision());
        return;
      }
      if (!rolling_submap_.add_sample(
              cloud, stamp.toSec(), odom.odom_T_base, odom.stamp.toSec(),
              message->header.frame_id)) {
        publish_diagnostic_locked("CLOUD_REJECTED",
                                  RegistrationResult(), CorrectionDecision());
        return;
      }
      if (!has_seed_ || !has_seed_map_T_odom_guess_ ||
          !alignment_controller_.auto_correction_enabled()) {
        return;
      }
      initializing =
          alignment_controller_.state() ==
          static_livox_localization::AlignmentState::VERIFYING;
      const bool first_verification = last_correction_stamp_s_ < 0.0;
      const double required_period =
          first_verification ? initialization_window_s_ : correction_period_s_;
      const double reference_stamp =
          first_verification ? initialization_start_stamp_s_
                             : last_correction_stamp_s_;
      if (correction_in_progress_ ||
          stamp.toSec() - reference_stamp < required_period) return;
      submap = rolling_submap_.build_in_base_frame(odom.odom_T_base);
      if (!submap ||
          static_cast<int>(submap->size()) < registration_config_.min_points) {
        publish_diagnostic_locked("INSUFFICIENT_ROLLING_SUBMAP",
                                  RegistrationResult(), CorrectionDecision());
        return;
      }
      predicted_map_T_base = map_T_odom_ * odom.odom_T_base;
      correction_in_progress_ = true;
      last_correction_stamp_s_ = stamp.toSec();
    }

    RegistrationResult registration = static_livox_localization::register_cloud(
        submap, map_, predicted_map_T_base, registration_config_);
    static_livox_localization::TrackingConfig decision_config = tracking_config_;
    if (initializing) {
      decision_config.max_prediction_translation_m =
          registration_config_.max_seed_translation;
      decision_config.max_prediction_rotation_rad =
          registration_config_.max_seed_rotation_rad;
    }
    const CorrectionDecision decision =
        static_livox_localization::evaluate_correction(
            registration, predicted_map_T_base, decision_config);

    std::lock_guard<std::mutex> lock(mutex_);
    correction_in_progress_ = false;
    if (!alignment_controller_.auto_correction_enabled()) return;
    if (decision.accepted) {
      const Eigen::Isometry3d candidate_map_T_odom =
          static_livox_localization::compute_map_T_odom(
              registration.map_T_base, odom.odom_T_base);
      if (initializing) {
        const ConsensusDecision consensus =
            alignment_controller_.observe_candidate(candidate_map_T_odom);
        if (consensus.ready) {
          map_T_odom_ = candidate_map_T_odom;
          if (state_machine_.state() ==
              TrackingState::WAITING_INITIALIZATION) {
            state_machine_.initialize(stamp.toSec());
          } else {
            state_machine_.observe(true, stamp.toSec());
          }
          publish_pose_tf_path_locked(odom);
        }
        publish_diagnostic_locked(consensus.reason, registration, decision);
      } else {
        map_T_odom_ = static_livox_localization::limit_map_T_odom_step(
            map_T_odom_, candidate_map_T_odom, tracking_config_);
        state_machine_.observe(true, stamp.toSec());
        publish_pose_tf_path_locked(odom);
        publish_diagnostic_locked(decision.reason, registration, decision);
      }
    } else {
      alignment_controller_.observe_rejection();
      if (alignment_controller_.state() ==
          static_livox_localization::AlignmentState::TRACKING &&
          state_machine_.observe(false, stamp.toSec()) ==
              TrackingState::LOST) {
        alignment_controller_.begin_reacquisition();
      }
      publish_diagnostic_locked(decision.reason, registration, decision);
    }
  }

  void publish_pose_tf_path_locked(const OdomSample& odom) {
    const Eigen::Isometry3d& odom_T_base = odom.odom_T_base;
    const Eigen::Isometry3d map_T_base = map_T_odom_ * odom_T_base;
    geometry_msgs::PoseWithCovarianceStamped pose_message;
    pose_message.header.stamp = odom.stamp;
    pose_message.header.frame_id = map_frame_;
    pose_message.pose.pose = eigen_to_pose(map_T_base);
    pose_message.pose.covariance = odom.covariance;
    pose_pub_.publish(pose_message);

    geometry_msgs::PoseStamped path_pose;
    path_pose.header = pose_message.header;
    path_pose.pose = pose_message.pose.pose;
    path_.header = pose_message.header;
    path_.poses.push_back(path_pose);
    if (path_max_poses_ > 0 &&
        static_cast<int>(path_.poses.size()) > path_max_poses_) {
      path_.poses.erase(path_.poses.begin(),
                        path_.poses.begin() +
                            (path_.poses.size() - path_max_poses_));
    }
    path_pub_.publish(path_);

    geometry_msgs::TransformStamped transform;
    transform.header.stamp = odom.stamp;
    transform.header.frame_id = map_frame_;
    transform.child_frame_id = odom_frame_;
    transform.transform = eigen_to_transform(map_T_odom_);
    tf_broadcaster_.sendTransform(transform);
  }

  void publish_diagnostic_locked(const std::string& reason,
                                 const RegistrationResult& registration,
                                 const CorrectionDecision& decision) {
    diagnostic_msgs::DiagnosticArray array;
    array.header.stamp = ros::Time::now();
    diagnostic_msgs::DiagnosticStatus status;
    status.name = "fast_lio_icp";
    status.hardware_id = map_id_;
    const TrackingState tracking_state = state_machine_.state();
    const static_livox_localization::AlignmentState alignment_state =
        alignment_controller_.state();
    if (alignment_state !=
        static_livox_localization::AlignmentState::TRACKING) {
      status.level = diagnostic_msgs::DiagnosticStatus::WARN;
      status.message =
          static_livox_localization::alignment_state_name(alignment_state);
    } else if (tracking_state == TrackingState::TRACKING) {
      status.level = diagnostic_msgs::DiagnosticStatus::OK;
      status.message =
          static_livox_localization::tracking_state_name(tracking_state);
    } else if (tracking_state == TrackingState::LOST) {
      status.level = diagnostic_msgs::DiagnosticStatus::ERROR;
      status.message =
          static_livox_localization::tracking_state_name(tracking_state);
    } else {
      status.level = diagnostic_msgs::DiagnosticStatus::WARN;
      status.message =
          static_livox_localization::tracking_state_name(tracking_state);
    }
    status.values.push_back(key_value("raw_state", status.message));
    status.values.push_back(key_value("auto_correction_enabled",
                                      alignment_controller_.auto_correction_enabled()
                                          ? "true" : "false"));
    status.values.push_back(key_value("consistent_candidate_count",
                                      std::to_string(
                                          alignment_controller_.consistent_count())));
    status.values.push_back(key_value("reason", reason));
    status.values.push_back(
        key_value("fitness", std::to_string(registration.fitness)));
    status.values.push_back(
        key_value("inlier_ratio", std::to_string(registration.inlier_ratio)));
    status.values.push_back(key_value(
        "prediction_translation_m",
        std::to_string(decision.prediction_translation_m)));
    status.values.push_back(key_value(
        "prediction_rotation_rad",
        std::to_string(decision.prediction_rotation_rad)));
    status.values.push_back(key_value(
        "source_points", std::to_string(registration.source_points)));
    status.values.push_back(key_value(
        "target_points", std::to_string(registration.target_points)));
    status.values.push_back(
        key_value("reset_count", std::to_string(reset_count_)));
    status.values.push_back(key_value("map_id", map_id_));
    status.values.push_back(key_value("map_sha256", map_sha256_));
    status.values.push_back(key_value("map_frame", map_frame_));
    status.values.push_back(key_value("odom_frame", odom_frame_));
    status.values.push_back(key_value("base_frame", base_frame_));
    array.status.push_back(status);
    diagnostics_pub_.publish(array);
  }

  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  ros::Publisher pose_pub_;
  ros::Publisher path_pub_;
  ros::Publisher diagnostics_pub_;
  ros::Subscriber seed_sub_;
  ros::Subscriber odom_sub_;
  ros::Subscriber cloud_sub_;
  ros::ServiceServer auto_correction_service_;
  tf2_ros::TransformBroadcaster tf_broadcaster_;

  Cloud::Ptr map_;
  static_livox_localization::RegistrationConfig registration_config_;
  static_livox_localization::TrackingConfig tracking_config_;
  static_livox_localization::RollingSubmapConfig rolling_config_;
  static_livox_localization::AlignmentConfig alignment_config_;
  static_livox_localization::RollingSubmap rolling_submap_;
  static_livox_localization::TrackingStateMachine state_machine_;
  static_livox_localization::AssistedAlignmentController alignment_controller_;

  std::mutex mutex_;
  std::deque<OdomSample> odom_history_;
  OdomSample latest_odom_;
  nav_msgs::Path path_;
  Eigen::Isometry3d seed_map_T_base_ = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d seed_map_T_odom_guess_ = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d map_T_odom_ = Eigen::Isometry3d::Identity();
  bool has_seed_ = false;
  bool has_latest_odom_ = false;
  bool has_seed_map_T_odom_guess_ = false;
  bool has_map_T_odom_ = false;
  bool correction_in_progress_ = false;
  bool auto_correction_on_start_ = false;
  int reset_count_ = 0;
  int path_max_poses_ = 2000;
  double initialization_window_s_ = 3.0;
  double correction_period_s_ = 1.0;
  double initialization_start_stamp_s_ = -1.0;
  double last_correction_stamp_s_ = -1.0;

  std::string map_path_;
  std::string map_sha256_;
  std::string map_id_;
  std::string map_frame_;
  std::string odom_frame_;
  std::string base_frame_;
  std::string cloud_topic_;
  std::string odom_topic_;
  std::string seed_topic_;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "moving_icp_localizer");
  try {
    MovingIcpLocalizer node;
    ros::AsyncSpinner spinner(2);
    spinner.start();
    ros::waitForShutdown();
  } catch (const std::exception& error) {
    ROS_FATAL("moving localization startup rejected: %s", error.what());
    return 2;
  }
  return 0;
}


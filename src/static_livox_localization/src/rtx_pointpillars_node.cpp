// ROS1 wrapper around NVIDIA CUDA-PointPillars.
//
// The CUDA/TensorRT implementation is not copied here. It is built from the
// pinned NVIDIA-AI-IOT/CUDA-PointPillars commit by
// tools/setup_rtx2060_pointpillars.sh, then linked as libpointpillar_core.so.
// Geometric obstacle detection remains authoritative; these detections add
// learned labels and must never erase LiDAR geometry.

#include <cuda_runtime.h>

#include <diagnostic_msgs/DiagnosticArray.h>
#include <diagnostic_msgs/DiagnosticStatus.h>
#include <diagnostic_msgs/KeyValue.h>
#include <geometry_msgs/Pose.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointField.h>
#include <std_msgs/String.h>
#include <tf2/LinearMath/Quaternion.h>
#include <vision_msgs/Detection3D.h>
#include <vision_msgs/Detection3DArray.h>
#include <vision_msgs/ObjectHypothesisWithPose.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <limits>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include "pointpillar/pointpillar.hpp"

namespace {

constexpr const char* kUpstreamCommit =
    "ce7e2bd694c90207435c8751d61cdb38d48a9f4c";

std::string jsonEscape(const std::string& value) {
  std::ostringstream out;
  for (const char c : value) {
    switch (c) {
      case '\\': out << "\\\\"; break;
      case '"': out << "\\\""; break;
      case '\n': out << "\\n"; break;
      case '\r': out << "\\r"; break;
      case '\t': out << "\\t"; break;
      default: out << c; break;
    }
  }
  return out.str();
}

void requireCuda(cudaError_t result, const char* operation) {
  if (result != cudaSuccess) {
    throw std::runtime_error(std::string(operation) + ": " +
                             cudaGetErrorString(result));
  }
}

const sensor_msgs::PointField* findField(
    const sensor_msgs::PointCloud2& message, const std::string& name) {
  for (const auto& field : message.fields) {
    if (field.name == name) return &field;
  }
  return nullptr;
}

bool finiteBox(const pointpillar::lidar::BoundingBox& box) {
  const float values[] = {box.x, box.y, box.z, box.w, box.l,
                          box.h, box.rt, box.score};
  for (const float value : values) {
    if (!std::isfinite(value)) return false;
  }
  return box.w > 0.0f && box.l > 0.0f && box.h > 0.0f;
}

void addValue(diagnostic_msgs::DiagnosticStatus* status,
              const std::string& key, const std::string& value) {
  diagnostic_msgs::KeyValue pair;
  pair.key = key;
  pair.value = value;
  status->values.push_back(pair);
}

}  // namespace

class RtxPointPillarsNode {
 public:
  RtxPointPillarsNode() : private_nh_("~") {
    private_nh_.param("gpu_device", gpu_device_, 0);
    private_nh_.param("model_path", model_path_, std::string());
    private_nh_.param("input_topic", input_topic_,
                      std::string("/cloud_registered_body"));
    private_nh_.param("detections_topic", detections_topic_,
                      std::string("/pointpillars/detections"));
    private_nh_.param("status_topic", status_topic_,
                      std::string("/pointpillars/status"));
    private_nh_.param("expected_frame", expected_frame_, std::string("body"));
    private_nh_.param("require_rtx2060", require_rtx2060_, true);
    private_nh_.param("max_cloud_age_s", max_cloud_age_s_, 0.50);
    private_nh_.param("max_inference_ms", max_inference_ms_, 90.0);
    private_nh_.param("minimum_points", minimum_points_, 800);
    private_nh_.param("max_points", max_points_, 300000);
    private_nh_.param("normalize_livox_intensity", normalize_intensity_, true);
    private_nh_.param("car_score_threshold", car_threshold_, 0.45);
    private_nh_.param("person_score_threshold", person_threshold_, 0.30);
    private_nh_.param("cyclist_score_threshold", cyclist_threshold_, 0.35);
    private_nh_.param("other_score_threshold", other_threshold_, 0.50);

    validateParameters();
    initializeGpu();
    initializeCore();

    detections_pub_ = nh_.advertise<vision_msgs::Detection3DArray>(
        detections_topic_, 1);
    status_pub_ = nh_.advertise<std_msgs::String>(status_topic_, 1, true);
    diagnostics_pub_ = nh_.advertise<diagnostic_msgs::DiagnosticArray>(
        "/diagnostics", 1);
    cloud_sub_ = nh_.subscribe(input_topic_, 1,
                               &RtxPointPillarsNode::onCloud, this,
                               ros::TransportHints().tcpNoDelay());

    points_.reserve(static_cast<std::size_t>(max_points_) * 4U);
    publishStatus("INITIALIZING", 0, 0, 0, 0.0, "waiting for first cloud");
    ROS_INFO("RTX PointPillars ready: GPU=%s compute=%d.%d model=%s input=%s",
             gpu_name_.c_str(), compute_major_, compute_minor_,
             model_path_.c_str(), input_topic_.c_str());
  }

  ~RtxPointPillarsNode() {
    if (stream_ != nullptr) cudaStreamDestroy(stream_);
  }

 private:
  void validateParameters() const {
    if (model_path_.empty()) {
      throw std::runtime_error("~model_path is required");
    }
    std::ifstream model(model_path_, std::ios::binary);
    if (!model.good()) {
      throw std::runtime_error("TensorRT engine not readable: " + model_path_);
    }
    if (max_points_ < 1000 || max_points_ > 300000) {
      throw std::runtime_error("~max_points must be within [1000, 300000]");
    }
    if (minimum_points_ < 1 || minimum_points_ >= max_points_) {
      throw std::runtime_error("~minimum_points must be positive and below max_points");
    }
    const double values[] = {max_cloud_age_s_, max_inference_ms_, car_threshold_,
                             person_threshold_, cyclist_threshold_, other_threshold_};
    for (const double value : values) {
      if (!std::isfinite(value)) {
        throw std::runtime_error("PointPillars parameters must be finite");
      }
    }
  }

  void initializeGpu() {
    int count = 0;
    requireCuda(cudaGetDeviceCount(&count), "cudaGetDeviceCount");
    if (count <= 0 || gpu_device_ < 0 || gpu_device_ >= count) {
      throw std::runtime_error("requested CUDA device is unavailable");
    }
    requireCuda(cudaSetDevice(gpu_device_), "cudaSetDevice");
    cudaDeviceProp property{};
    requireCuda(cudaGetDeviceProperties(&property, gpu_device_),
                "cudaGetDeviceProperties");
    gpu_name_ = property.name;
    compute_major_ = property.major;
    compute_minor_ = property.minor;
    total_memory_mb_ = static_cast<double>(property.totalGlobalMem) /
                       (1024.0 * 1024.0);
    if (compute_major_ < 7 ||
        (compute_major_ == 7 && compute_minor_ < 5)) {
      throw std::runtime_error(
          "CUDA-PointPillars requires compute capability 7.5 or newer");
    }
    if (require_rtx2060_ && gpu_name_.find("RTX 2060") == std::string::npos) {
      throw std::runtime_error(
          "expected an RTX 2060, found: " + gpu_name_ +
          " (set ~require_rtx2060:=false only after review)");
    }
    requireCuda(cudaStreamCreateWithFlags(&stream_, cudaStreamNonBlocking),
                "cudaStreamCreateWithFlags");
  }

  void initializeCore() {
    pointpillar::lidar::VoxelizationParameter voxelization;
    voxelization.min_range = nvtype::Float3(0.0f, -39.68f, -3.0f);
    voxelization.max_range = nvtype::Float3(69.12f, 39.68f, 1.0f);
    voxelization.voxel_size = nvtype::Float3(0.16f, 0.16f, 4.0f);
    voxelization.grid_size = voxelization.compute_grid_size(
        voxelization.max_range, voxelization.min_range,
        voxelization.voxel_size);
    voxelization.max_voxels = 40000;
    voxelization.max_points_per_voxel = 32;
    voxelization.max_points = max_points_;
    voxelization.num_feature = 4;

    pointpillar::lidar::PostProcessParameter post;
    post.min_range = voxelization.min_range;
    post.max_range = voxelization.max_range;
    post.feature_size = nvtype::Int2(voxelization.grid_size.x / 2,
                                     voxelization.grid_size.y / 2);

    pointpillar::lidar::CoreParameter parameters;
    parameters.voxelization = voxelization;
    parameters.lidar_model = model_path_;
    parameters.lidar_post = post;
    core_ = pointpillar::lidar::create_core(parameters);
    if (!core_) {
      throw std::runtime_error("NVIDIA CUDA-PointPillars core creation failed");
    }
    core_->set_timer(false);
  }

  float thresholdFor(int class_id) const {
    switch (class_id) {
      case 0: return static_cast<float>(car_threshold_);
      case 1: return static_cast<float>(person_threshold_);
      case 2: return static_cast<float>(cyclist_threshold_);
      default: return static_cast<float>(other_threshold_);
    }
  }

  bool decodeCloud(const sensor_msgs::PointCloud2& message,
                   std::size_t* finite_points) {
    points_.clear();
    *finite_points = 0;
    if (message.is_bigendian) return false;

    const auto* x_field = findField(message, "x");
    const auto* y_field = findField(message, "y");
    const auto* z_field = findField(message, "z");
    const auto* intensity_field = findField(message, "intensity");
    const auto valid_float = [&message](const sensor_msgs::PointField* field) {
      return field != nullptr &&
             field->datatype == sensor_msgs::PointField::FLOAT32 &&
             field->count == 1 && field->offset + sizeof(float) <=
                                    message.point_step;
    };
    if (!valid_float(x_field) || !valid_float(y_field) ||
        !valid_float(z_field)) {
      return false;
    }
    const bool has_intensity = valid_float(intensity_field);
    const std::size_t total = static_cast<std::size_t>(message.width) *
                              static_cast<std::size_t>(message.height);
    if (total == 0 || message.point_step == 0 || message.row_step == 0) {
      return true;
    }
    const std::size_t stride = std::max<std::size_t>(
        1U, (total + static_cast<std::size_t>(max_points_) - 1U) /
                static_cast<std::size_t>(max_points_));

    std::size_t linear_index = 0;
    for (std::uint32_t row = 0; row < message.height; ++row) {
      const std::size_t row_start = static_cast<std::size_t>(row) *
                                    message.row_step;
      if (row_start + message.row_step > message.data.size()) return false;
      for (std::uint32_t column = 0; column < message.width;
           ++column, ++linear_index) {
        if (linear_index % stride != 0) continue;
        const std::size_t offset = row_start +
            static_cast<std::size_t>(column) * message.point_step;
        if (offset + message.point_step > message.data.size()) return false;
        const std::uint8_t* point = message.data.data() + offset;
        float x = 0.0f, y = 0.0f, z = 0.0f, intensity = 0.0f;
        std::memcpy(&x, point + x_field->offset, sizeof(float));
        std::memcpy(&y, point + y_field->offset, sizeof(float));
        std::memcpy(&z, point + z_field->offset, sizeof(float));
        if (has_intensity) {
          std::memcpy(&intensity, point + intensity_field->offset,
                      sizeof(float));
        }
        if (!std::isfinite(x) || !std::isfinite(y) ||
            !std::isfinite(z) || !std::isfinite(intensity)) {
          continue;
        }
        ++(*finite_points);
        if (x < 0.0f || x >= 69.12f || y <= -39.68f || y >= 39.68f ||
            z <= -3.0f || z >= 1.0f) {
          continue;
        }
        if (normalize_intensity_) {
          intensity = std::max(0.0f, std::min(255.0f, intensity)) / 255.0f;
        }
        points_.push_back(x);
        points_.push_back(y);
        points_.push_back(z);
        points_.push_back(intensity);
        if (points_.size() / 4U >= static_cast<std::size_t>(max_points_)) {
          return true;
        }
      }
    }
    return true;
  }

  void onCloud(const sensor_msgs::PointCloud2ConstPtr& message) {
    const ros::Time now = ros::Time::now();
    if (!expected_frame_.empty() && message->header.frame_id != expected_frame_) {
      publishStatus("FRAME_MISMATCH", message->width * message->height,
                    0, 0, 0.0,
                    "expected " + expected_frame_ + ", got " +
                    message->header.frame_id);
      return;
    }
    if (!message->header.stamp.isZero()) {
      const double age = (now - message->header.stamp).toSec();
      if (!std::isfinite(age) || age < -0.05 || age > max_cloud_age_s_) {
        publishStatus("CLOUD_STALE", message->width * message->height,
                      0, 0, 0.0, "cloud age outside the accepted window");
        return;
      }
    }

    std::size_t finite_points = 0;
    if (!decodeCloud(*message, &finite_points)) {
      publishStatus("BAD_CLOUD_LAYOUT", message->width * message->height,
                    finite_points, 0, 0.0,
                    "x/y/z must be little-endian FLOAT32 fields");
      return;
    }
    const std::size_t point_count = points_.size() / 4U;
    if (point_count < static_cast<std::size_t>(minimum_points_)) {
      publishStatus("TOO_FEW_POINTS", message->width * message->height,
                    finite_points, point_count, 0.0,
                    "not enough in-range points for learned inference");
      return;
    }

    std::vector<pointpillar::lidar::BoundingBox> boxes;
    double inference_ms = 0.0;
    try {
      const auto begin = std::chrono::steady_clock::now();
      boxes = core_->forward(points_.data(), static_cast<int>(point_count),
                             stream_);
      requireCuda(cudaStreamSynchronize(stream_), "cudaStreamSynchronize");
      const auto end = std::chrono::steady_clock::now();
      inference_ms = std::chrono::duration<double, std::milli>(end - begin).count();
    } catch (const std::exception& error) {
      ++failed_frames_;
      publishStatus("INFERENCE_ERROR", message->width * message->height,
                    finite_points, point_count, 0.0, error.what());
      return;
    }

    vision_msgs::Detection3DArray output;
    output.header = message->header;
    for (const auto& box : boxes) {
      if (!finiteBox(box) || box.score < thresholdFor(box.id)) continue;
      vision_msgs::Detection3D detection;
      detection.header = message->header;
      detection.bbox.center.position.x = box.x;
      detection.bbox.center.position.y = box.y;
      detection.bbox.center.position.z = box.z;
      tf2::Quaternion orientation;
      orientation.setRPY(0.0, 0.0, box.rt);
      orientation.normalize();
      detection.bbox.center.orientation.x = orientation.x();
      detection.bbox.center.orientation.y = orientation.y();
      detection.bbox.center.orientation.z = orientation.z();
      detection.bbox.center.orientation.w = orientation.w();
      detection.bbox.size.x = std::max(0.05f, box.w);
      detection.bbox.size.y = std::max(0.05f, box.l);
      detection.bbox.size.z = std::max(0.05f, box.h);

      vision_msgs::ObjectHypothesisWithPose hypothesis;
      hypothesis.id = box.id;
      hypothesis.score = box.score;
      hypothesis.pose.pose = detection.bbox.center;
      detection.results.push_back(hypothesis);
      output.detections.push_back(detection);
    }
    detections_pub_.publish(output);
    ++processed_frames_;
    const std::string state =
        inference_ms <= max_inference_ms_ ? "OK" : "SLOW";
    publishStatus(state, message->width * message->height, finite_points,
                  point_count, output.detections.size(), inference_ms,
                  state == "OK" ? "GPU inference completed" :
                                  "inference exceeded the latency gate");
  }

  void publishStatus(const std::string& state, std::size_t input_points,
                     std::size_t finite_points, std::size_t used_points,
                     double inference_ms, const std::string& detail) {
    publishStatus(state, input_points, finite_points, used_points, 0,
                  inference_ms, detail);
  }

  void publishStatus(const std::string& state, std::size_t input_points,
                     std::size_t finite_points, std::size_t used_points,
                     std::size_t detections, double inference_ms,
                     const std::string& detail) {
    std::size_t free_bytes = 0, total_bytes = 0;
    cudaMemGetInfo(&free_bytes, &total_bytes);
    const double now = ros::Time::now().toSec();
    std::ostringstream json;
    json << std::fixed << std::setprecision(3)
         << "{\"stamp\":" << now
         << ",\"status\":\"" << jsonEscape(state) << "\""
         << ",\"gpu_active\":true"
         << ",\"device_index\":" << gpu_device_
         << ",\"device_name\":\"" << jsonEscape(gpu_name_) << "\""
         << ",\"compute_capability\":\"" << compute_major_ << "."
         << compute_minor_ << "\""
         << ",\"cuda_memory_total_mb\":" << total_memory_mb_
         << ",\"cuda_memory_free_mb\":"
         << static_cast<double>(free_bytes) / (1024.0 * 1024.0)
         << ",\"model_path\":\"" << jsonEscape(model_path_) << "\""
         << ",\"upstream_commit\":\"" << kUpstreamCommit << "\""
         << ",\"input_points\":" << input_points
         << ",\"finite_points\":" << finite_points
         << ",\"used_points\":" << used_points
         << ",\"detections\":" << detections
         << ",\"inference_ms\":" << inference_ms
         << ",\"processed_frames\":" << processed_frames_
         << ",\"failed_frames\":" << failed_frames_
         << ",\"detail\":\"" << jsonEscape(detail) << "\"}";
    std_msgs::String message;
    message.data = json.str();
    status_pub_.publish(message);

    diagnostic_msgs::DiagnosticArray array;
    array.header.stamp = ros::Time::now();
    diagnostic_msgs::DiagnosticStatus diagnostic;
    diagnostic.name = "rtx_pointpillars";
    diagnostic.hardware_id = gpu_name_;
    if (state == "OK") {
      diagnostic.level = diagnostic_msgs::DiagnosticStatus::OK;
    } else if (state == "SLOW" || state == "INITIALIZING") {
      diagnostic.level = diagnostic_msgs::DiagnosticStatus::WARN;
    } else {
      diagnostic.level = diagnostic_msgs::DiagnosticStatus::ERROR;
    }
    diagnostic.message = state;
    addValue(&diagnostic, "gpu", gpu_name_);
    addValue(&diagnostic, "upstream_commit", kUpstreamCommit);
    addValue(&diagnostic, "model", model_path_);
    addValue(&diagnostic, "inference_ms", std::to_string(inference_ms));
    addValue(&diagnostic, "used_points", std::to_string(used_points));
    addValue(&diagnostic, "detections", std::to_string(detections));
    array.status.push_back(diagnostic);
    diagnostics_pub_.publish(array);
  }

  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  ros::Subscriber cloud_sub_;
  ros::Publisher detections_pub_;
  ros::Publisher status_pub_;
  ros::Publisher diagnostics_pub_;

  int gpu_device_ = 0;
  int compute_major_ = 0;
  int compute_minor_ = 0;
  int minimum_points_ = 800;
  int max_points_ = 300000;
  bool require_rtx2060_ = true;
  bool normalize_intensity_ = true;
  double max_cloud_age_s_ = 0.5;
  double max_inference_ms_ = 90.0;
  double car_threshold_ = 0.45;
  double person_threshold_ = 0.30;
  double cyclist_threshold_ = 0.35;
  double other_threshold_ = 0.50;
  double total_memory_mb_ = 0.0;
  std::string model_path_;
  std::string input_topic_;
  std::string detections_topic_;
  std::string status_topic_;
  std::string expected_frame_;
  std::string gpu_name_;
  cudaStream_t stream_ = nullptr;
  std::shared_ptr<pointpillar::lidar::Core> core_;
  std::vector<float> points_;
  std::uint64_t processed_frames_ = 0;
  std::uint64_t failed_frames_ = 0;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "rtx_pointpillars");
  try {
    RtxPointPillarsNode node;
    ros::spin();
  } catch (const std::exception& error) {
    ROS_FATAL("RTX PointPillars failed: %s", error.what());
    return 1;
  }
  return 0;
}

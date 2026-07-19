#include <exception>
#include <string>

#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include "static_livox_localization/bounded_cloud_preview.hpp"

class BoundedCloudPreviewNode {
 public:
  using Preview = static_livox_localization::BoundedCloudPreview;

  BoundedCloudPreviewNode()
      : private_nh_("~"), preview_(load_config()) {
    private_nh_.param<std::string>("preview_input_topic", input_topic_,
                                   "/cloud_registered_body");
    private_nh_.param<std::string>("preview_output_topic", output_topic_,
                                   "/fast_lio_icp/live_preview");
    publisher_ = nh_.advertise<sensor_msgs::PointCloud2>(output_topic_, 1);
    subscriber_ = nh_.subscribe(input_topic_, 1,
                                &BoundedCloudPreviewNode::cloud_callback, this);
    ROS_INFO("Bounded RViz preview: %s -> %s", input_topic_.c_str(),
             output_topic_.c_str());
  }

 private:
  static_livox_localization::PreviewConfig load_config() {
    static_livox_localization::PreviewConfig config;
    private_nh_.param("preview_voxel_resolution", config.voxel_resolution, 0.30);
    private_nh_.param("preview_max_rate_hz", config.max_rate_hz, 5.0);
    return config;
  }

  void cloud_callback(const sensor_msgs::PointCloud2ConstPtr& message) {
    const ros::Time stamp =
        message->header.stamp.isZero() ? ros::Time::now() : message->header.stamp;
    if (!preview_.should_publish(stamp.toSec())) return;

    try {
      Preview::Cloud::Ptr input(new Preview::Cloud);
      pcl::fromROSMsg(*message, *input);
      const Preview::Cloud::Ptr output = preview_.downsample(input);
      if (!output || output->empty()) return;

      sensor_msgs::PointCloud2 output_message;
      pcl::toROSMsg(*output, output_message);
      output_message.header = message->header;
      publisher_.publish(output_message);
    } catch (const std::exception& error) {
      ROS_WARN_THROTTLE(5.0, "Bounded preview dropped cloud: %s", error.what());
    }
  }

  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  ros::Publisher publisher_;
  ros::Subscriber subscriber_;
  Preview preview_;
  std::string input_topic_;
  std::string output_topic_;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "bounded_cloud_preview");
  BoundedCloudPreviewNode node;
  ros::spin();
  return 0;
}

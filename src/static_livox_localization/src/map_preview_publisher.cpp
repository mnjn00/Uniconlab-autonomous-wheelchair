#include <openssl/sha.h>

#include <fstream>
#include <iomanip>
#include <sstream>
#include <stdexcept>
#include <string>

#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

namespace {

std::string sha256_file(const std::string& path) {
  std::ifstream input(path, std::ios::binary);
  if (!input) throw std::runtime_error("cannot open map: " + path);
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

}  // namespace

int main(int argc, char** argv) {
  ros::init(argc, argv, "map_preview_publisher");
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");
  std::string map_path;
  std::string map_sha256;
  std::string map_frame = "map";
  std::string topic = "/fast_lio_icp/map_preview";
  double map_preview_voxel_resolution = 0.75;
  private_nh.param<std::string>("map_path", map_path, "");
  private_nh.param<std::string>("map_sha256", map_sha256, "");
  private_nh.param<std::string>("map_frame", map_frame, "map");
  private_nh.param<std::string>("map_preview_topic", topic, topic);
  private_nh.param("map_preview_voxel_resolution",
                   map_preview_voxel_resolution, 0.75);

  try {
    if (map_path.empty() || map_sha256.size() != 64 ||
        sha256_file(map_path) != map_sha256) {
      throw std::runtime_error("map path or SHA-256 rejected");
    }
    pcl::PointCloud<pcl::PointXYZI>::Ptr map(
        new pcl::PointCloud<pcl::PointXYZI>);
    if (pcl::io::loadPCDFile(map_path, *map) != 0 || map->empty()) {
      throw std::runtime_error("failed to load map preview source");
    }
    pcl::PointCloud<pcl::PointXYZI> preview;
    pcl::VoxelGrid<pcl::PointXYZI> voxel;
    voxel.setLeafSize(map_preview_voxel_resolution,
                      map_preview_voxel_resolution,
                      map_preview_voxel_resolution);
    voxel.setInputCloud(map);
    voxel.filter(preview);

    sensor_msgs::PointCloud2 message;
    pcl::toROSMsg(preview, message);
    message.header.frame_id = map_frame;
    message.header.stamp = ros::Time::now();
    ros::Publisher publisher =
        nh.advertise<sensor_msgs::PointCloud2>(topic, 1, true);
    publisher.publish(message);
    ROS_INFO("Published latched map preview with %zu of %zu points",
             preview.size(), map->size());
    ros::spin();
  } catch (const std::exception& error) {
    ROS_FATAL("map preview startup rejected: %s", error.what());
    return 2;
  }
  return 0;
}


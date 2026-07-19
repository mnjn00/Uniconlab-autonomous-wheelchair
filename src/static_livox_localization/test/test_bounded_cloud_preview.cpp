#include <gtest/gtest.h>

#include "static_livox_localization/bounded_cloud_preview.hpp"

namespace static_livox_localization {

TEST(BoundedCloudPreview, LimitsPublishRateToFiveHertz) {
  PreviewConfig config;
  config.max_rate_hz = 5.0;
  BoundedCloudPreview preview(config);

  EXPECT_TRUE(preview.should_publish(10.0));
  EXPECT_FALSE(preview.should_publish(10.1));
  EXPECT_TRUE(preview.should_publish(10.2));
}

TEST(BoundedCloudPreview, AcceptsClockResetWithoutLongSilence) {
  PreviewConfig config;
  config.max_rate_hz = 5.0;
  BoundedCloudPreview preview(config);
  ASSERT_TRUE(preview.should_publish(10.0));

  EXPECT_TRUE(preview.should_publish(1.0));
}

TEST(BoundedCloudPreview, DownsamplesDenseCloudAndPreservesFrameShape) {
  PreviewConfig config;
  config.voxel_resolution = 0.30;
  BoundedCloudPreview preview(config);
  BoundedCloudPreview::Cloud::Ptr cloud(new BoundedCloudPreview::Cloud);
  for (int i = 0; i < 100; ++i) {
    pcl::PointXYZI point;
    point.x = 0.001F * static_cast<float>(i);
    point.y = 0.0F;
    point.z = 0.0F;
    point.intensity = static_cast<float>(i);
    cloud->push_back(point);
  }

  const BoundedCloudPreview::Cloud::Ptr output = preview.downsample(cloud);

  ASSERT_TRUE(output);
  EXPECT_FALSE(output->empty());
  EXPECT_LT(output->size(), cloud->size());
}

TEST(BoundedCloudPreview, EmptyInputProducesEmptyOutput) {
  BoundedCloudPreview preview(PreviewConfig{});
  BoundedCloudPreview::Cloud::Ptr cloud(new BoundedCloudPreview::Cloud);

  const BoundedCloudPreview::Cloud::Ptr output = preview.downsample(cloud);

  ASSERT_TRUE(output);
  EXPECT_TRUE(output->empty());
}

}  // namespace static_livox_localization

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

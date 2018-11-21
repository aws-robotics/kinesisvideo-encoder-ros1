/*
 *  Copyright 2018 Amazon.com, Inc. or its affiliates. All Rights Reserved.
 *
 *  This library is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU Lesser General Public
 *  License as published by the Free Software Foundation; either
 *  version 2.1 of the License, or (at your option) any later version.
 *
 *  This library is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 *  Lesser General Public License for more details.
 *
 *  You should have received a copy of the GNU Lesser General Public
 *  License along with this library; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */

#include <aws_ros1_common/sdk_utils/ros1_node_parameter_reader.h>
#include <gtest/gtest.h>
#include <h264_encoder_core/h264_encoder.h>
#include <h264_encoder_core/h264_encoder_node_config.h>
#include <image_transport/image_transport.h>
#include <kinesis_video_msgs/KinesisImageMetadata.h>
#include <kinesis_video_msgs/KinesisVideoFrame.h>
#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>

using namespace Aws::Utils::Encoding;
using namespace Aws::Utils::Logging;

namespace Aws {
namespace Kinesis {

void InitializeEncoder(const sensor_msgs::ImageConstPtr & msg,
                       std::unique_ptr<H264Encoder> & encoder,
                       const Aws::Client::ParameterReaderInterface & param_reader);

void ImageCallback(const sensor_msgs::ImageConstPtr & msg, const H264Encoder * encoder,
                   uint64_t & frame_num, kinesis_video_msgs::KinesisImageMetadata & metadata,
                   ros::Publisher & pub);

}  // namespace Kinesis
}  // namespace Aws

constexpr static char kDefaultTopicName[] = "/video/encoded";

class H264EncoderNodeSuite : public ::testing::Test
{
public:
  constexpr static int kDefaultWidth = 410;
  constexpr static int kDefaultHeight = 308;
  constexpr static const std::string & kDefaultEncoding = sensor_msgs::image_encodings::RGB8;
  constexpr static int kBytesPerPixel = 3;  // 3 color channels (red, green, blue)

  H264EncoderNodeSuite() {}

protected:
  void SetUp() override
  {
    ros::Time::init();

    msg = boost::make_shared<sensor_msgs::Image>();
    msg->header.seq = 0;
    msg->header.frame_id = "";
    msg->header.stamp = ros::Time::now();
    msg->height = kDefaultHeight;
    msg->width = kDefaultWidth;
    msg->encoding = kDefaultEncoding;
    msg->step = kBytesPerPixel * kDefaultWidth;
  }

  void KinesisVideoCallback(const kinesis_video_msgs::KinesisVideoFrame::ConstPtr & frame)
  {
    if (frame->index > 0) {
      EXPECT_GT(frame->index, prev_frame.index);
      EXPECT_EQ(frame->duration, prev_frame.duration);
      EXPECT_EQ((frame->decoding_ts - prev_frame.decoding_ts) % frame->duration, 0);
      EXPECT_EQ((frame->presentation_ts - prev_frame.presentation_ts) % frame->duration, 0);
      EXPECT_GT(frame->presentation_ts, prev_frame.presentation_ts);
    }

    prev_frame = *frame;
    if (0 == prev_frame.index) {
      fwrite(prev_frame.codec_private_data.data(), 1, prev_frame.codec_private_data.size(),
             debug_file);
    }
    fwrite(prev_frame.frame_data.data(), 1, prev_frame.frame_data.size(), debug_file);
  }

  sensor_msgs::ImagePtr msg;
  std::unique_ptr<H264Encoder> encoder;
  Aws::Client::Ros1NodeParameterReader param_reader;

  kinesis_video_msgs::KinesisVideoFrame prev_frame;
  FILE * debug_file;
};

/**
 * Tests the callback of the H264 Encoder Node that performs the initialization
 */
TEST_F(H264EncoderNodeSuite, EncoderInit)
{
  Aws::Kinesis::InitializeEncoder(msg, encoder, param_reader);
  EXPECT_NE(encoder, nullptr);
}

static void RainbowColor(const float h, uint8_t & r_out, uint8_t & g_out, uint8_t & b_out)
{
  int i = 6.0f * h;
  float f = 6.0f * h - i;
  float t = f;
  float q = 1.0f - f;

  float r, g, b;
  switch (i % 6) {
    case 0:
      r = 1.0f;
      g = t;
      b = 0.0f;
      break;
    case 1:
      r = q;
      g = 1.0f;
      b = 0.0f;
      break;
    case 2:
      r = 0.0f;
      g = 1.0f;
      b = t;
      break;
    case 3:
      r = 0.0f;
      g = q;
      b = 1.0f;
      break;
    case 4:
      r = t;
      g = 0.0f;
      b = 1.0f;
      break;
    case 5:
      r = 1.0f;
      g = 0.0f;
      b = q;
      break;
  }

  r_out = std::lround(255.0f * r);
  g_out = std::lround(255.0f * g);
  b_out = std::lround(255.0f * b);
}

/**
 * Tests the callback of the H264 Encoder Node that performs the encoding
 */
TEST_F(H264EncoderNodeSuite, EncoderCallback)
{
  Aws::Kinesis::InitializeEncoder(msg, encoder, param_reader);
  EXPECT_NE(encoder, nullptr);

  ros::NodeHandle pub_node;
  ros::Publisher pub =
    pub_node.advertise<kinesis_video_msgs::KinesisVideoFrame>(kDefaultTopicName, 100);

  ros::NodeHandle sub_node;
  boost::function<void(const kinesis_video_msgs::KinesisVideoFrame::ConstPtr &)> callback;
  callback = [this](const kinesis_video_msgs::KinesisVideoFrame::ConstPtr & frame) -> void {
    this->KinesisVideoCallback(frame);
  };
  ros::Subscriber sub = sub_node.subscribe(kDefaultTopicName, 100, callback);

  msg->data.resize(kBytesPerPixel * kDefaultWidth * kDefaultHeight);
  debug_file = fopen("frames.bin", "wb");

  // let's encode 30 frames
  constexpr int kNumTestFrames = 30;
  uint64_t prev_frame_index = 0, frame_index = 0;
  kinesis_video_msgs::KinesisImageMetadata metadata;
  for (int i = 0; i < kNumTestFrames; ++i) {
    ++msg->header.seq;
    msg->header.stamp = ros::Time::now();

    // prepare a dummy image
    int shift = static_cast<float>(i) / (kNumTestFrames - 1) * kDefaultWidth;
    for (int y = 0; y < kDefaultHeight; ++y) {
      for (int x = 0; x < kDefaultWidth; ++x) {
        uint8_t r, g, b;
        RainbowColor(static_cast<float>((x + shift) % kDefaultWidth) / kDefaultWidth, r, g, b);
        msg->data[kBytesPerPixel * y * kDefaultWidth + kBytesPerPixel * x + 0] = r;
        msg->data[kBytesPerPixel * y * kDefaultWidth + kBytesPerPixel * x + 1] = g;
        msg->data[kBytesPerPixel * y * kDefaultWidth + kBytesPerPixel * x + 2] = b;
      }
    }

    Aws::Kinesis::ImageCallback(msg, encoder.get(), frame_index, metadata, pub);
    ros::spinOnce();

    EXPECT_GE(frame_index, prev_frame_index);
    prev_frame_index = frame_index;
  }

  fclose(debug_file);
  // you can dump the debug frames by executing: ffmpeg -i frames.bin -frames:v 10 -f image2
  // frame%03d.png
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "test_h264_video_encoder");
  return RUN_ALL_TESTS();
}

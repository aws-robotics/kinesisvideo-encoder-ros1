# h264_video_encoder


## Overview
This package provides a ROS Node that will encode a stream of images into an H264 video stream.

**Keywords**: ROS, AWS, Kinesis

### License
The source code is released under [LGPL 2.1]. However, this package uses `h264_encoder_core` which incorporates several different encoding components which may further restrict the license. By default, x264 is used for software encoding, thereby applying GPL to all of h264_video_encoder.

**Author**: AWS RoboMaker<br/>
**Affiliation**: [Amazon Web Services (AWS)]<br/>
**Maintainer**: AWS RoboMaker, ros-contributions@amazon.com

### Supported ROS Distributions
- Kinetic
- Lunar
- Melodic


## Installation

### Building from Source
Create a ROS workspace and a source directory

    mkdir -p ~/ros-workspace/src
    
To build from source, clone the latest version from master branch and compile the package.

- Clone the package into the source directory

        cd ~/ros-workspace/src
        git clone https://github.com/aws/aws-ros-utils-common.git
        git clone https://github.com/aws/aws-ros-utils-ros1.git
        git clone https://github.com/aws/aws-ros-kinesisvideo-encoder-common.git
        git clone https://github.com/aws/aws-ros-kinesisvideo-encoder-ros1.git

- Install dependencies


        cd ~/ros-workspace && sudo apt-get update
        rosdep install --from-paths src --ignore-src -r -y

- Build the packages

        cd ~/ros-workspace && colcon build

- Configure ROS library Path

        source ~/ros-workspace/install/setup.bash

- Build and run the unit tests

        colcon build --packages-select h264_video_encoder --cmake-target tests
        colcon test --packages-select h264_video_encoder h264_encoder core && colcon test-results --all


## Launch Files
A launch file called `h264_video_encoder.launch` is included in this package. The launch file uses the following arguments:

| Arg Name | Description |
| --------- | ------------ |
| node_name | (optional) The name the H264 encoder node should be launched with. If not provided, the node name will default to `h264_video_encoder` |
| config_file | (optional) A path to a rosparam config file. |

An example launch file called `sample_application.launch` is included in this project that gives an example of how you can include this node in your project and provide it with arguments.


## Usage

### Running the node
To launch the H264 encoder node, you can run the following command:

    roslaunch h264_video_encoder sample_application.launch


## Configuration File and Parameters
An example configuration file called `sample_configuration.yaml` is provided for running the H264 encoder node on a Raspberry Pi based system.
When the parameters are absent in the ROS parameter server, default values are used, thus all parameters are optional. See table below for details.

| Parameter Name | Description | Type |
| ------------- | -----------------------------------------------------------| ------------- |
| queue_size | (optional) The maximum number of incoming and outgoing messages to be queued towards the subscribed and publishing topics. | integer |
| output_width | (optional) The desired width (in pixels) of each frame in the encoded video output. | integer |
| output_height | (optional) The desired height (in pixels) of each frame in the encoded video output. | integer |
| fps_numerator | (optional) The desired frames per second (the numerator portion when expressing FPS as a rational number) for the encoded video output. | integer |
| fps_denominator | (optional) The desired frames per second (the denominator portion when expressing FPS as a rational number) for the encoded video output. | integer |
| bitrate | (optional) The desired bitrate (in bits per second) of the encoded video output. | integer |


## Node Details

#### Published Topics
| Topic Name | Message Type | Description |
| ---------- | ------------ | ----------- |
| *Configurable* (default="video/encoded") | kinesis_video_msgs/KinesisVideoFrame | The node will publish to a topic of a given name. Each message being published contains a chunk of the video stream, usually per video frame. |

#### Subscribed Topics
| Topic Name | Message Type | Description |
| ---------- | ------------ | ----------- |
| *Configurable* (default="/raspicam_node/image") | sensor_msgs/Image | The node will subscribe to a topic of a given name. The data is expected to be a stream of images from a source (such as a Raspberry Pi camera). |


## Bugs & Feature Requests
Please contact the team directly if you would like to request a feature.

Please report bugs in [Issue Tracker].


[Amazon Web Services (AWS)]: https://aws.amazon.com/
[LGPL 2.1]: http://www.gnu.org/licenses/old-licenses/lgpl-2.1.html
[Issue Tracker]: TODO
[ROS]: http://www.ros.org

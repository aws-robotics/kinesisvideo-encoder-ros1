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
- Melodic

### Build status
* Travis CI:
    * "master" branch [![Build Status](https://travis-ci.org/aws-robotics/kinesisvideo-encoder-ros1.svg?branch=master)](https://travis-ci.org/aws-robotics/kinesisvideo-encoder-ros1/branches)
    * "release-latest" branch [![Build Status](https://travis-ci.org/aws-robotics/kinesisvideo-encoder-ros1.svg?branch=release-latest)](https://travis-ci.org/aws-robotics/kinesisvideo-encoder-ros1/branches)
* ROS build farm:
    * ROS Kinetic @ u16.04 Xenial [![Build Status](http://build.ros.org/job/Kbin_uX64__h264_video_encoder__ubuntu_xenial_amd64__binary/badge/icon)](http://build.ros.org/job/Kbin_uX64__h264_video_encoder__ubuntu_xenial_amd64__binary)
    * ROS Melodic @ u18.04 Bionic [![Build Status](http://build.ros.org/job/Mbin_uB64__h264_video_encoder__ubuntu_bionic_amd64__binary/badge/icon)](http://build.ros.org/job/Mbin_uB64__h264_video_encoder__ubuntu_bionic_amd64__binary)


## Installation

### Binaries
On Ubuntu you can install the latest version of this package using the following command

        sudo apt-get update
        sudo apt-get install -y ros-$ROS_DISTRO-h264-video-encoder
        
        
### Building from Source

To build from source you'll need to create a new workspace, clone and checkout the latest release branch of this repository, install all the dependencies, and compile. If you need the latest development features you can clone from the `master` branch instead of the latest release branch. While we guarantee the release branches are stable, __the `master` should be considered to have an unstable build__ due to ongoing development. 

- Create a ROS workspace and a source directory

        mkdir -p ~/ros-workspace/src

- Clone the package into the source directory . 

    _Note: Replace __`{MAJOR.VERSION}`__ below with the latest major version number to get the latest release branch._

        cd ~/ros-workspace/src
        git clone https://github.com/aws-robotics/kinesisvideo-encoder-ros1.git -b release-v{MAJOR.VERSION}

- Install dependencies

        cd ~/ros-workspace 
        sudo apt-get update && rosdep update
        rosdep install --from-paths src --ignore-src -r -y
        
_Note: If building the master branch instead of a release branch you may need to also checkout and build the master branches of the packages this package depends on._

- Build the packages

        cd ~/ros-workspace && colcon build

- Configure ROS library Path

        source ~/ros-workspace/install/setup.bash


- Build and run the unit tests

        colcon build --packages-select h264_video_encoder --cmake-target tests
        colcon test --packages-select h264_video_encoder h264_encoder core && colcon test-results --all


### Building on Cloud9 - Cross Compilation

- In RoboMaker's Cloud9, start with an empty workspace and in the Cloud9 console:

       # build docker image
       cd /opt/robomaker/cross-compilation-dockerfile/
       sudo bin/build_image.bash  # this step will take a while

       # create workspace
       mkdir -p ~/environment/robot_ws/src
       cd ~/environment/robot_ws/src
       git clone https://github.com/aws-robotics/kinesisvideo-encoder-common.git
       git clone https://github.com/aws-robotics/kinesisvideo-encoder-ros1.git

       # run docker image
       cd ..
       sudo docker run -v $(pwd):/ws -it ros-cross-compile:armhf

- Now you're inside the cross-compilation docker container

       # build the workspace
       cd ws
       apt update
       rosdep install --from-paths src --ignore-src -r -y  # this step will take a while
       colcon build --build-base armhf_build --install-base armhf_install
       colcon bundle --build-base armhf_build --install-base armhf_install --bundle-base armhf_bundle --apt-sources-list /opt/cross/apt-sources.yaml   # this step will take a while
       exit

- Now you're oustide the cross-compilation docker container

       # for more on copying s3 buckets see: https://docs.aws.amazon.com/cli/latest/reference/s3/cp.html
       aws s3 cp armhf_bundle/output.tar.gz s3://<bucket_name_in_your_robomaker_account>/h264_video_encoder.armhf.tar

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
[Issue Tracker]: https://github.com/aws-robotics/kinesisvideo-encoder-ros1/issues
[ROS]: http://www.ros.org

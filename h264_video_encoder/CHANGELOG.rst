^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package h264_video_encoder
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------
* increment patch version (`#31 <https://github.com/aws-robotics/kinesisvideo-encoder-ros1/issues/31>`_)
  Signed-off-by: Miaofei <miaofei@amazon.com>
* Use standard CMake macros for adding gtest/gmock tests (`#25 <https://github.com/aws-robotics/kinesisvideo-encoder-ros1/issues/25>`_)
  * modify h264_video_encoder to use add_rostest_gmock()
  Signed-off-by: Miaofei <miaofei@amazon.com>
  * update travis.yml to be compatible with specifying multiple package names
  Signed-off-by: Miaofei <miaofei@amazon.com>
* Update package.xml for 1.1.2 release
  Signed-off-by: Ryan Newell <ryanewel@amazon.com>
* fix issue with uninitialized frame number (`#19 <https://github.com/aws-robotics/kinesisvideo-encoder-ros1/issues/19>`_)
  Signed-off-by: Miaofei <miaofei@amazon.com>
* Add gtest and gmock as test dependencies
* Update package.xml
* Merge pull request `#8 <https://github.com/aws-robotics/kinesisvideo-encoder-ros1/issues/8>`_ from ryanewel/master
  increases unit test code coverage
* increases unit test code coverage
* Contributors: AAlon, M. M, Ross Desmond, Ryan Newell, ryanewel

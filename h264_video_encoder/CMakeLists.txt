cmake_minimum_required(VERSION 2.8)
project(h264_video_encoder)


add_compile_options(-std=c++11)

find_package(aws_common REQUIRED)

find_package(catkin REQUIRED COMPONENTS
    h264_encoder_core
    aws_ros1_common
    image_transport
    message_generation
    sensor_msgs
    kinesis_video_msgs)

catkin_package(CATKIN_DEPENDS
    aws_ros1_common
    image_transport
    message_runtime)


#############
## Compile ##
#############

set(h264_video_encoder_SRCS
    src/h264_video_encoder.cpp
)
set(h264_video_encoder_INCS
    include
    ${aws_common_INCLUDE_DIRS}
    ${catkin_INCLUDE_DIRS}
)
set(h264_video_encoder_LIBS
    ${catkin_LIBRARIES}
    aws_common
)

# add the publisher example
add_executable(${PROJECT_NAME} src/main.cpp ${h264_video_encoder_SRCS})
add_library(${PROJECT_NAME}_lib STATIC ${h264_video_encoder_SRCS})

target_include_directories(${PROJECT_NAME} PRIVATE ${h264_video_encoder_INCS})
target_include_directories(${PROJECT_NAME}_lib PUBLIC ${h264_video_encoder_INCS})

target_link_libraries(${PROJECT_NAME} ${h264_video_encoder_LIBS})
target_link_libraries(${PROJECT_NAME}_lib ${h264_video_encoder_LIBS})

add_dependencies(${PROJECT_NAME}
    ${catkin_EXPORTED_TARGETS}
    ${${PROJECT_NAME}_EXPORTED_TARGETS}
)


#############
## Install ##
#############

# Mark executables and/or libraries for installation
install(TARGETS ${PROJECT_NAME}
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY launch/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)
install(DIRECTORY config/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config)


#############
## Tests ##
#############

if(CATKIN_ENABLE_TESTING)
    find_package(rostest REQUIRED)
    add_rostest_gtest(test_h264_video_encoder
        test/test_h264_video_encoder.test
        test/h264_video_encoder_test.cpp)
    target_include_directories(test_h264_video_encoder PRIVATE ${h264_video_encoder_INCS})
    target_link_libraries(test_h264_video_encoder ${PROJECT_NAME}_lib)
endif()

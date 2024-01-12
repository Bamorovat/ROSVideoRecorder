cmake_minimum_required(VERSION 3.0.2)
project(video_recorder)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  rospy
  sensor_msgs
  std_msgs
)

# find_package(Boost REQUIRED COMPONENTS system)
find_package(OpenCV REQUIRED)

catkin_python_setup()

catkin_package(
   CATKIN_DEPENDS rospy std_msgs sensor_msgs cv_bridge image_transport
   DEPENDS OpenCV
)

## Specify additional locations of header files
## Your package locations should be listed before other locations

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS scripts/VideoRecorder.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


cmake_minimum_required(VERSION 2.8.3)
project(camera_to_velodyne)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp std_msgs sensor_msgs 
	geometry_msgs pcl_ros tf tf_conversions nav_msgs eigen_conversions visualization_msgs)

###################################
## catkin specific configuration ##
###################################
catkin_package(INCLUDE_DIRS include)

###########
## Build ##
###########
set(CMAKE_BUILD_TYPE Release)
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)


add_library(camera_to_velodyne src/camera_to_velodyne.cpp)
target_link_libraries(camera_to_velodyne ${catkin_LIBRARIES} yaml-cpp)

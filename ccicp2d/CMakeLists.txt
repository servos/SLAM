cmake_minimum_required(VERSION 2.8.3)
project(ccicp2d)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp std_msgs sensor_msgs 
	geometry_msgs pcl_ros tf eigen_conversions tf_conversions ground_segmentation)

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

add_library(ccicp2d src/icp.cpp src/icpPointToPoint.cpp src/kdtree.cpp src/matrix.cpp src/icpTools.cpp )
target_link_libraries(ccicp2d ${catkin_LIBRARIES} ground_segmentation)

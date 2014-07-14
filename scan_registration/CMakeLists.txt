cmake_minimum_required(VERSION 2.8.3)
project(scan_registration)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp std_msgs sensor_msgs 
	geometry_msgs pcl_ros tf tf_conversions  visualization_msgs
    nav_msgs microstrain_3dmgx2_imu eigen_conversions
    actionlib actionlib_msgs nasa_msgs ground_segmentation ccicp2d)

find_package(OpenCV)
include_directories(${OpenCV_INCLUDE_DIRS})

###################################
## catkin specific configuration ##
###################################
#catkin_package(INCLUDE_DIRS include)
catkin_package()

###########
## Build ##
###########
set(CMAKE_BUILD_TYPE Release)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)


add_executable(scan_registration src/scan_registration.cpp) 

target_link_libraries(scan_registration ${catkin_LIBRARIES} ccicp2d)

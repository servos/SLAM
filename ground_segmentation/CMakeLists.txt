cmake_minimum_required(VERSION 2.8.3)
project(ground_segmentation)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp std_msgs sensor_msgs 
	geometry_msgs pcl_ros tf tf_conversions  visualization_msgs
    nav_msgs microstrain_3dmgx2_imu 
    nasa_msgs)

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


#add_executable(ground_segmentation src/ground_segmentation.cpp src/groundSegmentation.cpp)
add_library(ground_segmentation src/groundSegmentation.cpp)

set_target_properties(ground_segmentation PROPERTIES COMPILE_FLAGS "-ftemplate-depth-256" )

target_link_libraries(ground_segmentation ${catkin_LIBRARIES} )

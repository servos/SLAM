cmake_minimum_required(VERSION 2.8.3)
project(local_mapper)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp std_msgs sensor_msgs 
	geometry_msgs pcl_ros tf tf_conversions  visualization_msgs
    nav_msgs  graph_slam ekf nasa_msgs mls eigen_conversions ground_segmentation)

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


add_executable(local_mapper src/local_mapper.cpp )

target_link_libraries(local_mapper ${OpenCV_LIBRARIES} ${catkin_LIBRARIES} mls)

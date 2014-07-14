cmake_minimum_required(VERSION 2.8.3)
project(ekf)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp std_msgs sensor_msgs nav_msgs 
	geometry_msgs pcl_ros tf tf_conversions )


catkin_package()

###########
## Build ##
###########
set(CMAKE_BUILD_TYPE Release)
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)
#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
add_executable(ekf_node src/nasa_ekf.cpp)
 target_link_libraries(ekf_node
   ${catkin_LIBRARIES}  )




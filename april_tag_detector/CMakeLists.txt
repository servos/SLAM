cmake_minimum_required(VERSION 2.8.3)
project(april_tag_detector)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp std_msgs sensor_msgs nav_msgs 
	geometry_msgs tf tf_conversions )

catkin_package()

###########
## Build ##
###########
set(CMAKE_BUILD_TYPE Release)
include_directories(
  ${catkin_INCLUDE_DIRS}
)
#common commands for building c++ executables and libraries
add_executable(april_tag_detector src/april_tag_detector.cpp)
 target_link_libraries(april_tag_detector
   ${catkin_LIBRARIES}  )




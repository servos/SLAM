cmake_minimum_required(VERSION 2.8.3)
project(global_matching)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp std_msgs sensor_msgs graph_slam 
	geometry_msgs pcl_ros tf tf_conversions )



add_message_files(FILES GlocCloud.msg)
generate_messages(DEPENDENCIES sensor_msgs geometry_msgs)

###################################
## catkin specific configuration ##
###################################
catkin_package()

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

#rosbuild_add_executable(cylinder_segmentation src/cyl_segmentation.cpp)
#rosbuild_add_executable(save_laser_file src/save_laser_file.cpp)
add_executable(global_matching src/global_match.cpp)
add_executable(global_transform src/global_transform.cpp)
add_executable(global_generate src/global_generate.cpp)
 target_link_libraries(global_matching
   ${catkin_LIBRARIES}  )
 target_link_libraries(global_transform
   ${catkin_LIBRARIES}  )
 target_link_libraries(global_generate
   ${catkin_LIBRARIES}  )

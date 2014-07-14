cmake_minimum_required(VERSION 2.8.3)
project(graph_slam)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp std_msgs sensor_msgs nav_msgs 
	geometry_msgs pcl_ros tf tf_conversions message_generation visualization_msgs mls ccicp2d
    eigen_conversions ground_segmentation actionlib nasa_msgs)



add_message_files(FILES Edge.msg )
generate_messages(DEPENDENCIES std_msgs geometry_msgs)

###################################
## catkin specific configuration ##
###################################
catkin_package()

###########
## Build ##
###########
set(CMAKE_BUILD_TYPE Release)
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)


#find_package(libg2o REQUIRED)
#include_directories(/usr/include/suitesparse)
set(libg2o_INCLUDE_DIRS /usr/local/include)

add_executable(graph_slam  src/graph_slam.cpp src/global_mapping_viz.cpp src/graphSlamTools.cpp)
#add_executable(gs_test  src/gs_test.cpp)

target_link_libraries (graph_slam  mls ccicp2d 
    g2o_ext_csparse g2o_csparse_extension g2o_stuff g2o_core g2o_solver_csparse g2o_types_slam3d
    ${catkin_LIBRARIES})
#target_link_libraries (gs_test ${catkin_LIBRARIES})

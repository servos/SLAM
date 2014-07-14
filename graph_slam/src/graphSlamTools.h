#ifndef GRAPHSLAMTOOLS_H
#define GRAPHSLAMTOOLS_H

#include <vector>
#include <iostream>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/registration/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <vector>
#include <algorithm>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Path.h>

#include "ccicp2d/icpTools.h"

#include "graph_slam/Edge.h"

#include "graph_slam.h"

#define KNN_DIST_THRESH 5.0 //distance between kf's
#define GSLAM_KNN 3 //number of nearest neighbours to add in pose graph

//FIXME: this should be an enum
#define CTYPE_HOME 0
#define CTYPE_ODOM 1
#define CTYPE_NN 2

//edge rejection based on ICP allowed movement
#define DIST_MOVE_THRESH 10 //in meters
#define ROT_MOVE_THRESH 0.2 //in radians

namespace Eigen
   {
	typedef Eigen::Matrix<double, 6, 1> Vector6d;
    typedef Eigen::Matrix<double, 6, 6> Matrix6d;
   }

double graphSlamGetNearestKF(PoseGraph& pG, Node& gN);
void graphSlamAddEdgeToPrevious(PoseGraph& pG);
void graphSlamAddEdgeToHome(PoseGraph& pG);
bool graphSlamAddEdgeToX(PoseGraph& pG, int from, int to);
vector<int> graphSlamGetKNN(PoseGraph& pG, Node& gN, int K);
Eigen::Matrix6d computeEdgeInformationLUM(pcl::PointCloud<pcl::PointXYZ>::Ptr& source_trans, pcl::PointCloud<pcl::PointXYZ>::Ptr& target, double max_corr_dist);

void setup_gicp();

#endif

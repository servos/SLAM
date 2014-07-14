#ifndef GRAPH_SLAM_H
#define GRAPH_SLAM_H

#include <vector>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <geometry_msgs/PoseStamped.h>

struct Edge
{
    int from;
    int to;
    geometry_msgs::PoseStamped edge; 
	Eigen::MatrixXd edgeCov;
	Eigen::MatrixXd edgeInf;
	int ctype;

    Edge():edgeCov(6,6) {}
};

struct Node
{
    int idx;
    geometry_msgs::PoseStamped pose;
    pcl::PointCloud<pcl::PointXYZ>::Ptr keyframe;

    Node():keyframe(new pcl::PointCloud<pcl::PointXYZ>) {}
};

typedef std::vector<struct Edge> EdgeList;
typedef std::vector<struct Node> NodeList;

struct PoseGraph 
{
    NodeList nodes;
    EdgeList edges;
};


#endif

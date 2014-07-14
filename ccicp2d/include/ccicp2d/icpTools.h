#ifndef ICPTOOLS_H
#define ICPTOOLS_H

#include <vector>

#include "ground_segmentation/PointcloudXYZGD.h"
#include "ground_segmentation/groundSegmentation.h"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <algorithm>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PoseStamped.h>

//ICP includes
//#include "scan_registration2D.h"

#define ICP_MAX_PTS 20000

//ground adjacentcy 
#define NUMBINSGA 1200
#define RESOLUTION 0.5
#define GRD_ADJ_THRESH 2

using namespace std;

enum RegistrationType {
    SCAN_TO_SCAN,
    SCAN_TO_MAP
};

class CCICP {

private:
	double* refPts_GA;
	double* refPts_NGA;
	double* tarPts_GA;
	double* tarPts_NGA;

    pcl::PointCloud<PointXYZGD>::Ptr seg_target;
    pcl::PointCloud<PointXYZGD>::Ptr seg_scene;

    pcl::PointCloud<PointXYZGD>::Ptr ground_target;
    pcl::PointCloud<PointXYZGD>::Ptr ground_scene;
    
    RegistrationType type;

    groundSegmentation gSeg;


    void classifyPoints(pcl::PointCloud<PointXYZGD>::Ptr outcloud);
    void segmentGround(pcl::PointCloud<pcl::PointXYZ>::Ptr incloud, pcl::PointCloud<PointXYZGD>::Ptr outcloud, pcl::PointCloud<PointXYZGD>::Ptr ground_cloud);

    geometry_msgs::PoseStamped doICPMatch(vector<Eigen::Vector2d,Eigen::aligned_allocator<Eigen::Vector2d> >& model_GA,
            vector<Eigen::Vector2d,Eigen::aligned_allocator<Eigen::Vector2d> >& model_NGA,
            vector<Eigen::Vector2d,Eigen::aligned_allocator<Eigen::Vector2d> >& scene_GA, 
            vector<Eigen::Vector2d,Eigen::aligned_allocator<Eigen::Vector2d> >& scene_NGA,
            geometry_msgs::PoseStamped& initPose);

    geometry_msgs::PoseStamped doRPHRegistration(pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr scene_cloud,geometry_msgs::PoseStamped& initPose);
    geometry_msgs::PoseStamped doHeightOnlyReg(pcl::PointCloud<pcl::PointXYZ>::Ptr target, pcl::PointCloud<pcl::PointXYZ>::Ptr scene,geometry_msgs::PoseStamped& initPose);
    geometry_msgs::PoseStamped doHeightInterpolate(pcl::PointCloud<pcl::PointXYZ>::Ptr ground,geometry_msgs::PoseStamped& initPose);

public:
    CCICP(RegistrationType type_ = SCAN_TO_SCAN);

    //Setters
    void setTargetCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr target, geometry_msgs::PoseStamped& initPose);
    void setTargetGndCloud( pcl::PointCloud<pcl::PointXYZ>::Ptr target);
    void setSceneCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr scene);
    
    //returns the residual of the ICP
    double getResidual();

    //Calculated the ICP of the target and scene clouds

    //Calls setter functions fisrt
    geometry_msgs::PoseStamped doICPMatch(pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr scene_cloud, geometry_msgs::PoseStamped& initPose, RegistrationType type = SCAN_TO_SCAN); 
    //Assumes setters have been used
    geometry_msgs::PoseStamped doICPMatch(geometry_msgs::PoseStamped& initPose);

    void getSegmentedClouds(pcl::PointCloud<pcl::PointXYZ>& target, pcl::PointCloud<pcl::PointXYZ>& scene,pcl::PointCloud<pcl::PointXYZ>& ground_target,pcl::PointCloud<pcl::PointXYZ>& ground_scene);

};

#endif

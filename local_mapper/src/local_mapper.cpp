
#include <vector>
#include <algorithm>
#include <fstream>

// PCL Includes
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>

//ROS includes
#include <ros/ros.h>
#include <tf/tf.h>
#include <tf_conversions/tf_eigen.h>
#include <eigen_conversions/eigen_msg.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/Pose.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>

//Our Includes
#include <mls/mls.h>


MLS local_map(200, 200, 0.2, true);

geometry_msgs::PoseStamped curPose;

ros::Publisher localCloudPub; 
ros::Publisher drivablilityPub;

pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud(new pcl::PointCloud<pcl::PointXYZ>);

ros::Time pose_time;
ros::Time cloud_time;
bool newCloud = false;

void pose_cb(const geometry_msgs::PoseStamped& pose)
{
    curPose = pose;
    pose_time = pose.header.stamp;
}

void poseOffet_cb(const geometry_msgs::PoseStamped& pose)
{
    local_map.offsetMap(pose);
}

void cloud_cb(const sensor_msgs::PointCloud2ConstPtr& input)
{
    //Update Map  
    pcl::fromROSMsg(*input, *input_cloud);

    cloud_time = input->header.stamp;
    newCloud = true;

    //shoudl we setPose() here?

}

int
main (int argc, char** argv)
{
  // Initialize ROS
    ros::init (argc, argv, "local_mapper");
    ros::NodeHandle nh;

    // Create a ROS subscriber for the input point cloud
    ros::Subscriber poseSub = nh.subscribe("/mapping/ekf/pose", 1, pose_cb);
    //ros::Subscriber poseSub = nh.subscribe("/mapping/scan_reg/pose", 1, pose_cb);
    ros::Subscriber poseOffsetSub = nh.subscribe("/mapping/graph_slam/pose_offset", 1, poseOffet_cb);
    ros::Subscriber sub = nh.subscribe  ("/velodyne_points", 1, cloud_cb);

    localCloudPub = nh.advertise<sensor_msgs::PointCloud2>("/mapping/local_poiuntcloud", 1, true);
    drivablilityPub = nh.advertise<nav_msgs::OccupancyGrid>("/mapping/local_drivability", 1, true);
    ros::Publisher mlsPub = nh.advertise<visualization_msgs::MarkerArray>("/mapping/local/mls_viz", 1, true);


    //Set map options
    //local_map.setDisablePointCloud(true);
    //local_map.setMaxClusterPoints(100);
    local_map.setMinClusterPoints(20);

    //local_map.setClusterDistTheshold(1.0);
    //local_map.setClusterSigmaFactor(5);
    //local_map.setNormalTheshold(0.2);
    //local_map.setHeightTheshold(0.3);

    local_map.clearMap();

    ros::Rate loop_rate(50); //FIXME should this be higer?
    
    int count = 0; 
    while(ros::ok()) {
        loop_rate.sleep();
        ros::spinOnce();

        if(newCloud && (pose_time-cloud_time).toSec() >= 0) {
            //FIXME this only syncs to the scan regisration... roll and pitch will be off....
            newCloud = false;
            count++;

            local_map.addToMap(input_cloud, curPose);

            //publish cloud and map
            pcl::PointCloud<pcl::PointXYZ>::Ptr local_cloud;
            sensor_msgs::PointCloud2 cloud_msg;
            local_map.filterPointCloud(0.1,0.1); //filter gloibal cloud so it won't kill rviz
            local_cloud = local_map.getGlobalCloud();
            pcl::toROSMsg(*local_cloud,cloud_msg);
            cloud_msg.header.frame_id = "/local_oriented";
            cloud_msg.header.stamp = ros::Time::now();
            localCloudPub.publish(cloud_msg);

            nav_msgs::OccupancyGrid::Ptr drivability;
            drivability = local_map.getDrivability();
            drivability->header.frame_id = "/local_oriented";
            drivablilityPub.publish(drivability);
            
           local_map.visualize(mlsPub, "/local_oriented");

        }

    }
  
}



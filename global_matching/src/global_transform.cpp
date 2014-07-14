
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <time.h>
// PCL specific includes
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/registration/gicp.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/io/pcd_io.h>
#include <tf/tf.h>
#include <iostream>
#include <fstream>

//use this file to transform a point cloud to proper global co-ordinates

ros::Publisher pub_map;
sensor_msgs::PointCloud2 ros_map;

pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> gicp;
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_map (new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr tcloud_map (new pcl::PointCloud<pcl::PointXYZ>);


Eigen::Matrix4f transformation;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "global_transform");
    ros::NodeHandle nh;
    pub_map=nh.advertise<sensor_msgs::PointCloud2>("/mapping/global_transform_map",1);

    //read map cloud
    ROS_INFO_STREAM("Loading Map Point Cloud...");
    if (pcl::io::loadPCDFile<pcl::PointXYZ> ("maps/WPI_FINAL.pcd", *cloud_map) == -1) // load the file
	{
		ROS_ERROR_STREAM("No map cloud found");
		return (-1);
	}
	ROS_INFO_STREAM("Done Loading Map");

	//eye transform
	transformation.block(0,0,2,2) <<  1,0,0,1;
	transformation.block(0,3,3,1) << 0,0,0;
	transformation(2,2) = 1;
	transformation(3,3) = 1;

	
	//put in the transform (WPI)
	//transformation.block(0,0,2,2) <<  0.98627,-0.16516,0.16516,0.98627;
	//transformation.block(0,3,3,1) <<  74.249,-24.841,0;
	//transformation(2,2) = 1;
	//transformation(3,3) = 1;
	
	//put in the transform (SDC)
	//transformation.block(0,0,2,2) <<  0.98415,-0.17736,0.17736,0.98415;
	//transformation.block(0,3,3,1) << 5,-10,0;
	//transformation(2,2) = 1;
	//transformation(3,3) = 1;
	
    std::cout<<"Transform is : " <<std::endl << transformation <<std::endl;
	pcl::transformPointCloud(*cloud_map, *tcloud_map, transformation);

	//save the cloud
	//pcl::io::savePCDFileASCII ("maps/WPI_Transform.pcd", *tcloud_map);
	//std::cerr << "Saved " << tcloud_map->size () << " data points to WPI_Transform.pcd." << std::endl;
	
    ros::Rate r(1);
    while(ros::ok()) {
		pcl::toROSMsg(*tcloud_map,ros_map);
		ros_map.header.frame_id = "/nasa";
		ROS_INFO_STREAM("Publishing Map");
		pub_map.publish(ros_map);
    	ros::spinOnce();
    	r.sleep();
    }
    return 0;
}

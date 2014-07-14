//
//  scan_registration.cpp
//
//  This node warps the scan registration library to subscribe and
//  publish ros messages. `
//
//

//C includes

//ros includes
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/tf.h>
#include <eigen_conversions/eigen_msg.h>

//pcl includes
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>

//libraries
#include "ccicp2d/icpTools.h"

//local includes
#include "scan_registration.h"


using namespace std;
using namespace pcl;

//ros

#define DEBUG  1

ros::Publisher posePub;
ros::Publisher targetPub;
ros::Publisher scenePub;

//ros occupancy data struct
pcl::PointCloud<PointXYZ>::Ptr input_cloud (new pcl::PointCloud<PointXYZ>);
pcl::PointCloud<PointXYZ>::Ptr target_obs_cloud (new pcl::PointCloud<PointXYZ>);
pcl::PointCloud<PointXYZ>::Ptr target_gnd_cloud (new pcl::PointCloud<PointXYZ>);

#if DEBUG
    pcl::PointCloud<PointXYZ>::Ptr target (new pcl::PointCloud<PointXYZ>);
    pcl::PointCloud<PointXYZ>::Ptr scene (new pcl::PointCloud<PointXYZ>);
    pcl::PointCloud<PointXYZ>::Ptr target_ground (new pcl::PointCloud<PointXYZ>);
    pcl::PointCloud<PointXYZ>::Ptr scene_ground (new pcl::PointCloud<PointXYZ>);
#endif

geometry_msgs::PoseStamped poseOut;

CCICP icp(SCAN_TO_MAP);

//////////////////////
//Callbacks
//
void pose_cb(const geometry_msgs::PoseStamped& input)
{
	poseOut = input;
}

bool obs_flag = false;
bool gnd_flag = false;

bool first_gnd = false;
bool first_obs = false;

void target_obs_cb(const sensor_msgs::PointCloud2ConstPtr& target_input)
{
	
	 //ROS_INFO_STREAM("Got obs Cloud");
    //This is in the global frame
    pcl::fromROSMsg (*target_input, *target_obs_cloud);
    obs_flag = true;
    
    if(obs_flag && gnd_flag) {
        icp.setTargetCloud(target_obs_cloud, poseOut);
        icp.setTargetGndCloud(target_gnd_cloud);
        obs_flag = false;
        gnd_flag = false;
    }
    
    first_obs = true;
}

void target_gnd_cb(const sensor_msgs::PointCloud2ConstPtr& target_input)
{
    //This is in the global frame
    pcl::fromROSMsg (*target_input, *target_gnd_cloud);
    gnd_flag = true;
    if(obs_flag && gnd_flag) {
        icp.setTargetCloud(target_obs_cloud, poseOut);
        icp.setTargetGndCloud(target_gnd_cloud);
        obs_flag = false;
        gnd_flag = false;
    }
    
     first_gnd = true;
}

////////////////////////////////////
//MAIN CALLBACK
// most work happens in this callback
void cloud_cb(const sensor_msgs::PointCloud2ConstPtr& input)
{
	
	//wait for first ground and obs targets
	
	if(!(first_gnd && first_obs))
		return;
	
	input_cloud->clear();
    pcl::fromROSMsg (*input, *input_cloud);
    //note: this is in the local frame
    
    if(target_obs_cloud->empty()) return;
    if(input_cloud->size() < 20000) {
        ROS_WARN_STREAM("Input Cloud is to small!! Size: " << input_cloud->size());
        return;
    }
    
    //Compensate for roll and pitch
    Eigen::Affine3d trans;
    pcl::PointCloud<pcl::PointXYZ>::Ptr temp(new pcl::PointCloud<pcl::PointXYZ>); 
    geometry_msgs::Pose cpose;
    double roll, pitch, yaw;
    tf::Quaternion q;
    tf::quaternionMsgToTF(poseOut.pose.orientation, q);
    tf::Matrix3x3(q).getEulerYPR(yaw, pitch, roll,1); //get angles
    cpose.orientation = tf::createQuaternionMsgFromRollPitchYaw(roll,pitch,0); //remove yaw
    cpose.position.z = poseOut.pose.position.z; 
    tf::poseMsgToEigen(cpose, trans); //generate transform
    pcl::transformPointCloud(*input_cloud,*temp, trans); //transform cloud
    icp.setSceneCloud(temp);

#if DEBUG
    icp.getSegmentedClouds(*target, *scene,*target_ground,*scene_ground);
    sensor_msgs::PointCloud2 cloud_msg;
    pcl::toROSMsg(*scene,cloud_msg);
    cloud_msg.header.frame_id = "/local";
    cloud_msg.header.stamp = ros::Time::now();
    scenePub.publish(cloud_msg);
#endif


	///////////////////////////////////////////
	//MAIN FUNCTIONS
	///////////////////////////////////////////
    
    //Perform scan registration!
    //   output should be in the glbal frame given initial position poseOut
    //   and traget_cloud are in global frame
    geometry_msgs::PoseStamped result;
    result = icp.doICPMatch(poseOut);

    if(result.pose.orientation.w == 9999) {
        //An error occured
        ROS_ERROR_STREAM("ICP could not complete registration, skipping this scan");
        return;
    }

    poseOut = result;

	//publish pose
	poseOut.header.stamp = input->header.stamp;
	poseOut.header.frame_id = "/global";
	posePub.publish(poseOut);
}  

int main (int argc, char** argv)
{
    // Initialize ROS
    ros::init (argc, argv, "scan_registration");
    ros::NodeHandle nh;

    ros::Subscriber poseSub = nh.subscribe  ("/mapping/ekf/pose", 1, pose_cb);

    // Create a ROS subscriber for the input point cloud
    ros::Subscriber subScene = nh.subscribe  ("/velodyne_points", 1, cloud_cb);
    ros::Subscriber subObsTarget = nh.subscribe  ("/mapping/global/obstacle_pointcloud", 1, target_obs_cb);
    ros::Subscriber subGndTarget = nh.subscribe  ("/mapping/global/ground_pointcloud", 1, target_gnd_cb);
    
    //ros publisher for pose
    posePub = nh.advertise<geometry_msgs::PoseStamped>("mapping/scan_reg/pose", 1);

   //debugging topics 
    targetPub = nh.advertise<sensor_msgs::PointCloud2>("mapping/scan_reg/target", 1);
    scenePub = nh.advertise<sensor_msgs::PointCloud2>("mapping/scan_reg/scene", 1);

    //init Pose
    poseOut.pose.orientation.w = 1; //normalized quaternion

    // Spin
    ros::spin ();

}



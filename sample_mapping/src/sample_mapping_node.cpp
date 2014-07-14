#include "sample_mapping/sample_mapping.h"

//Our Includes
#include <calibration/camera_to_velodyne.h>

//globals

geometry_msgs::PoseStamped curPose;
ros::Publisher sampleMapPub;
ros::Time pose_time;
bool first_pose=false;

//opencv

static const std::string OPENCV_WINDOW = "sample map window";

//new sample map

SampleMap SM(400,400,0.5);

void pose_cb(const geometry_msgs::PoseStamped& pose)
{
    curPose = pose;
    pose_time = pose.header.stamp;
    first_pose=true;
}


int main (int argc, char** argv)
{
  // Initialize ROS
    ros::init (argc, argv, "sample_mapper");
    ros::NodeHandle nh;

    // Create a ROS subscriber for the input point cloud
    ros::Subscriber poseSub = nh.subscribe("/mapping/ekf/pose", 1, pose_cb);
    sampleMapPub = nh.advertise<nav_msgs::OccupancyGrid>("/mapping/sample_map", 1, true);
    ros::Rate loop_rate(10); //FIXME should this be higer?
    int loopcounter = 0;
    
    //set the integration params
    intparameters ip;
    ip.prob_max = 5;
    ip.prob_min = 0;
    ip.rmax = 10;
    ip.rmin = 1;
    SM.setIntegrationParameters(ip);
    //init the map
    SM.initalizeSampleMap();
    
    std::vector<SampleGaussian> sampleVec;
    
    SampleGaussian sample1 = SampleGaussian(1,0.1, 5,0);
    SampleGaussian sample2 = SampleGaussian(1,0.1, 20,-10);
    SampleGaussian sample3 = SampleGaussian(5,0.2, 20,1);
    
    sampleVec.push_back(sample1);
    sampleVec.push_back(sample2);
    sampleVec.push_back(sample3);
    
    SM.addSampleToMap(sample3.x,sample3.y,sample3);
     
     
    //test camera to velodnye
    CameraToVelodyne CtoV;
    
    cv::Mat src;
    //src = cv::Mat::zeros(600, 600, CV_8UC1);

    while(ros::ok()) {
        loop_rate.sleep();
        
        double curr_x = 0;
		double curr_y = 0;
		double curr_theta = 0;
        
        if(first_pose)
        {
        
			curr_x = curPose.pose.position.x;
			curr_y = curPose.pose.position.y;
			curr_theta = tf::getYaw(curPose.pose.orientation);
		}
		
        
      
              
        if(loopcounter%10 == 0)
        {
			//pretend we see a new sample
		   int ridx = (rand() % (int)(1+ 1));
		   SampleGaussian rsample = sampleVec[ridx];
			//add it to the map
		   SM.addSampleToMap(rsample.x,rsample.y,rsample);
		}
        
        //remove from map
        
        SM.removeConeFromMap(curr_x,curr_y,curr_theta, 0.008);
        
        //publish the occupancy grid
        
        nav_msgs::OccupancyGrid::Ptr sample_occupancy;
        sample_occupancy = SM.getSampleOccupancy();
        sampleMapPub.publish(sample_occupancy);
       
        //std::cout<<"published occupancy"<<std::endl;
        
        // Update GUI Window
        src = SM.getCVMap();
		cv::imshow(OPENCV_WINDOW, src);
		cv::waitKey(3);
               
        ros::spinOnce();
        loopcounter++;
      

    }
  
}

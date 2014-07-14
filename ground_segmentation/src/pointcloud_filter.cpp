#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <vector>
#include <algorithm>
#include <fstream>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/Pose.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/Imu.h>

//must be before PCL includes
#include "pointcloud_filter/PointcloudXYZGD.h"

// PCL Includes
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h> //needed by voxel grid don't know why....
//#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/impl/voxel_grid.hpp>
#include <pcl/surface/mls.h>
//#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/kdtree/impl/kdtree_flann.hpp>
#include <tf/tf.h>
#include <tf_conversions/tf_eigen.h>

#include "groundSegmentation.h"

struct driveCellInfo
{
        float z;
        int isDrivable;

};  


#define NUMABINS 250 //number of angular sectors
#define NUMLBINS 250 //number of linear bins
#define NONDRIVE_POINTS_THRESH 1
#define DRIVEUNKNOWN -1000
#define VOXSIZE_XY 0.5
#define VOXSIZE_Z  1.1

#define GRD_ADJ_THRESH 2

#define USE_RECTANGULAR
#define USE_QUATS

#ifndef USE_QUATS
#include <microstrain_3dmgx2_imu/imu_3dmgx3_msg.h>
#endif

groundSegmentation gSeg;

//ros
ros::Publisher pub;
ros::Publisher pubGnd;
ros::Publisher pubDrv;
ros::Publisher occupancyGridPub;

//ros occupancy data struct

nav_msgs::OccupancyGrid occGrid;

//pointcloud data structures

pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud (new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<PointXYZGD>::Ptr gCloudTemp (new pcl::PointCloud<PointXYZGD>);
pcl::PointCloud<PointXYZGD>::Ptr proc_cloud (new pcl::PointCloud<PointXYZGD>);

pcl::PointCloud<PointXYZGD>::Ptr ground_cloud (new pcl::PointCloud<PointXYZGD>);
pcl::PointCloud<PointXYZGD>::Ptr obs_cloud (new pcl::PointCloud<PointXYZGD>);
pcl::PointCloud<PointXYZGD>::Ptr drv_cloud (new pcl::PointCloud<PointXYZGD>);
pcl::PointCloud<pcl::PointXYZ>::Ptr obs_cloud_noI (new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<PointXYZGD>::Ptr cloud_temp (new pcl::PointCloud<PointXYZGD>);

pcl::PointCloud<pcl::PointNormal>::Ptr mlsCloud (new pcl::PointCloud<pcl::PointNormal>);
sensor_msgs::PointCloud2 output_msg;
pcl::VoxelGrid<PointXYZGD> sor;
pcl::MovingLeastSquares< pcl::PointXYZ, pcl::PointNormal > MLSFilter; 

// Create a KD-Tree
pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);

using namespace std;
vector<vector<PointXYZGD> > ptBins; 
driveCellInfo driveCells[NUMABINS][NUMLBINS]; 

static const char WINDOW[] = "Map Image";

//translation from body frame to laser frame
#define BF_X -0.19685
#define BF_Y 0.3937
#define BF_Z 1.270

Eigen::Quaterniond curQuat;
double curRoll = 0;
double curPitch = 0;


std::fstream timeOut;

void compRollPitchCloud(pcl::PointCloud<PointXYZGD>::Ptr & cloud, double roll, double pitch)
{
	//map to points from perspective of body frame
    Eigen::Matrix4f trans;

#ifdef USE_QUATS
    trans.block(0,0,3,3) << curQuat.toRotationMatrix().cast<float>();
    trans(3,3) = 1;
#else
    trans << cos(roll), 0, sin(roll), 0,
             sin(roll)*sin(pitch), cos(pitch), -cos(roll)*sin(pitch), 0,
             -cos(pitch)*sin(roll), sin(pitch), cos(roll)*cos(pitch), 0,
             0, 0, 0, 1;
#endif

  //TODO: add translation
  //cout << trans << endl << endl;

  pcl::transformPointCloud(*cloud, *cloud_temp, trans);
  *cloud = *cloud_temp;
}


#ifdef USE_QUATS
void imu_cb_quats(const sensor_msgs::ImuConstPtr& input)
{
    double roll = 0;
    double pitch = 0;
    double yaw = 0;

    tf::Quaternion q;
    tf::quaternionMsgToTF(input->orientation, q);
    tf::Matrix3x3 mat = tf::Matrix3x3(q);
    mat.getEulerYPR(yaw, pitch, roll,2);

    //Old Orientation
    //mat.setEulerYPR(0, roll,  pitch+M_PI);
    //New Orientation
   // cout << pitch << " " << roll << endl;
    //**arun edit
    
    mat.setEulerYPR(M_PI, pitch,  roll);
    mat.getRotation(q);
    tf::quaternionTFToEigen(q, curQuat);
    
}

#else
void imu_cb(const microstrain_3dmgx2_imu::imu_3dmgx3_msgConstPtr& input)
{
    curRoll = input->roll;
    curPitch = input->pitch;
        //cout << input->yaw << " " << input->pitch << " " << input->roll << endl;
}

//DEPRICATED: ONLY USDED WHEN NOT USING GP
PointXYZGD compRollPitch(double roll, double pitch, float ptx, float pty, float ptz)
{
  //map to points from perspective of body frame

        PointXYZGD ret;
        ret.x = ptx*cos(roll) + ptz*sin(roll) + BF_X;
        ret.y = pty*cos(pitch) - ptz*cos(roll)*sin(pitch) + ptx*sin(roll)*sin(pitch) + BF_Y;
        ret.z = pty*sin(pitch) + ptz*cos(roll)*cos(pitch) - ptx*cos(pitch)*sin(roll);

        return ret;
}

#endif

bool heightSort (PointXYZGD p1,PointXYZGD p2) 
{ 
    return (p1.z < p2.z); 
}

void addToCloud(int lin_idx, pcl::PointCloud<PointXYZGD>::Ptr outcloud, int groundAdj = 0) 
{
    int num_bin_pts = ptBins[lin_idx].size();
    if(num_bin_pts < NONDRIVE_POINTS_THRESH) {
        return;
    }
    for(int k=0;k<num_bin_pts;k++) {
         PointXYZGD pt;
       	 pt.x = ptBins[lin_idx][k].x;
         pt.y = ptBins[lin_idx][k].y;             
       	 pt.z = ptBins[lin_idx][k].z;
       	 pt.ground_adj = groundAdj;
         pt.drivable = ptBins[lin_idx][k].drivable;
         outcloud->push_back(pt); //add the point to outcloud
    }
}

void segment_ground(pcl::PointCloud<PointXYZGD>::Ptr incloud, pcl::PointCloud<PointXYZGD>::Ptr outcloud, pcl::PointCloud<PointXYZGD>::Ptr drvcloud)
{
    outcloud -> clear();
    int nPts = incloud->points.size();

    gSeg.setupGroundSegmentation(incloud,ground_cloud,outcloud,drvcloud);
	gSeg.segmentGround();

    //cout << incloud->size() << ", " << outcloud->size() << endl;

    //cout << "Done segmenting" << endl;
    //start with everything as drivable
    for(int i = 0; i<NUMABINS; i++) {
        for(int j=0; j<NUMLBINS;j++) {
            driveCells[i][j].isDrivable = 1;
        }
    }
 
#ifdef USE_RECTANGULAR
    for(int i=0;i<NUMABINS*NUMLBINS;i++)
    {
        ptBins[i].clear();                      
    }

    //rebin into rectangular
    double bsize_x = (double)2*RMAX / NUMABINS; //in degrees
    double bsize_y = (double)2*RMAX / NUMLBINS; //in meters
    nPts = outcloud->points.size();

    for(int i=0;i<nPts;i++) //binning stage
    {
        PointXYZGD pt = outcloud->points[i];
	    float curr_x = pt.x;
        float curr_y = pt.y;

    	if(sqrt(curr_x*curr_x + curr_y*curr_y) < RMAX)
        {
	        int bind_x = floor((curr_x+RMAX) /bsize_x);
		    int bind_y = floor((curr_y+RMAX) /bsize_y); 
	       
            int hidx = (bind_x*NUMLBINS + bind_y);
            ptBins[hidx].push_back(pt);
            driveCells[bind_x][bind_y].isDrivable = 0;
	    }

    }

    outcloud->clear();
#endif
     
    //Determine ground adjacentcy
    for(int i = 0; i<NUMABINS; i++) {
        for(int j=0; j<NUMLBINS;j++) {
            int lin_idx = i*NUMLBINS + j; //linear index of bin
            int occGridIdx = j*NUMABINS +i;
            
            if(driveCells[i][j].isDrivable != 0) {
                continue;
            }
            
            if( i == 0 || i == NUMABINS-1 || j == 0 || j == NUMLBINS-1) {
                //TODO What should I do with the edges of the grid??
                //add to cloud
                addToCloud(lin_idx, outcloud, 0);
                continue;
            }
            
            int groundCount = 0;
            for(int q = i -1; q <= i+1; q++) {
                for( int r = j -1; r <= j+1; r++) {
                    if(q == i && r == j) {
                        continue;
                    }
                    if(driveCells[q][r].isDrivable != 0) {
                        groundCount++;
                    }
                }
            }
            
            if(groundCount >= GRD_ADJ_THRESH) {
                //mark as ground adjacent
                addToCloud(lin_idx, outcloud, 1);
            } else {
                addToCloud(lin_idx, outcloud, 0);
            }
        }
    }
        
       
}

void cloud_cb(const sensor_msgs::PointCloud2ConstPtr& input)
{
    double tempTime = ros::Time::now().toSec();

    if(input->width < 30000) {
      ROS_ERROR_STREAM("Velodyne scan incomplete. Cloud points: "<< input->width);
      return;
    }
	  //ROS_INFO("got the cloud");
    //init the bin grid
    for(int i=0;i<NUMABINS*NUMLBINS;i++)
    {
            ptBins[i].clear();                      
    }

    //clear all clouds	
    ptBins.clear();
    input_cloud->clear();
    obs_cloud->clear();
    drv_cloud->clear();
	  ground_cloud->clear(); 
	  cloud_temp->clear(); 	
	  gCloudTemp->clear();

    pcl::fromROSMsg (*input, *input_cloud);
    copyPointCloud(*input_cloud, *proc_cloud);
    compRollPitchCloud(proc_cloud,curRoll, curPitch); //roll/pitch compensation
  
    segment_ground(proc_cloud,obs_cloud,drv_cloud); //perform ground segmentation

    //ground cloud
    if(obs_cloud->size() == 0) {
      ROS_WARN_STREAM("No obsticale points!");
      return;
    }
    
    sor.setLeafSize(VOXSIZE_XY,VOXSIZE_XY,VOXSIZE_Z);
    sor.setInputCloud(ground_cloud);
    sor.filter(*gCloudTemp); 
    pcl::toROSMsg(*gCloudTemp,output_msg);
    output_msg.header.frame_id = "/nasa";
    output_msg.header.stamp = input->header.stamp;
    pubGnd.publish(output_msg);

    //obsticale cloud
    sor.setInputCloud(obs_cloud);
    sor.filter(*gCloudTemp); 
    pcl::toROSMsg(*gCloudTemp,output_msg);
    output_msg.header.frame_id = "/nasa";
    output_msg.header.stamp = input->header.stamp;
    pub.publish (output_msg);

    //drv cloud
    sor.setLeafSize(0.1, 0.1, 0.1);
    sor.setInputCloud(drv_cloud);
    sor.filter(*gCloudTemp); 
    pcl::toROSMsg(*gCloudTemp,output_msg);
   // pcl::toROSMsg(*drv_cloud,output_msg);
    output_msg.header.frame_id = "/nasa";
    output_msg.header.stamp = input->header.stamp;
    pubDrv.publish (output_msg);

    timeOut << ros::Time::now().toSec() - tempTime << endl;
}

int
main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "local_mapping_node");
  ros::NodeHandle nh;

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe  ("/velodyne_points", 1, cloud_cb);
   
  // Create a ROS subscriber for the input point cloud
#ifdef USE_QUATS
  ros::Subscriber imu_sub = nh.subscribe  ("/imu/data", 1, imu_cb_quats);
#else
  ros::Subscriber imu_sub = nh.subscribe  ("/imu/imu_3dm_gx3", 1, imu_cb);
#endif
  // Create a ROS publisher for the output point cloud
  pub = nh.advertise<sensor_msgs::PointCloud2> ("mapping/velodyne_obs", 1);
  pubGnd = nh.advertise<sensor_msgs::PointCloud2> ("mapping/velodyne_gnd", 1);
  pubDrv = nh.advertise<sensor_msgs::PointCloud2> ("mapping/velodyne_drv", 1);

  //ros publisher for map

  occupancyGridPub = nh.advertise<nav_msgs::OccupancyGrid> ("mapping/local_occupancy", 1);
                
  //resize the binning vector
  ptBins.resize(NUMABINS*NUMLBINS);
  occGrid.data.resize(NUMABINS*NUMLBINS);
  fill(occGrid.data.begin(), occGrid.data.end(),-1);
      
  //timeOut.open("/home/jdservos/timeOutLocal.csv", fstream::out); 
    
  //Init rotation
  tf::Quaternion q;
  tf::Matrix3x3 mat;
  mat.setEulerYPR(0, 0,  0);
  mat.getRotation(q);
  tf::quaternionTFToEigen(q, curQuat);
    
  // Spin
  ros::spin ();
  
   timeOut.close();
}



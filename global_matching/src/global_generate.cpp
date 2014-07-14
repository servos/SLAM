#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <time.h>
// PCL specific includes
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/registration/gicp.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/statistical_outlier_removal.h>

#include <iostream>
#include <fstream>

#define arrLen(x)  sizeof(x)/sizeof(x[0])

#define MAX_DIST 4.0

#define MAX_SCORE 1.0

#define VOXEL_FILTER
#define LEAF_SIZE 0.30

#define CROP
#define CROP_DIST 100.0

//#define BUILD_FULL

ros::Publisher pub;
sensor_msgs::PointCloud2 ros_out;
ros::Publisher pub2;
sensor_msgs::PointCloud2 ros_out2;
ros::Publisher pub3;
sensor_msgs::PointCloud2 ros_out3;
ros::Publisher pub_full;
sensor_msgs::PointCloud2 ros_full;
ros::Publisher pub_small;
sensor_msgs::PointCloud2 ros_small;

pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZI, pcl::PointXYZI> gicp;

pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_targ (new pcl::PointCloud<pcl::PointXYZI>);
pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_targ_filt(new pcl::PointCloud<pcl::PointXYZI>);
pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_in (new pcl::PointCloud<pcl::PointXYZI>);
pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_in_filt (new pcl::PointCloud<pcl::PointXYZI>);
pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_final (new pcl::PointCloud<pcl::PointXYZI>);
pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_small (new pcl::PointCloud<pcl::PointXYZI>);
pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_full (new pcl::PointCloud<pcl::PointXYZI>);
pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_temp (new pcl::PointCloud<pcl::PointXYZI>);


int new_cloud;
int i = 0;

void laser_callback(const sensor_msgs::PointCloud2& point_cloud) {
    pcl::PointCloud<pcl::PointXYZI>::Ptr temp(new pcl::PointCloud<pcl::PointXYZI>);

    if(cloud_in->size() != 0) {
        if(cloud_small->size() == 0) {
            //pcl::toROSMsg(*cloud_in,ros_full);
            //pcl::toROSMsg(*cloud_in,ros_small);
#ifdef BUILD_FULL
            *cloud_full = *cloud_in;
#endif
            *cloud_small = *cloud_in;
        }
        //pcl::fromROSMsg(ros_small, *cloud_targ);
        cloud_targ = cloud_small;
        //cloud_targ = cloud_in;
    }

    cloud_in = temp;
    pcl::fromROSMsg(point_cloud, *cloud_in);

    new_cloud++;
}

void setup_gicp() {
    // Set the max correspondence distance 
    gicp.setMaxCorrespondenceDistance (10);
    // Set the maximum number of iterations (criterion 1)
    gicp.setMaximumIterations (100);
    // Set the transformation epsilon (criterion 2)
    gicp.setTransformationEpsilon (1e-6);
    // Set the euclidean distance difference epsilon (criterion 3)
    gicp.setEuclideanFitnessEpsilon (1e-6);
    gicp.setRANSACIterations(0);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "global_transform");
    ros::NodeHandle nh;
    ros::Time time;
    new_cloud = -1;
    
    std::ofstream fout;
    fout.open("gicp_results.csv");

    ros::Subscriber cloud_sub = nh.subscribe("velodyne_points", 1, laser_callback);
    pub=nh.advertise<sensor_msgs::PointCloud2>("output",1);
    pub2=nh.advertise<sensor_msgs::PointCloud2>("output2",1);
    pub3=nh.advertise<sensor_msgs::PointCloud2>("output3",1);
    pub_full=nh.advertise<sensor_msgs::PointCloud2>("output_full",1);
    pub_small=nh.advertise<sensor_msgs::PointCloud2>("output_small",1);
    
    //setup gicp
    setup_gicp();

   pcl::VoxelGrid<pcl::PointXYZI> sor;
   sor.setLeafSize (LEAF_SIZE,LEAF_SIZE,LEAF_SIZE);

   pcl::PassThrough<pcl::PointXYZI> pass;

    Eigen::Matrix4f transformation;
    Eigen::Matrix4f trans_full;
    trans_full << Eigen::MatrixXf::Identity(4,4);
    while(ros::ok()) {
        ros::spinOnce();
        if(new_cloud < 1) {
            continue;
        }
        time = ros::Time::now();
        new_cloud = 0;
        // Create the filtering object
        #ifdef VOXEL_FILTER
            sor.setInputCloud (cloud_in);
            sor.filter(*cloud_in_filt);

            //cloud_targ_filt = cloud_small;
            sor.setInputCloud(cloud_targ);
            sor.filter(*cloud_targ_filt);
            *cloud_small = *cloud_targ_filt;
            //pcl::toROSMsg(*cloud_targ_filt,ros_small);
        #else
            cloud_in_filt = cloud_in;
            cloud_targ_filt = cloud_targ;
        #endif
        
        pcl::transformPointCloud(*cloud_in_filt, *cloud_in_filt, trans_full);
        
        #ifdef CROP
            //std::cout << "cropping" << std::endl;
            // Create the filtering object
            pass.setFilterLimits ((-CROP_DIST+trans_full(0,3)), (CROP_DIST+trans_full(0,3)));
            pass.setInputCloud (cloud_targ_filt);
            pass.setFilterFieldName ("x");
            pass.filter (*cloud_temp);

            pass.setFilterLimits (-CROP_DIST+trans_full(1,3), CROP_DIST+trans_full(1,3));
            pass.setInputCloud (cloud_temp);
            pass.setFilterFieldName ("y");
            pass.filter (*cloud_targ_filt);

            //filter input
            pass.setFilterLimits (-CROP_DIST+trans_full(0,3), CROP_DIST+trans_full(0,3));
            pass.setInputCloud (cloud_in_filt);
            pass.setFilterFieldName ("x");
            pass.filter (*cloud_temp);

            pass.setFilterLimits (-CROP_DIST+trans_full(1,3), CROP_DIST+trans_full(1,3));
            pass.setInputCloud (cloud_temp);
            pass.setFilterFieldName ("y");
            pass.filter (*cloud_in_filt);

        #endif
        //Calculate gicp
        gicp.setInputCloud(cloud_in_filt);
        gicp.setInputTarget(cloud_targ_filt);
        
        std::cout << "Map Points: " << cloud_small->size() << std::endl;
        gicp.align(*cloud_final);
        
        float score = gicp.getFitnessScore(MAX_DIST);
        std::cout << " Score: " <<
         score << std::endl;
        
        if(score > MAX_SCORE) {
            std::cout << trans_full << std::endl << std::endl;
            continue;
        }
        
        transformation =  gicp.getFinalTransformation();
        trans_full = transformation*trans_full;
        
        std::cout << "Time: " <<  ros::Time::now().toSec() - time.toSec() << std::endl;
        std::cout << trans_full << std::endl << std::endl;
   
        // outputResult(fout, rotVec_init, transformation, gicp.getFitnessScore());
        fout << trans_full(0,3) << ", "<< trans_full(1,3) << ", "<< trans_full(2,3) << '\n';//std::endl;
        
        pcl::transformPointCloud(*cloud_in, *cloud_final, trans_full);
        
        pcl::toROSMsg(*cloud_targ_filt,ros_out);
        pcl::toROSMsg(*cloud_in_filt,ros_out2);
        pcl::toROSMsg(*cloud_final,ros_out3);
        ros_out.header.frame_id = "/velodyne";
        ros_out2.header.frame_id = "/velodyne";
        ros_out3.header.frame_id = "/velodyne";
        ros_full.header.frame_id = "/velodyne";
        ros_small.header.frame_id = "/velodyne";

        //pcl::concatenatePointCloud(ros_full, ros_out3, ros_full);
        //pcl::concatenatePointCloud(ros_small, ros_out3, ros_small);
#ifdef BUILD_FULL
        (*cloud_full) += *cloud_final;
        pcl::toROSMsg(*cloud_full,ros_full);
#endif
        
        //noise filter
/*        pcl::StatisticalOutlierRemoval<pcl::PointXYZI> nf;
        nf.setInputCloud (cloud_final);
        nf.setMeanK (50);
        nf.setStddevMulThresh (3.0);
        nf.filter (*cloud_temp);
        *cloud_final = *cloud_temp;
*/

        (*cloud_small) += *cloud_final;
        
        pcl::toROSMsg(*cloud_small,ros_small);
        pub.publish(ros_out);
        pub2.publish(ros_out2);
        pub3.publish(ros_out3);
        pub_full.publish(ros_full);
        pub_small.publish(ros_small);

    }//endloop
    
    fout.close();
    
    //save pointcloud
    std::cout << "Saving Point Cloud" << std::endl;
    pcl::io::savePCDFileASCII ("maps/WPIArun.pcd", *cloud_small);
    //pcl::io::savePLYFile ("/home/jdservos/test_data/pointclouds/plyfile.ply", *cloud_small);

    return 0;
}

#ifndef SAMPLEMAPPING_H
#define SAMPLEMAPPING_H

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
#include <tf/tf.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>


struct cell
{
	double probability;
	
};

struct intparameters
{
	double prob_max;
	double prob_min;
	double rmin;
	double rmax;
};

class SampleGaussian
{
	private:
	double bound; //3 sigma bound
	
	public:
	double amplitude;
	double cov; //sigma squared
	double x;
	double y;
	
	SampleGaussian(double amp, double var,double xpos, double ypos)
	{
		amplitude = amp;
		cov = var;
		bound = 3*sqrt(var);
		x = xpos;
		y = ypos;
	}
	
	double getProbabilityValue(double x, double y) //gets the probability value for the gaussian (assume centered at 0,0)
	{
		return amplitude*exp( -1.0*( (x*x)/(2*cov) + (y*y)/(2*cov)) );
	}
	
	double getBound(){return bound;}
	
	
};

class SampleMap {  
    private:
    
    cell* data; //grid data
    int size_x; //in bins
    int size_y; //in bins
    double size_x_meters;
    double size_y_meters;
    double xmin_meters;
    double ymin_meters;
    double resolution;
    nav_msgs::OccupancyGrid::Ptr sample_grid;
    intparameters integration_parameters;
    double fov_cone;
    
    //opencv variables
    cv::Mat cv_map;
    double pixel_prob_threshold; // min probability in order to make it into the cv map (normalize 0 - 1)
    std::vector<cv::Point2d> sample_points; //vector of sample locations in global xy
          
    //functions
    
    int computeSampleGaussian(double cov);
    int mapPixelToXY(int camera, int u, int v); //maps a pixel uv co-ordinate to a XY (meters) location in the map
    int mapXYToLinear(double x, double y);  //converts global XY co-ordinates to a linear index for map cell
    int mapIJToXY(double i, double j, double& x, double& y);  //converts map i-j coordinates to global XY
    int mapIJToLinear(double i, double j); //maps 2 index in cells to linear index
	int mapProbabilityToOccupancy(double _input_range_min, double _input_range_max, double _input_value_tobe_converted);
	void copyMaptoCV();
	void processMapCV();
		
	public:
    
    SampleMap(int xbins, int ybins, double res);
    void initalizeSampleMap();
   	void addSampleToMap(double x, double y, SampleGaussian sample); //adds a sample at global x y to the map with a mean prob and cov
	void removeConeFromMap(double x, double y, double theta, double dec_value);
	void setIntegrationParameters(intparameters ip){ integration_parameters = ip;}
	nav_msgs::OccupancyGrid::Ptr getSampleOccupancy() { return sample_grid; }
	cv::Mat getCVMap(){ copyMaptoCV(); processMapCV(); return cv_map;}
  
};

#endif

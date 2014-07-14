#include "ros/ros.h"
#include <unistd.h>
#include <iostream>
#include <sstream>
#include <pcl/point_types.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <tf/tf.h>

#define NUMINITIMUSAMPLES 200
#define LOOPRATE 100 //hz
#define PUBRATE 1.0 //hz
#define PUBITER (int)(((double)LOOPRATE)/((double)PUBRATE))

//motion noise
#define MOTNOISE 0.05
#define MEASNOISEYAW 0.01
#define MEASNOISESLAM 10

struct imuMeasurement
{
	bool newMeas; 
	double dt;
	double ax;
	double ay;
	double az;
	double gx;
	double gy;
	double gz;
	
	//bias terms
	
	double bias_gz;
};

struct slamMeasurement
{
	bool newMeas; 
	double x;
	double y;
	double z;
	double pitch;
	double roll;
	double yaw;
	
	
};

struct globalMeasurement
{
	bool newMeas; 
	double x;
	double y;
	double yaw;
	
	
		
};


struct cmdPacket
{
	bool newCmd; 
	double dt;
	double ux;
	double uy;
	double uw;
};





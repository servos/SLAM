#include <ros/ros.h>

//definitions
using namespace std;

//global variables

int main(int argc, char **argv)
{

	ros::init(argc, argv, "april_tag_detector");
	ros::NodeHandle n;
	ros::Rate loop_rate(20);	

	//subscriber
	//FIXME camera image: ros::Subscriber imu_sub = n.subscribe("/imu/data", 1, imuCallback);	
	
	//publisher
    //pose_pub = n.advertise<geometry_msgs::PoseStamped> ("mapping/apriltag/pose", 1);

	while (ros::ok())
  	{
		
		ros::spinOnce(); //check for new sensor messages
		loop_rate.sleep(); //sleep
	}

	return 0;
}









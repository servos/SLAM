/*
nasa robot kalman filter
states: x,y,yaw (pass the roll and pitch of IMU directly to output message)
*/

#include "nasa_ekf.h"
#include <Eigen/LU>
#include <tf/transform_broadcaster.h>

//definitions
using namespace std;

//global variables
double imuTime = 0;
bool ekfInitialized = false;
imuMeasurement currImu;
slamMeasurement currSlam;
cmdPacket currCmd;
globalMeasurement currGlobal;

Eigen::MatrixXd R; //motion noise
double Qyaw; //measurement noise on yaw
Eigen::MatrixXd Qslam; //measurement noise on SLAM
double imuStamp = 0;
double cmdStamp=0;
double gyroDelta = 0;

double curPitch =0;
double curRoll =0;
double newRoll =0;
double newPitch =0;

ros::Time scan_time;

//ekf variables
Eigen::VectorXd mu(8); //state
Eigen::VectorXd mubar(8); //predicted state
Eigen::VectorXd controlIn(3); //control input
Eigen::MatrixXd sigma; //state cov
Eigen::MatrixXd sigmabar; //predicted cov

//ros publishers
ros::Publisher pose_pub;
ros::Publisher path_pub;

//ekf path
nav_msgs::Path ekfPath;

//convert euler angles to quat
geometry_msgs::Quaternion Euler2Quat(double yaw, double pitch, double roll) 
{
    // Assuming the angles are in radians.
    double cy = cos(yaw/2);
    double sy = sin(yaw/2);
    double cp = cos(pitch/2);
    double sp = sin(pitch/2);
    double cr = cos(roll/2);
    double sr = sin(roll/2);
     //quaternion to return

    geometry_msgs::Quaternion ret;
        
    ret.w = cr*cp*cy + sr*sp*sy;
    ret.x = -cr*sp*sy + cp*cy*sr;
    ret.y = cr*cy*sp + sr*cp*sy;
    ret.z = cr*cp*sy - sr*cy*sp;

    return ret;
}

double quat2Euler(geometry_msgs::Quaternion q)
{
    double ret;
    float q0, q1,q2,q3;
    q0 = q.w;
    q1 = q.x;
    q2 = q.y;
    q3 = q.z;

    // Converting to Yaw (SHOULD be from -pi to pi because of atan2)
    ret = atan2(2*(q0*q3 + q1*q2), 1 - 2*(q2*q2 + q3*q3));
    return ret;
    
}
//check pi wrap around (standardize to -pi to pi)
double angleWrap(double a)
{
	double ret=0;
	if(a>M_PI)
		ret = -2*M_PI +a;
	else if(a< -M_PI)
		ret = 2*M_PI -a;
	else
		ret = a;

	return ret;
}

//subtract two yaw values, takes care of the case where the difference crosses the -pi to pi boundary
double subtractYaw(double a, double b)
{
	double diff = a-b;
	if(diff>M_PI)
		diff = -2*M_PI + diff;
	else if(diff<-M_PI)
		diff = 2*M_PI + diff;
	return diff;
}

/*
function for imu callback
*/

void imuCallback(const sensor_msgs::Imu& msg)
{
	
	currImu.dt = msg.header.stamp.toSec() - imuStamp;
	imuStamp = msg.header.stamp.toSec();
	currImu.gz = msg.angular_velocity.z;
	currImu.newMeas=true;


    //extract pitch and roll
    tf::Quaternion q;
    tf::quaternionMsgToTF(msg.orientation, q);
    tf::Matrix3x3 mat = tf::Matrix3x3(q);
    double dummy;
    mat.getEulerYPR(dummy, newPitch, newRoll,1); //123 euler angles.  First rotate about X, then Y, then Z
    curPitch = newPitch;
    curRoll = newRoll;

}

/*
function for mapping callback
*/

void mappingCallback(const geometry_msgs::PoseStamped& msg)
{
	//this gets the x,y,z, and yaw pitch roll angles from scan registration
	
	currSlam.x = msg.pose.position.x;
	currSlam.y = msg.pose.position.y;
	currSlam.z = msg.pose.position.z;
	currSlam.yaw = quat2Euler(msg.pose.orientation);
	currSlam.newMeas=true;
	 
    scan_time = msg.header.stamp;
}

/*
function for global pose callback
*/

void globalCallback(const geometry_msgs::PoseStamped& msg)
{
	
	
	double dx = msg.pose.position.x; //delta on global x
	double dy = msg.pose.position.y; //delta on global y
	double dyaw = quat2Euler(msg.pose.orientation); //delta on yaw

	mu(0) =  mu(0) + dx; //shift x
	mu(1) = mu(1) + dy; //shift y
	mu(6) = mu(6) + dyaw;

	mubar(0) =  mubar(0) + dx; //shift x
	mubar(1) = mubar(1) + dy; //shift y
	mubar(6) = mubar(6) + dyaw;

	mu(6) = angleWrap(mu(6)); //check for angle wrap!
	mubar(6) = angleWrap(mubar(6)); //check for angle wrap!

	currGlobal.newMeas = true;
	 
}


/*
function: motion update
Inputs: sigma, mu, controlIn
Outputs: mubar, sigmabar

*/
void motion_update(Eigen::MatrixXd sigma,Eigen::VectorXd mu,Eigen::VectorXd controlIn,Eigen::VectorXd& mubar,Eigen::MatrixXd& sigmabar,double dt)
{
	//state vector
	/*0 = inertial x
	1 = inertial y
	2 = inertal x vel
	3 = inertial y vel
	4 = body x vel
	5 = body y vel
	6 = inertial yaw
	7 =  body yaw rate*/
			
	//new motion model and linearization 
	double vx = controlIn(0); //body x vel
	double vy = controlIn(1); //body y vel
	double X = mu(0); //inertial X
	double Y = mu(1); //inertial Y
	double yaw = mu(6) + mu(7)*dt; // inertial yaw
	double omega = mu(7);//controlIn(2); //body yaw rate
	double vX = vx*cos(yaw) - vy*sin(yaw); //inertial x vel
	double vY = vx*sin(yaw) + vy*cos(yaw); //inertial y vel

	//propogate motion model
	mubar(0) = X +vX*dt;
	mubar(1) = Y +vY*dt;
	mubar(2) = vX;
	mubar(3) = vY;
	mubar(4) = vx;
	mubar(5) = vy;
	mubar(6) = yaw;
	mubar(7) = omega;

	//angle wrap
	mubar(6) = angleWrap(mubar(6));
	
	//linearize the motion model
	Eigen::MatrixXd G = Eigen::MatrixXd::Zero(8,8);
	G(0,0) = 1; G(0,2)=dt;
	G(1,1) = 1; G(1,3) = dt;
	G(2,4) = cos(yaw); G(2,5) = -sin(yaw); G(2,6) = -vx*sin(yaw) -vy*cos(yaw);
	G(3,4) = sin(yaw); G(3,5) = cos(yaw); G(3,6) = vx*cos(yaw)-vy*sin(yaw);
	G(6,6) = 1; G(6,7) = dt;

	//calc the predicted covariance (sigmabar)
	sigmabar = G*sigma*G.transpose() + R;
	
	//ROS_INFO_STREAM("Motion MU : " <<endl <<mubar<<endl);
	

}

/*
function: measurement update
Inputs:  mubar(3x1), sigmabar(3x3), measIn(gyro rate)
Outputs: sigma(3x3), mu(3x1)

*/

void yaw_measurement_update(Eigen::VectorXd mubar,Eigen::MatrixXd sigmabar,double measIn, Eigen::MatrixXd& sigma, Eigen::VectorXd& mu, double dt)
{

	Eigen::VectorXd C(8); C<<0,0,0,0,0,0,0,1; //direct measurement of yaw rate
	Eigen::MatrixXd I = Eigen::MatrixXd::Identity(8,8); 
	
	double temp = 1.0/(C.transpose()*sigmabar*C +Qyaw); 
	Eigen::VectorXd K(8); K = sigmabar*C*temp; 
	double yawRateMeas = measIn; 
	mu = mubar + K*(yawRateMeas - C.transpose()*mubar); 
	sigma = (I-K*C.transpose())*sigmabar; 

	//angle wrap
	mu(6) = angleWrap(mu(6));

	//ROS_INFO_STREAM(mu<<endl<<endl);;
	
}

void slam_measurement_update(Eigen::VectorXd mubar,Eigen::MatrixXd sigmabar,Eigen::VectorXd measIn, Eigen::MatrixXd& sigma, Eigen::VectorXd& mu)
{

	Eigen::MatrixXd C(3,8); C = Eigen::MatrixXd::Zero(3,8); //direct measurement of X, Y and Yaw
	Eigen::MatrixXd I = Eigen::MatrixXd::Identity(8,8); 

	//set up C matrix
	C(0,0) = 1; //measure X
	C(1,1) = 1; //measure Y
	C(2,6) = 1; //measure Yaw

	//create innovation vector
	Eigen::MatrixXd innovation(3,1);
	//predicted measurements from measurement model and state
	Eigen::MatrixXd pMeas = C*mubar;
	innovation(0,0) = measIn(0) - pMeas(0,0); //x measurement
	innovation(1,0) = measIn(1) - pMeas(1, 0); //y measurement
	innovation(2,0) = subtractYaw(measIn(2),pMeas(2, 0)); //take the acute angle, make sure the measIn(2) is standardized to -pi to pi
	
	Eigen::Matrix3d temp; 
    temp = (C*sigmabar*C.transpose() + Qslam);  
	temp = temp.inverse().eval(); 
	Eigen::MatrixXd K(8,3); K = sigmabar*C.transpose()*temp; 
	mu = mubar + K*(innovation); 
	sigma = (I-K*C)*sigmabar; 

	//ROS_INFO_STREAM("SLAM MU : " <<endl <<mu<<endl);

}

void initialize_ekf()
{
	//init motion and meas noise
	R= Eigen::MatrixXd::Identity(8,8)*MOTNOISE;
	Qyaw=MEASNOISEYAW;
	Qslam = Eigen::MatrixXd::Identity(3,3)*MEASNOISESLAM;
	
	int try_count =0;
	//find the gyro bias
	//ROS_INFO_STREAM("Initalizing gyro bias -- DO NOT MOVE ROBOT"<<endl);
	double gzSum =0;
	for(int i=0; i<NUMINITIMUSAMPLES && ros::ok() && try_count < NUMINITIMUSAMPLES*20;i++)
	{
		try_count = 0;
		while(currImu.newMeas==false && ros::ok() && try_count < NUMINITIMUSAMPLES*20) { //wait for new sensor messages
			ros::spinOnce(); 
			ros::Duration(0.005).sleep();
			try_count++;
		}
				
		gzSum +=currImu.gz;
		currImu.newMeas=false; //reset the meas
		
		ROS_DEBUG_STREAM("Initalizing gyro bias -- DO NOT MOVE ROBOT: waiting for "<< NUMINITIMUSAMPLES - i << " more samples" <<endl);
		
	}

	if(try_count >= NUMINITIMUSAMPLES*20) {
		ROS_INFO_STREAM( "IMU Initializetion failed... ");
	}
	
	//got all the samples, average them
	currImu.bias_gz = gzSum/(double)NUMINITIMUSAMPLES;
	ROS_DEBUG_STREAM("Gyro bias is "<<currImu.bias_gz <<" rad/s");

	//FIXME: no longer waiting for global localization 
	//while(currGlobal.newMeas==false && ros::ok())
	//	ros::spinOnce();

	ROS_INFO_STREAM(" Initaization complete"<<endl);
	
	ekfInitialized=true;
	
}

/*
function for cmd callback
*/

void cmdCallback(const geometry_msgs::TwistStamped& msg)
{
	//check for nan
	if( msg.twist.linear.x!=msg.twist.linear.x || msg.twist.linear.y!=msg.twist.linear.y || msg.twist.angular.z!=msg.twist.angular.z)
	{	
		ROS_ERROR_STREAM( " Cmd_vel is NANing!!!!! " << endl);
		currCmd.dt=msg.header.stamp.toSec()- cmdStamp;
		cmdStamp = msg.header.stamp.toSec();
		currCmd.ux = 0;
		currCmd.uy = 0;
		currCmd.uw = 0;
	}
	else
	{
		currCmd.dt= msg.header.stamp.toSec()- cmdStamp;// = ros::Time::now().toSec()- cmdStamp;
		cmdStamp = msg.header.stamp.toSec();
        //FIXME: this is hardcoded but we should know this from params
		currCmd.ux = max(min(msg.twist.linear.x, 3.0),-3.0);
		currCmd.uy = max(min(msg.twist.linear.y, 3.0),-3.0);
		currCmd.uw = max(min(msg.twist.angular.z, 0.75),-0.75);

	}

	controlIn<<currCmd.ux,currCmd.uy,currCmd.uw;
	currCmd.newCmd = true; //have to skip the first one to make sure we get good time stamp
	 
}


int main(int argc, char **argv)
{

	//time variables
	double t1=0;
	double t2=0;
	double dt = 0;
	unsigned int numIter = 0;

	//state vector
	mu<<0,0,0,0,0,0,0,0;
	mubar<<0,0,0,0,0,0,0,0;
	controlIn<<0,0,0;
	sigma = Eigen::MatrixXd::Identity(8,8)*0.001;
	sigmabar = Eigen::MatrixXd::Identity(8,8)*0.001;

	ros::init(argc, argv, "nasa_ekf");
	ros::NodeHandle n;
	ros::Rate loop_rate(LOOPRATE);	
	
	//subscriber
	ros::Subscriber imu_sub = n.subscribe("/imu/data", 1, imuCallback);	
	//ros::Subscriber cmd_sub = n.subscribe("/cmd_vel", 1, cmdCallback);
	ros::Subscriber cmd_sub = n.subscribe("/chassis/vel", 1, cmdCallback);
	ros::Subscriber mapping_sub = n.subscribe("/mapping/scan_reg/pose", 1, mappingCallback); //from scan registration
	
    //FIXME: EKF not listening to graph slam!
    //ros::Subscriber global_sub = n.subscribe("/mapping/graph_slam/pose_offset", 1, globalCallback); //will be from graph slam
	
	//publisher
	path_pub = n.advertise<nav_msgs::Path> ("mapping/ekf/ekf_path_viz", 1);
	pose_pub = n.advertise<geometry_msgs::PoseStamped> ("mapping/ekf/pose", 1);

    tf::TransformBroadcaster br;

	//init measurement 
	currImu.newMeas=false;
	currSlam.newMeas=false;
	currGlobal.newMeas=false;
	//command init
	currCmd.newCmd = false;
	currCmd.ux = 0;
	currCmd.uy = 0;
	currCmd.uw = 0;
	
	if(ekfInitialized == false) //initalize the ekf
	{
		initialize_ekf();
	}
	
	t1 = ros::Time::now().toSec();
	while (ros::ok())
  	{
		
		ros::spinOnce(); //check for new sensor messages
			
		//perform motion update
		t2 = ros::Time::now().toSec();
		dt = t2-t1;
		motion_update(sigma,mu,controlIn,mubar,sigmabar,dt);
		t1=t2;		

		//perform measurement update 
		if(currImu.newMeas==false && currSlam.newMeas==false) //no new measurement(s), assume best state is predicted state
		{
			mu = mubar;
			sigma = sigmabar;
		}
		else //new measurement, perform meas update(s), depending on what we have in
		{
			if(currImu.newMeas) //new yaw meas, put it in
			{
				yaw_measurement_update(mubar,sigmabar, (currImu.gz-currImu.bias_gz),sigma,mu,currImu.dt);
				currImu.newMeas = false;
				mubar=mu; //predicted is now mu, since we just finished a meas update
				sigmabar=sigma;
			}
			if(currSlam.newMeas) //new slam meas, put it in
			{
				//generate the measurements
				Eigen::VectorXd meas(3); //x y and yaw
				meas(0) = currSlam.x;  meas(1) = currSlam.y; meas(2) = angleWrap(currSlam.yaw);
				slam_measurement_update(mubar,sigmabar,meas,sigma,mu);
				currSlam.newMeas = false;
				mubar=mu; //predicted is now mu, since we just finished a meas update
				sigmabar=sigma;
			}
		}
		
		//publish to pose topic for visualization
		geometry_msgs::PoseStamped posViz;
		posViz.header.stamp = scan_time;
		posViz.header.frame_id = "/global";
		posViz.pose.position.x = mu(0);
		posViz.pose.position.y = mu(1);
		posViz.pose.position.z = currSlam.z;
		posViz.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(curRoll, curPitch, mu(6));//Euler2Quat(mu(6),curPitch,curRoll);
		pose_pub.publish (posViz);

		if(numIter >=PUBITER)
		{
			numIter =0; //reset the iteration counter
			//publish path topic according to PUBITER
			ekfPath.header = posViz.header;
			ekfPath.header.frame_id = "/global";
			ekfPath.poses.push_back(posViz);
			path_pub.publish(ekfPath);
		}


        //Setup Transforms:
        tf::Transform transform;
        transform.setOrigin(tf::Vector3(posViz.pose.position.x, posViz.pose.position.y, 0));
        transform.setRotation(tf::Quaternion(0,0,0,1));

        br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "global", "local_oriented"));


        transform.setOrigin(tf::Vector3(posViz.pose.position.x, posViz.pose.position.y, posViz.pose.position.z));
        transform.setRotation(tf::Quaternion( posViz.pose.orientation.x, posViz.pose.orientation.y,
                                                posViz.pose.orientation.z, posViz.pose.orientation.w));

        br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "global", "local"));
        br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "global", "velodyne"));


		loop_rate.sleep(); //sleep
		numIter++; //iteration counter
			
	}

	return 0;
}









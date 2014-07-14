#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <nasa_msgs/ObjectMapCam.h>

//Our Includes
#include <calibration/camera_to_velodyne.h>

using namespace std;

//globals
// camera to velodnye
CameraToVelodyne CtoV;
geometry_msgs::PoseStamped curPose;
ros::Publisher sampleConversionPub;
ros::Time pose_time;
bool first_pose=false;


void pose_cb(const geometry_msgs::PoseStamped& pose)
{
    curPose = pose;
    pose_time = pose.header.stamp;
    
}


Eigen::Vector3d convert_uv_to_xyz(int cam_id, int u, int v){

		
	Eigen::Vector2d camuv(u,v);
	Eigen::Vector3d cray = CtoV.camerauv_to_cameraray(camuv,cam_id);
		
	Eigen::Vector3d rpoint;
	rpoint = CtoV.cameraray_to_rangeflatground(cray,cam_id,-1.3);//-1.464); //last param is the height of the sensor from the ground
	
	
	//now convert the point in velodyne frame to global frame
	tf::Quaternion q_robot;
    tf::quaternionMsgToTF(curPose.pose.orientation, q_robot);   
    Eigen::Matrix4d transformation = Eigen::Matrix4d::Identity();
    tf::Matrix3x3 matq(q_robot);
    transformation << matq[0][0], matq[0][1], matq[0][2], curPose.pose.position.x,
                     matq[1][0], matq[1][1], matq[1][2], curPose.pose.position.y,
                     matq[2][0], matq[2][1], matq[2][2], curPose.pose.position.z,
                     0,0,0,1;
                     
     Eigen::Vector4d rpoint_h(rpoint(0),rpoint(1),rpoint(2),1);
     Eigen::Vector4d transformed_rpoint = transformation*rpoint_h;
     
     Eigen::Vector3d xyz_location(transformed_rpoint(0),transformed_rpoint(1),transformed_rpoint(2));
	
	 return xyz_location;

}

void camera_cb(const nasa_msgs::ObjectMapCam& cam)
{
    if(cam.state >= 1) { 
        nasa_msgs::ObjectMapCam xyzObjectCam;
        Eigen::Vector3d xyz_location = convert_uv_to_xyz(cam.camera_id, (int)cam.x, (int)cam.y);
        //copy all the info
        xyzObjectCam = cam;
        xyzObjectCam.global_pose.position.x = xyz_location(0);
        xyzObjectCam.global_pose.position.y = xyz_location(1);
        xyzObjectCam.global_pose.position.z = xyz_location(2);
        
        sampleConversionPub.publish(xyzObjectCam); //publish message with xyz global info
    }
}

int main (int argc, char** argv)
{
  // Initialize ROS
    ros::init (argc, argv, "sample_mapper");
    ros::NodeHandle nh;

    // Create a ROS subscriber for the current pose
    ros::Subscriber poseSub = nh.subscribe("/mapping/ekf/pose", 1, pose_cb);
    //ros publisher for sample xyz location
    sampleConversionPub = nh.advertise<nasa_msgs::ObjectMapCam>("/mapping/cam_center", 1, true);
    //ros subscriber for cameara uv info
    ros::Subscriber cameraSub = nh.subscribe("/sample_detection/cam_center", 1, camera_cb); //FIXME: change me to the right topic!
    
    ros::Rate loop_rate(50); 
    
    curPose.pose.position.x=0;curPose.pose.position.y=0;curPose.pose.position.z=0;
    curPose.pose.orientation.x=0;curPose.pose.orientation.y=0;curPose.pose.orientation.z=0;curPose.pose.orientation.w=1;
       

    while(ros::ok()) {
        loop_rate.sleep();
        
	ros::spinOnce();
        /*double curr_x = 0;
		double curr_y = 0;
		double curr_theta = 0;
        cout<<"converting"<<endl;
        cout<< convert_uv_to_xyz(0,3020,2020) <<endl;*/

    }
  
}

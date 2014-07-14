#ifndef CAMERA_TO_VELODNYE_H
#define CAMERA_TO_VELODNYE_H

#include <ros/ros.h>
#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <Eigen/Eigenvalues>
#include <cassert>
#include <algorithm>
#include <tf/tf.h>
#include <yaml-cpp/yaml.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <eigen_conversions/eigen_msg.h>
#include <ros/package.h>

namespace Eigen
   {
    typedef Eigen::Matrix<double, 3, 4> Matrix34d;
    typedef Eigen::Matrix<double, 5, 1> Vector5d;
   }

struct camera_parameters
{
	int cam_id;
	Eigen::Matrix34d projection_matrix;
	Eigen::Matrix4d extrinsics; //from camera frame to velodyne frame
	Eigen::Vector5d distortion_coefficients;
	Eigen::Matrix3d camera_matrix;
	
	
};

void operator >> (const YAML::Node& node, Eigen::Matrix34d& v) {
	  
	node[0] >> v(0,0); node[1] >> v(0,1); node[2] >> v(0,2); node[3] >> v(0,3);
	node[4] >> v(1,0); node[5] >> v(1,1); node[6] >> v(1,2); node[7] >> v(1,3);
	node[8] >> v(2,0); node[9] >> v(2,1); node[10] >> v(2,2); node[11] >> v(2,3);
}

void operator >> (const YAML::Node& node, Eigen::Matrix4d& v) {
	node[0] >> v(0,0); node[1] >> v(0,1); node[2] >> v(0,2); node[3] >> v(0,3);
	node[4] >> v(1,0); node[5] >> v(1,1); node[6] >> v(1,2); node[7] >> v(1,3);
	node[8] >> v(2,0); node[9] >> v(2,1); node[10] >> v(2,2); node[11] >> v(2,3);
	node[12] >> v(3,0); node[13] >> v(3,1); node[14] >> v(3,2); node[15] >> v(3,3);
}

void operator >> (const YAML::Node& node, Eigen::Vector5d& v) {
	node[0] >> v(0,0); node[1] >> v(1,0); node[2] >> v(2,0); node[3] >> v(3,0); node[4] >> v(4,0);
}

void operator >> (const YAML::Node& node, camera_parameters& cam) {
   node["cam_id"] >> cam.cam_id;
   node["projection_matrix"] >> cam.projection_matrix;
   node["extrinsics"] >> cam.extrinsics;
   node["distortion"] >> cam.distortion_coefficients;

}



class CameraToVelodyne {
    private:

	std::string fp;
	std::vector<camera_parameters> camera_vector;    
	void read_yaml_file(std::string file_name);
	
    
    public:

	
     CameraToVelodyne(); //constructor
     Eigen::Vector3d rangexyz_to_cameraxyz(Eigen::Vector3d rangexyz, int cam_id);
     Eigen::Vector2d cameraxyz_to_camerauv(Eigen::Vector3d cameraxyz, int cam_id);
     Eigen::Vector3d camerauv_to_cameraray(Eigen::Vector2d camerauv, int cam_id);
     Eigen::Vector3d cameraray_to_rangeflatground(Eigen::Vector3d cameraray, int cam_id,double range_height);
      
	
	  
};

#endif

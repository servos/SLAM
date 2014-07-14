#include "calibration/camera_to_velodyne.h"

using namespace std;

void CameraToVelodyne::read_yaml_file(std::string file_name)
{
	      std::ifstream fin(file_name.c_str());
	      YAML::Parser parser(fin);
	      YAML::Node doc;
	      parser.GetNextDocument(doc);
	      for(unsigned i=0;i<doc.size();i++) {
		      camera_parameters cam;
		      doc[i] >> cam;
		   
		      //manually make camera matrix from projection matrix
		      cam.camera_matrix = cam.projection_matrix.block<3,3>(0,0);
			  camera_vector.push_back(cam);
	      }

	      //std::cout<<camera_vector.size()<<std::endl;

}

 CameraToVelodyne::CameraToVelodyne()
{
   std::string pkg_path = ros::package::getPath("camera_to_velodyne");
   fp = "/calibration/extrinsics.yaml";
   pkg_path.append(fp);
   std::cout<<pkg_path<<std::endl;
   read_yaml_file(pkg_path);
   
   ROS_INFO_STREAM( "Camera Extrinsic Calibration: Found calibration information for " << camera_vector.size() << " camera(s)");
   
   for(int i=0; i< camera_vector.size(); i++)
   {
		ROS_DEBUG_STREAM( "Calibration for camera " << i);
		ROS_DEBUG_STREAM( "Projection Matrix: " << endl<< camera_vector[i].projection_matrix);
		ROS_DEBUG_STREAM( "Camera Matrix: " << endl<< camera_vector[i].camera_matrix);
		ROS_DEBUG_STREAM( "Distortion Vector: " << endl<< camera_vector[i].distortion_coefficients);
		ROS_DEBUG_STREAM( "Extrinsics: " << endl<< camera_vector[i].extrinsics);
	}
	
	if(camera_vector.size() ==0)
		ROS_ERROR_STREAM("Camera Extrinsic Calibration: NO CALIBRATION FILES FOUND.  Is the exstrinics.yaml file in the right place?");
   
   
   //test stuff
  /* Eigen::Vector3d temp1(4.507,-0.6367,-0.4247);
   cout<< "mapped to "<<endl;
   Eigen::Vector3d temp2 = rangexyz_to_cameraxyz(temp1,0);
   cout<<temp2<<endl;
   Eigen::Vector2d temp3 = cameraxyz_to_camerauv(temp2,0);
   cout<< "mapped to uv "<<endl;
   cout<<temp3<<endl;
   
   Eigen::Vector2d camuv(3020,2020);
   Eigen::Vector3d cray = camerauv_to_cameraray(camuv,0);
   cout<< "camray" <<endl;
   cout <<cray<<endl;
	
	Eigen::Vector3d rpoint;
	rpoint = cameraray_to_rangeflatground(cray,0,-1.464);
	cout<< "rpoint" <<endl;
   cout <<rpoint<<endl;*/

}

Eigen::Vector3d CameraToVelodyne::rangexyz_to_cameraxyz(Eigen::Vector3d rangexyz,int cam_id)
{
	Eigen::Matrix4d tmat = camera_vector[cam_id].extrinsics;
	
	Eigen::Vector4d temp;
	Eigen::Vector4d in( rangexyz(0),rangexyz(1),rangexyz(2),1);
	temp = (tmat.inverse())*in;
	Eigen::Vector3d out(temp(0),temp(1),temp(2));
	return out;
	
}

Eigen::Vector2d CameraToVelodyne::cameraxyz_to_camerauv(Eigen::Vector3d cameraxyz, int cam_id)
{
	
	Eigen::Matrix34d pmat = camera_vector[cam_id].projection_matrix;
	Eigen::Vector3d temp;
	Eigen::Vector4d in( cameraxyz(0),cameraxyz(1),cameraxyz(2),1);
	temp = pmat*in;
	Eigen::Vector2d out(temp(0)/temp(2),temp(1)/temp(2));
	return out;
	
}

Eigen::Vector3d CameraToVelodyne::camerauv_to_cameraray(Eigen::Vector2d camerauv, int cam_id)
{
	Eigen::Vector3d uv_h(camerauv(0),camerauv(1),1); //homogenous coordinates
	
	Eigen::Matrix34d pmat = camera_vector[cam_id].projection_matrix;
	Eigen::Matrix3d pmat_li; //left inverse of projection matrix
	pmat_li(0,0) = 1.0/pmat(0,0); pmat_li(0,1) = 0; pmat_li(0,2) = -1.0*pmat_li(0,0)*pmat(0,2);
	pmat_li(1,0) = 0; pmat_li(1,1) = 1.0/pmat(1,1); pmat_li(1,2) = -1.0*pmat_li(1,1)*pmat(1,2);
	pmat_li(2,0) = 0; pmat_li(2,1) = 0; pmat_li(2,2) = 1;
	
	Eigen::Vector3d camray = pmat_li*uv_h;
	
	return camray/camray.norm(); //unit vector 
	
}


Eigen::Vector3d CameraToVelodyne::cameraray_to_rangeflatground(Eigen::Vector3d cameraray, int cam_id, double range_height)
{
	//make 3 points in range sensor frame that correspond to the ground
	Eigen::Vector3d r1(0,1,range_height);
	Eigen::Vector3d r2(0,-1,range_height);
	Eigen::Vector3d r3(-1,0,range_height);
	
	//transform these to camera frame
	Eigen::Vector3d c1 = rangexyz_to_cameraxyz(r1,cam_id);
	Eigen::Vector3d c2 = rangexyz_to_cameraxyz(r2,cam_id);
	Eigen::Vector3d c3 = rangexyz_to_cameraxyz(r3,cam_id);
	
	Eigen::Vector3d c31 = c3-c1; Eigen::Vector3d c32 = c3-c2;
	Eigen::Vector3d n = c31.cross(c32); //normal vector of plane
	
	//find scale factor for ray to intesect plane
	
	double d = (c1.dot(n) / cameraray.dot(n));
	
	//get the intersection point in camera frame
	Eigen::Vector3d cp = d*cameraray;
	Eigen::Vector4d cp_h( cp(0),cp(1),cp(2),1);
	
	//map the point back to range sensor frame
	Eigen::Matrix4d tmat = camera_vector[cam_id].extrinsics;
	Eigen::Vector4d rp_h = tmat*cp_h;
	Eigen::Vector3d rp(rp_h(0),rp_h(1),rp_h(2));
	
	return rp;
	
}



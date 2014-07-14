
#include "ccicp2d/icpTools.h"
#include "ccicp2d/icpPointToPoint.h"
#include <tf/tf.h>
#include <tf_conversions/tf_eigen.h>
#include <pcl_conversions/pcl_conversions.h>
#include <Eigen/StdVector>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <eigen_conversions/eigen_msg.h>
#include <pcl/registration/gicp.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/features/normal_3d.h>


CCICP::CCICP(RegistrationType type_): seg_target(new pcl::PointCloud<PointXYZGD>), seg_scene(new pcl::PointCloud<PointXYZGD>), 
    ground_target(new pcl::PointCloud<PointXYZGD>), ground_scene(new pcl::PointCloud<PointXYZGD>)
{
    type = type_;

    refPts_GA = (double*)calloc(2*ICP_MAX_PTS,sizeof(double));
    tarPts_GA = (double*)calloc(2*ICP_MAX_PTS,sizeof(double));
    refPts_NGA = (double*)calloc(2*ICP_MAX_PTS,sizeof(double));
    tarPts_NGA = (double*)calloc(2*ICP_MAX_PTS,sizeof(double));
    
    if(refPts_GA == NULL || tarPts_GA == NULL || refPts_NGA == NULL || tarPts_NGA == NULL)
    {
        ROS_INFO("error allocating memory for points, crashing...");
    }
    
}


void CCICP::classifyPoints(pcl::PointCloud<PointXYZGD>::Ptr outcloud)
{

    vector<vector<PointXYZGD> > ptBins; 
    ptBins.resize(NUMBINSGA*NUMBINSGA);

    //rebin into rectangular
    int nPts = outcloud->points.size();
    double offset = (double)NUMBINSGA*RESOLUTION/2;
    for(int i=0;i<nPts;i++) //binning stage
    {
        PointXYZGD pt = outcloud->points[i];
	    float curr_x = pt.x;
        float curr_y = pt.y;

        int bind_x = floor((curr_x+offset)/RESOLUTION);
        int bind_y = floor((curr_y+offset)/RESOLUTION); 
      
        if(bind_x < 0 || bind_x >= NUMBINSGA || bind_y < 0 || bind_y >= NUMBINSGA) continue;
        int hidx = (bind_x*NUMBINSGA + bind_y);
        ptBins[hidx].push_back(pt);

    }

    outcloud->clear();
     
    //Determine ground adjacentcy
    for(int i = 0; i<NUMBINSGA; i++) {
        for(int j=0; j<NUMBINSGA;j++) {

            int lin_idx = i*NUMBINSGA + j; //linear index of bin
                  
            if(ptBins[lin_idx].size() == 0) {
                continue;
            }
            
            if( i == 0 || i == NUMBINSGA-1 || j == 0 || j == NUMBINSGA-1) {
                //TODO What should I do with the edges of the grid??
                //add to cloud
                //addToCloud(lin_idx, outcloud, 0);
                continue;
            }
            
            int groundCount = 0;
            for(int q = i -1; q <= i+1; q++) {
                for( int r = j -1; r <= j+1; r++) {
                    if(q == i && r == j) {
                        continue;
                    }
                    int lidx = q*NUMBINSGA + r; //linear index of bin
                    if(ptBins[lidx].size() == 0) {
                        groundCount++;
                    }
                }
            }
            
            int num_bin_pts = ptBins[lin_idx].size();
            for(int k=0;k<num_bin_pts;k++) {
                 PointXYZGD pt;
                 pt.x = ptBins[lin_idx][k].x;
                 pt.y = ptBins[lin_idx][k].y;             
                 pt.z = ptBins[lin_idx][k].z;
                 pt.ground_adj = (groundCount >= GRD_ADJ_THRESH); 
                 outcloud->push_back(pt); //add the point to outcloud
            }
        }
    }
}

//Segment Cloud!
void CCICP::segmentGround(pcl::PointCloud<pcl::PointXYZ>::Ptr incloud, pcl::PointCloud<PointXYZGD>::Ptr outcloud, pcl::PointCloud<PointXYZGD>::Ptr ground_cloud)
{
    
    pcl::PointCloud<PointXYZGD>::Ptr drvcloud (new pcl::PointCloud<PointXYZGD>);

    outcloud -> clear();
    int nPts = incloud->points.size();

    gSeg.setupGroundSegmentation(incloud,ground_cloud,outcloud,drvcloud);
	gSeg.segmentGround();
	
    classifyPoints(outcloud);
 
}


//ICP function call with transformed parameters
geometry_msgs::PoseStamped CCICP::doICPMatch(vector<Eigen::Vector2d,Eigen::aligned_allocator<Eigen::Vector2d> >& model_GA,vector<Eigen::Vector2d,Eigen::aligned_allocator<Eigen::Vector2d> >& model_NGA, vector<Eigen::Vector2d,Eigen::aligned_allocator<Eigen::Vector2d> >& scene_GA, vector<Eigen::Vector2d,Eigen::aligned_allocator<Eigen::Vector2d> >& scene_NGA, geometry_msgs::PoseStamped& initPose)
{
		
		int numRefPts_GA = 0;
		int numTarPts_GA = 0;
		int numRefPts_NGA = 0;
		int numTarPts_NGA = 0;
		//fill in the ref points for GA
		//ROS_INFO("M_GA: %d M_NGA %d T_GA %d T_NGA %d/n", model_GA.size(),model_NGA.size(), scene_GA.size(),scene_NGA.size());
		
        //FIXME[3] this could be done in the previous loops....
        for(unsigned int i=0; i<model_GA.size(); i++)
		{
			Eigen::Vector2d pt = model_GA[i];
			refPts_GA[numRefPts_GA*2+0] = pt(0);
			refPts_GA[numRefPts_GA*2+1] = pt(1);
			numRefPts_GA++;
		}

		//fill in the ref points for NGA
		for(unsigned int i=0; i<model_NGA.size(); i++)
		{
			Eigen::Vector2d pt = model_NGA[i];
			refPts_NGA[numRefPts_NGA*2+0] = pt(0);
			refPts_NGA[numRefPts_NGA*2+1] = pt(1);
			numRefPts_NGA++;
		}
		
		//fill in the target points for GA
		for(unsigned int i=0; i<scene_GA.size(); i++)
		{
			Eigen::Vector2d pt = scene_GA[i];
			tarPts_GA[numTarPts_GA*2+0] = pt(0);
			tarPts_GA[numTarPts_GA*2+1] = pt(1);
			numTarPts_GA++;
		}
		//fill in the target points for GA
		for(unsigned int i=0; i<scene_NGA.size(); i++)
		{
			Eigen::Vector2d pt = scene_NGA[i];
			tarPts_NGA[numTarPts_NGA*2+0] = pt(0);
			tarPts_NGA[numTarPts_NGA*2+1] = pt(1);
			numTarPts_NGA++;
		}
	
		Matrix rot(2,2);
		Matrix trans(2,1);
		
		//initialize the ICP using ...
		trans.val[0][0] = initPose.pose.position.x;
		trans.val[1][0] = initPose.pose.position.y;
        double theta = tf::getYaw(initPose.pose.orientation);
		rot.val[0][0] = cos(theta); rot.val[0][1]= -sin(theta);
		rot.val[1][0] = sin(theta); rot.val[1][1]= cos(theta);

		  // check for minimum number of points
          if (numTarPts_GA+numTarPts_NGA < 5) {
            ROS_ERROR_STREAM("ERROR: Total Scene has " << numTarPts_GA+numTarPts_NGA <<" points");
            geometry_msgs::PoseStamped result;
            result.pose.orientation.w = 9999;
            return result;
          }


		IcpPointToPoint icp(refPts_GA,refPts_NGA,numRefPts_GA,numRefPts_NGA,(int32_t)(2));
		icp.fit(tarPts_GA,tarPts_NGA,numTarPts_GA,numTarPts_NGA,rot,trans,5,0);


		//if(getEw) //we want edge weight
			//icp.getEdgeWeight(eW);

		//we got the ICP match, push it out
		double corr_tx = trans.val[0][0];
		double corr_ty = trans.val[1][0];
		double corr_yaw = atan2(rot.val[1][0],rot.val[0][0]);

        geometry_msgs::PoseStamped result;

        result.pose.position.x = corr_tx;
        result.pose.position.y = corr_ty;
        result.pose.position.z = initPose.pose.position.z;
        
        double roll, pitch, yaw;
        tf::Quaternion q;
		tf::quaternionMsgToTF(initPose.pose.orientation, q);
		tf::Matrix3x3(q).getEulerYPR(yaw, pitch, roll,1); //get angles
		
		geometry_msgs::Quaternion q_msg;
		tf::quaternionTFToMsg(tf::createQuaternionFromRPY(roll,pitch,corr_yaw),q_msg);
        result.pose.orientation = q_msg;
        //result.pose.orientation = tf::createQuaternionMsgFromYaw(corr_yaw);

    	return result;
		

}



geometry_msgs::PoseStamped CCICP::doICPMatch(geometry_msgs::PoseStamped& initPose)
{
    //Segment Target Cloud given current Pose
    //crop cloud
    pcl::PointCloud<PointXYZGD>::Ptr temp(new pcl::PointCloud<PointXYZGD>);
    double curx = initPose.pose.position.x; 
    double cury = initPose.pose.position.y; 
    pcl::PassThrough<PointXYZGD> pass;
    double crop_dist = 75;
    pass.setFilterLimits (-crop_dist+curx,crop_dist+curx);
    pass.setFilterFieldName ("x");
    pass.setInputCloud (seg_target);
    pass.filter (*temp); //data is in target_obs

    pass.setFilterLimits (-crop_dist+cury,crop_dist+cury);
    pass.setFilterFieldName ("y");
    pass.setInputCloud (temp);
    pass.filter (*seg_target); //data is in  seg_target


    //Classify Points
    vector<Eigen::Vector2d,Eigen::aligned_allocator<Eigen::Vector2d> > model_GA;
    vector<Eigen::Vector2d,Eigen::aligned_allocator<Eigen::Vector2d> > model_NGA;
    vector<Eigen::Vector2d,Eigen::aligned_allocator<Eigen::Vector2d> > scene_GA;
    vector<Eigen::Vector2d,Eigen::aligned_allocator<Eigen::Vector2d> > scene_NGA;

	int numPts = seg_scene->size();
	for(int i=0; i<numPts; i++)
	{	
		Eigen::Vector2d sPt;
		sPt(0) = seg_scene->points[i].x; sPt(1) = seg_scene->points[i].y;	
		
		if(isGA(seg_scene->points[i].ground_adj)) { //it is ground adjacents
            if(scene_GA.size() >=  ICP_MAX_PTS-1) continue;
			scene_GA.push_back(sPt);
        } else {
            if(scene_NGA.size() >=  ICP_MAX_PTS-1) continue;
			scene_NGA.push_back(sPt);
        }
	}

    numPts = seg_target->size();
	for(int i=0; i<numPts; i++)
	{	
		Eigen::Vector2d sPt;
		sPt(0) = seg_target->points[i].x; sPt(1) = seg_target->points[i].y;	
		
		if(isGA(seg_target->points[i].ground_adj)) { //it is ground adjacents
            if(model_GA.size() >=  ICP_MAX_PTS-1) continue;
			model_GA.push_back(sPt);
        } else {
            if(model_NGA.size() >=  ICP_MAX_PTS-1) continue;
			model_NGA.push_back(sPt);
        }
	}

	geometry_msgs::PoseStamped result_2d;
	geometry_msgs::PoseStamped result_3d;

    //doICP
    result_2d = doICPMatch(model_GA, model_NGA, scene_GA, scene_NGA, initPose);  // this does the 2D ICP
    
    //do height reg
    pcl::PointCloud<pcl::PointXYZ>::Ptr ground_target_xyz (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr ground_scene_xyz (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::copyPointCloud(*ground_target,*ground_target_xyz);	
    pcl::copyPointCloud(*ground_scene,*ground_scene_xyz);	
    
    //result_3d = doRPHRegistration(ground_target_xyz,ground_scene_xyz,result_2d);
    //result_3d = doHeightOnlyReg(ground_target_xyz,ground_scene_xyz,result_2d);
    result_3d = doHeightInterpolate(ground_target_xyz,result_2d);
    
    
    return result_3d;
    
    
}


geometry_msgs::PoseStamped CCICP::doHeightInterpolate(pcl::PointCloud<pcl::PointXYZ>::Ptr ground, geometry_msgs::PoseStamped& initPose)
{
    double ROBO_HEIGHT = 1.45; //FIXME: magic number
    double x_wheel_dist = 0.5; //distance in x from velodyne centre to wheel
    double y_wheel_dist = 0.5; //distance in y from velodyne centre to wheel;
    pcl::PointCloud<pcl::PointXYZ>::Ptr robot_cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr robot_cloud_trans (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr corr_ground_cloud (new pcl::PointCloud<pcl::PointXYZ>);
    geometry_msgs::PoseStamped output = initPose; //passthrough most values, will change height in function
    
    for(int i =-1; i<=1; i+=2)
    {
		 for(int j =-1; j<=1; j+=2)
		 {
			   pcl::PointXYZ wheelpt(i*x_wheel_dist,j*y_wheel_dist,-1.0*ROBO_HEIGHT);
			   robot_cloud->points.push_back(wheelpt);
		 }
	 }
    
    //got robot cloud in robot frame, next transform into global frame
    tf::Quaternion q_robot;
    tf::quaternionMsgToTF(initPose.pose.orientation, q_robot);   
    Eigen::Matrix4f transformation = Eigen::Matrix4f::Identity();
    tf::Matrix3x3 matq(q_robot);
    transformation << matq[0][0], matq[0][1], matq[0][2], initPose.pose.position.x,
                     matq[1][0], matq[1][1], matq[1][2], initPose.pose.position.y,
                     matq[2][0], matq[2][1], matq[2][2], initPose.pose.position.z,
                     0,0,0,1;
                     
     //transform the pointcloud
      pcl::transformPointCloud(*robot_cloud,*robot_cloud_trans, transformation); //transform cloud
      
      //now find the nearest neighbour points in the ground cloud
      pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
      kdtree.setInputCloud(ground);
      int numPts = robot_cloud_trans->size();
      std::vector<int> pointIdx(1);
	  std::vector<float> pointSquaredDist(1);
	  
	 
      
      for(int i=0; i<numPts; i++)
      {
		pcl::PointXYZ qpt = robot_cloud_trans->points[i];
		kdtree.nearestKSearch(qpt, 1, pointIdx, pointSquaredDist);
		pcl::PointXYZ cpt = ground->points[pointIdx[0]];
		if(pointSquaredDist[0] < 9) //FIXME: should be a param
		{
			corr_ground_cloud->points.push_back(cpt); //collect all the corresponding ground points
		}
	  }
	  
	  int numCorrPts = corr_ground_cloud->size(); //number of valid correspondences in ground cloud
	  
	   //td::cout << "num_pts " << numCorrPts <<std::endl;
	  
	  if(numCorrPts>=4) //make sure we have 4 points
	  {
		  //get the average height
		  Eigen::Vector3f ground_sum = Eigen::Vector3f::Zero();
		  for(int i=0; i<numCorrPts; i++)
		  {
			  ground_sum+=corr_ground_cloud->points[i].getVector3fMap();
		  }
		  Eigen::Vector3f average_ground_point =  ground_sum/(double)numCorrPts;
		  
		  pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
		  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
		  static const int arr[] = {0,1,2,3}; //use all points
		  vector<int> vec (arr, arr + sizeof(arr) / sizeof(arr[0]) );
		  float nx,ny,nz,cv;
		  ne.computePointNormal(*corr_ground_cloud,vec,nx,ny,nz,cv);
		  //std::cout << "plane normal is: " << nx << ", " << ny << ", "<< nz<<","<<std::endl;
		   
		   //check nz value
          if( !isnan(nz) && !isnan(ny) && !isnan(nz) ) {
            
		    if(nz<0) // change all normal values
		    {
			    nz=nz*(-1.0); nx=nx*(-1.0); ny=ny*(-1.0);
		    }
		   
		    float dx = nx*ROBO_HEIGHT+average_ground_point(0); 
            float dy = ny*ROBO_HEIGHT+average_ground_point(1); 
            float dz = nz*ROBO_HEIGHT+average_ground_point(2); //ground point shifted by robot height 
		 
		    //calculate the scan interpolation orientation
		    //Eigen::Vector
		  
		  
		    //set the new height value
		    output.pose.position.z = dz;
          }
	   } else { 
          ROS_WARN_STREAM("Height could not be determined"); 
       }
	  
	   
	   
	   
    //find NN
    /* pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
     kdtree.setInputCloud(ground);

    pcl::PointXYZ pt;
    pt.x = initPose.pose.position.x;
    pt.y = initPose.pose.position.y;
    pt.z = initPose.pose.position.z-ROBO_HEIGHT;

    std::vector<int> pointIdx(1);
    std::vector<float> pointSquaredDist(1);
    kdtree.nearestKSearch(pt, 1, pointIdx, pointSquaredDist);

     output.pose.position.z = ground->points[pointIdx[0]].z+ROBO_HEIGHT; //lowpass filter //FIXME magic number*/

     return output;


}


geometry_msgs::PoseStamped CCICP::doHeightOnlyReg(pcl::PointCloud<pcl::PointXYZ>::Ptr target, pcl::PointCloud<pcl::PointXYZ>::Ptr scene, geometry_msgs::PoseStamped& initPose)
{
    geometry_msgs::PoseStamped output = initPose; 
    pcl::PointCloud<pcl::PointXYZ>::Ptr scene_trans (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr scene_filt (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr temp (new pcl::PointCloud<pcl::PointXYZ>);


    //crop ground
    pcl::PassThrough<pcl::PointXYZ> pass;
   
    pass.setFilterLimits (-5,5);
    pass.setFilterFieldName ("x");
    pass.setInputCloud (scene);
    pass.filter (*temp); //data is in temp

	pass.setFilterLimits (-5,5);
    pass.setFilterFieldName ("y");
    pass.setInputCloud (temp);
    pass.filter (*scene_filt); //data is in target_crop


    //transform scene to target frame
	Eigen::Affine3d trans;
    geometry_msgs::Pose cpose;
    double roll, pitch, yaw;
    tf::Quaternion q;
    tf::quaternionMsgToTF(initPose.pose.orientation, q);
    tf::Matrix3x3(q).getEulerYPR(yaw, pitch, roll,1); //get angles
    cpose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0,0,yaw); //remove roll/pitch
    cpose.position.x = initPose.pose.position.x;
    cpose.position.y = initPose.pose.position.y;
    tf::poseMsgToEigen(cpose, trans);
    pcl::transformPointCloud(*scene_filt,*scene_trans, trans); //transform scene using initPose
    

    //find NN
     pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
     kdtree.setInputCloud(target);


     double error = 0;
     for(int i = 0; i < scene_trans->size(); i++) {
         pcl::PointXYZ& pt = scene_trans->points[0];
         std::vector<int> pointIdx(1);
         std::vector<float> pointSquaredDist(1);
         kdtree.nearestKSearch(pt, 1, pointIdx, pointSquaredDist);

         error += target->points[pointIdx[0]].z - pt.z;
     }

     error /= static_cast<double>(scene_trans->size());

     output.pose.position.z += error/10; //lowpass filter //FIXME magic number

     return output;

}

geometry_msgs::PoseStamped CCICP::doRPHRegistration(pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr scene_cloud,geometry_msgs::PoseStamped& initPose)
{
	
	double vf_xy = 1; double vf_z = 5;
	double curr_x = initPose.pose.position.x; double curr_y = initPose.pose.position.y;
	double crop_dist = 5;
	//debug info
	
	
	double start_time =ros::Time::now().toSec();
	
	
	//first transform the scene cloud into target using initPose
	pcl::PointCloud<pcl::PointXYZ>::Ptr trans_scene_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr target_crop(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr scene_crop(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr temp(new pcl::PointCloud<pcl::PointXYZ>);
	
    //passthrough filters for xy for both scene and target
    pcl::PassThrough<pcl::PointXYZ> pass;
   
    pass.setFilterLimits (-crop_dist,crop_dist);
    pass.setFilterFieldName ("x");
    pass.setInputCloud (scene_cloud);
    pass.filter (*temp); //data is in temp

	pass.setFilterLimits (-crop_dist,crop_dist);
    pass.setFilterFieldName ("y");
    pass.setInputCloud (temp);
    pass.filter (*scene_crop); //data is in scene_crop


	Eigen::Affine3d trans;
    double roll, pitch, yaw;
    geometry_msgs::Pose cpose;
    tf::Quaternion q;
    tf::quaternionMsgToTF(initPose.pose.orientation, q);
    tf::Matrix3x3(q).getEulerYPR(yaw, pitch, roll,1); //get angles
    cpose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0,0,yaw); //remove roll and pitch
    cpose.position.x = initPose.pose.position.x;
    cpose.position.y = initPose.pose.position.y;
    tf::poseMsgToEigen(cpose, trans);
    pcl::transformPointCloud(*scene_crop,*trans_scene_cloud, trans); //transform scene using initPose
    

    
    //now it's registration time!
    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> reg;
    pcl::PointCloud<pcl::PointXYZ>::Ptr final_ground_trans(new pcl::PointCloud<pcl::PointXYZ>);
    reg.setInputSource(trans_scene_cloud);
    reg.setInputTarget(target_cloud);
    reg.setRANSACIterations(10);
    reg.setMaxCorrespondenceDistance (1);
    //reg.setCorrespondenceRandomness(16);
    //reg.setMaximumOptimizerIterations(5);
    reg.align(*final_ground_trans);
    Eigen::Matrix4f final_trans;
    final_trans = reg.getFinalTransformation();
    
    double end_time =ros::Time::now().toSec();
    
    
    geometry_msgs::PoseStamped result;
    
   //get the initial pitch and roll from the initPoseme 
   tf::Quaternion q_init;
   tf::quaternionMsgToTF(initPose.pose.orientation, q_init); //rotation that includes initial roll, pitch and yaw.

   double rx = atan2f(final_trans(2,1), final_trans(2,2)); //delta roll
   double ry = asinf(-final_trans(2,0)); //delta pitch
   double rz = 0;//tf::getYaw(initPose.pose.orientation);
   
   geometry_msgs::Quaternion q_msg;
   tf::Quaternion q_rp = tf::createQuaternionFromRPY(rx,ry,rz); //these are the deltas
   tf::Quaternion q_total = q_init*q_rp; //total rotation from robot frame to inertial frame
   
   tf::quaternionTFToMsg(q_total,q_msg); //quaternion message
   
   result.pose.position.x = initPose.pose.position.x; // translation x (from ccicp init pose)
   result.pose.position.y = initPose.pose.position.y; //translation y (from ccicp init pose)
   result.pose.position.z = initPose.pose.position.z+final_trans(2,3); //translation z(from 3D ICP z)

   //cout << "finaltrans\n" << final_trans << endl;
   
   result.pose.orientation = q_msg;	 //set the result message
   
   return result;
    
	
}

//Main function call to run ICP
geometry_msgs::PoseStamped CCICP::doICPMatch(pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr scene_cloud, geometry_msgs::PoseStamped& initPose, RegistrationType type)
{

    setTargetCloud(target_cloud, initPose);
    setSceneCloud(scene_cloud);

    return doICPMatch(initPose);
}

void CCICP::setTargetGndCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr target)
{
    copyPointCloud(*target,*ground_target);
}

void CCICP::setTargetCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr target,geometry_msgs::PoseStamped& initPose) 
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr target_obs (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr temp (new pcl::PointCloud<pcl::PointXYZ>);

    if(type == SCAN_TO_MAP) {

        copyPointCloud(*target, *seg_target); 
        classifyPoints(seg_target);

        
    } else {
        //Segment Scene Ground
        pcl::PointCloud<PointXYZGD>::Ptr target_ground (new pcl::PointCloud<PointXYZGD>);
        pcl::PointCloud<PointXYZGD>::Ptr target_obstacle (new pcl::PointCloud<PointXYZGD>);
        segmentGround(target, target_obstacle,target_ground);

        *seg_target = *target_obstacle;
        *ground_target = *target_ground;

    }
}


void CCICP::setSceneCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr scene) 
{
    pcl::PointCloud<PointXYZGD>::Ptr scene_obs (new pcl::PointCloud<PointXYZGD>);
    pcl::PointCloud<PointXYZGD>::Ptr scene_ground (new pcl::PointCloud<PointXYZGD>);
    //Segment Scene Ground

    segmentGround(scene, scene_obs,scene_ground);

    //voxel filter map
    pcl::PointCloud<PointXYZGD>::Ptr temp(new pcl::PointCloud<PointXYZGD>); 
    pcl::VoxelGrid<PointXYZGD> sor;
    sor.setInputCloud(scene_obs);
    sor.setLeafSize (0.5, 0.5, 2); //FIXME: parameterize! 
    sor.filter (*temp); //data is in temp*/
    
    *seg_scene = *temp;
    
    //filter for ground
    sor.setInputCloud(scene_ground);
    sor.setLeafSize (0.5, 0.5, 5); //FIXME: parameterize! 
    sor.filter (*temp); //data is in temp*/

    *ground_scene = *temp;
}


double CCICP::getResidual() {
    //TODO: calculate this somehow
    //
    return -1;
}


void CCICP::getSegmentedClouds(pcl::PointCloud<pcl::PointXYZ>& target, pcl::PointCloud<pcl::PointXYZ>& scene, pcl::PointCloud<pcl::PointXYZ>& g_target, pcl::PointCloud<pcl::PointXYZ>& g_scene)
{
   copyPointCloud(*seg_target, target); 
   copyPointCloud(*seg_scene, scene); 
   copyPointCloud(*ground_target, g_target); 
   copyPointCloud(*ground_scene, g_scene); 
}

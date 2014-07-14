#include "graphSlamTools.h"
#include <ros/ros.h>

#include <tf/tf.h>

#include "ccicp2d/icpTools.h"
#include <tf/tf.h>
#include <eigen_conversions/eigen_msg.h>

#include <pcl/registration/gicp.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>

CCICP icp(SCAN_TO_SCAN);


pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> gicp;

struct sortContainer
{
	double distance;
	int idx;

};


void setup_gicp() {
    // Set the max correspondence distance 
    gicp.setMaxCorrespondenceDistance (0.75);
    // Set the maximum number of iterations (criterion 1)
    gicp.setMaximumIterations (200);
    // Set the transformation epsilon (criterion 2)
    gicp.setTransformationEpsilon (1e-6);
    // Set the euclidean distance difference epsilon (criterion 3)
    gicp.setEuclideanFitnessEpsilon (1e-6);
    gicp.setRANSACIterations(10);
    //gicp.setCorrespondenceRandomness (30);

}



double graphSlamGetNearestKF(PoseGraph& pG, Node& gN)
{
	int numKF = pG.nodes.size();
	int bestKF;
	double smallestDist = 1e20;
	double dx = 0;
	double dy=0;
	
	for(int i=0; i<numKF; i++)
	{
		Node currNode = pG.nodes[i];
		dx = currNode.pose.pose.position.x - gN.pose.pose.position.x; dy = currNode.pose.pose.position.y - gN.pose.pose.position.y;
		double currDist = sqrt( dx*dx + dy*dy);
		
		if(currDist<smallestDist && gN.idx!=currNode.idx) 
		{
			smallestDist = currDist;
			bestKF = i;
		}
	}
	
	return smallestDist;
}

bool distSort (sortContainer p1,sortContainer p2) 
{ 
    return (p1.distance < p2.distance); 
}

vector<int> graphSlamGetKNN(PoseGraph& pG, Node& gN, int K)
{

	vector<int> toReturn;
	vector<sortContainer> sCVector;
	int numKF = pG.nodes.size();
	
	double dx = 0;
	double dy=0;
    
    K = min(numKF-1,K);

	//grab the KNN
	for(int i=0; i<numKF-2; i++)
	{
		sortContainer sC;
		Node currNode = pG.nodes[i];
		dx = currNode.pose.pose.position.x - gN.pose.pose.position.x; dy = currNode.pose.pose.position.y - gN.pose.pose.position.y;
		double currDist = sqrt( dx*dx + dy*dy);
		sC.distance = currDist;
		sC.idx = i;
		sCVector.push_back(sC);
	}
	
	sort(sCVector.begin(),sCVector.end(),distSort);

	for(int i=0; i<(K);i++)
	{
		if(i < sCVector.size() && gN.idx!=sCVector[i].idx)	{	
			toReturn.push_back(sCVector[i].idx);
	    }
	}
	
	return toReturn;
}

Eigen::Matrix6d computeEdgeInformationLUM(pcl::PointCloud<pcl::PointXYZ>::Ptr& source_trans, pcl::PointCloud<pcl::PointXYZ>::Ptr& target, double max_corr_distance)
{
	//this function assumes source_could is transformed by the pose provided by the scan reg algo
	
   int numSourcePts = source_trans->size();
   std::vector <Eigen::Vector3f> corrs_aver (numSourcePts);
   std::vector <Eigen::Vector3f> corrs_diff (numSourcePts);
   int numCorr = 0;
   
   Eigen::Matrix6d edgeCov = Eigen::Matrix6d::Identity ();
   
   //build kd tree for source points
  pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
  kdtree.setInputCloud(target);
  
  //iterate through the source cloud and compute match covariance
  
  for(int i=0; i<numSourcePts; i++)
  {
	  pcl::PointXYZ qpt = source_trans->points[i];
	  vector<int> nn_idx;
	  vector<float> nn_sqr_dist;
	  kdtree.nearestKSearch(qpt,1,nn_idx,nn_sqr_dist); //returns the index of the nn point in the target
	  
	if(nn_sqr_dist[0]<max_corr_distance*max_corr_distance) //if the distance to point is less than max correspondence distance, use it to calculate  
	{
		Eigen::Vector3f source_pt = qpt.getVector3fMap();
		Eigen::Vector3f target_pt = target->points[nn_idx[0]].getVector3fMap();
		
		// Compute the point pair average and difference and store for later use
		corrs_aver[numCorr] = 0.5 * (source_pt + target_pt);
		corrs_diff[numCorr] = source_pt - target_pt;
		numCorr++;
	}
	else
	{
		continue;
	}
   }
	corrs_aver.resize (numCorr);
	corrs_diff.resize (numCorr);
	
	//now compute the M matrix
	Eigen::Matrix6d MM = Eigen::Matrix6d::Zero ();
	Eigen::Vector6d MZ = Eigen::Vector6d::Zero ();
	for (int ci = 0; ci != numCorr; ++ci)  // ci = correspondence iterator
	{
		// Fast computation of summation elements of M'M
		MM (0, 4) -= corrs_aver[ci] (1);
		MM (0, 5) += corrs_aver[ci] (2);
		MM (1, 3) -= corrs_aver[ci] (2);
		MM (1, 4) += corrs_aver[ci] (0);
		MM (2, 3) += corrs_aver[ci] (1);
		MM (2, 5) -= corrs_aver[ci] (0);
		MM (3, 4) -= corrs_aver[ci] (0) * corrs_aver[ci] (2);
		MM (3, 5) -= corrs_aver[ci] (0) * corrs_aver[ci] (1);
		MM (4, 5) -= corrs_aver[ci] (1) * corrs_aver[ci] (2);
		MM (3, 3) += corrs_aver[ci] (1) * corrs_aver[ci] (1) + corrs_aver[ci] (2) * corrs_aver[ci] (2);
		MM (4, 4) += corrs_aver[ci] (0) * corrs_aver[ci] (0) + corrs_aver[ci] (1) * corrs_aver[ci] (1);
		MM (5, 5) += corrs_aver[ci] (0) * corrs_aver[ci] (0) + corrs_aver[ci] (2) * corrs_aver[ci] (2);
		
		// Fast computation of M'Z
		MZ (0) += corrs_diff[ci] (0);
		MZ (1) += corrs_diff[ci] (1);
		MZ (2) += corrs_diff[ci] (2);
		MZ (3) += corrs_aver[ci] (1) * corrs_diff[ci] (2) - corrs_aver[ci] (2) * corrs_diff[ci] (1);
		MZ (4) += corrs_aver[ci] (0) * corrs_diff[ci] (1) - corrs_aver[ci] (1) * corrs_diff[ci] (0);
		MZ (5) += corrs_aver[ci] (2) * corrs_diff[ci] (0) - corrs_aver[ci] (0) * corrs_diff[ci] (2);
    
	}
	// Remaining elements of M'M
	MM (0, 0) = MM (1, 1) = MM (2, 2) = static_cast<float> (numCorr);
	MM (4, 0) = MM (0, 4);
	MM (5, 0) = MM (0, 5);
	MM (3, 1) = MM (1, 3);
	MM (4, 1) = MM (1, 4);
	MM (3, 2) = MM (2, 3);
	MM (5, 2) = MM (2, 5);
	MM (4, 3) = MM (3, 4);
	MM (5, 3) = MM (3, 5);
	MM (5, 4) = MM (4, 5);
	
	// Compute pose difference estimation
	Eigen::Vector6d D = static_cast<Eigen::Vector6d> (MM.inverse () * MZ);

	// Compute s^2
	float ss = 0.0f;
	for (int ci = 0; ci != numCorr; ++ci)  // ci = correspondence iterator
	{
		ss += static_cast<float> (pow (corrs_diff[ci] (0) - (D (0) + corrs_aver[ci] (2) * D (5) - corrs_aver[ci] (1) * D (4)), 2.0f)
                            + pow (corrs_diff[ci] (1) - (D (1) + corrs_aver[ci] (0) * D (4) - corrs_aver[ci] (2) * D (3)), 2.0f)
                            + pow (corrs_diff[ci] (2) - (D (2) + corrs_aver[ci] (1) * D (3) - corrs_aver[ci] (0) * D (5)), 2.0f));
     }

  // When reaching the limitations of computation due to linearization
  if (ss < 0.0000000000001 || !pcl_isfinite (ss))
  {
    cout<< "Warning: Edge Covariance is singular or illdefined" <<endl;
    edgeCov = Eigen::Matrix6d::Identity();
    return edgeCov;
  }

  // Store the results in the slam graph
	edgeCov = MM * (1.0f / ss);
  	
  	return edgeCov; 
 }
  


bool calcEdgeIcp(Edge& gE, PoseGraph& pG) 
{
    geometry_msgs::PoseStamped edge;
    geometry_msgs::PoseStamped initPose;
    
    Node& nodeTo = pG.nodes[gE.to];
    Node& nodeFrom = pG.nodes[gE.from];
    
    /*float dx = (nodeTo.pose.pose.position.x - nodeFrom.pose.pose.position.x);
    float dy = (nodeTo.pose.pose.position.y - nodeFrom.pose.pose.position.y);
    float dz = (nodeTo.pose.pose.position.z - nodeFrom.pose.pose.position.z);

    initPose.pose.position.x = dx*cos(tf::getYaw(nodeFrom.pose.pose.orientation))+dy*sin(tf::getYaw(nodeFrom.pose.pose.orientation));
    initPose.pose.position.y = -dx*sin(tf::getYaw(nodeFrom.pose.pose.orientation))+dy*cos(tf::getYaw(nodeFrom.pose.pose.orientation));
    initPose.pose.position.z = dz;

    tf::Quaternion qto, qfrom;
    tf::quaternionMsgToTF(nodeTo.pose.pose.orientation, qto);   
    tf::quaternionMsgToTF(nodeFrom.pose.pose.orientation, qfrom);   
    tf::Quaternion q = qfrom.inverse()*qto;
    tf::quaternionTFToMsg(q,initPose.pose.orientation) ; //FIXME check this
    tf::Matrix3x3 matq(q);*/
    //start debug
    Eigen::Affine3d Tto,Tfrom;
    tf::poseMsgToEigen (nodeTo.pose.pose, Tto);
    tf::poseMsgToEigen (nodeFrom.pose.pose, Tfrom);
    
    Eigen::Matrix4d Mto = Tto.matrix(); 
    Eigen::Matrix4d Mfrom = Tfrom.matrix(); 
        
    // GICP
    Eigen::Matrix4f transformation = Eigen::Matrix4f::Identity();
    /*transformation << matq[0][0], matq[0][1], matq[0][2], initPose.pose.position.x,
                     matq[1][0], matq[1][1], matq[1][2], initPose.pose.position.y,
                     matq[2][0], matq[2][1], matq[2][2], initPose.pose.position.z,
                     0,0,0,1;*/
    
    //cout<< "quat init :" << endl<< transformation <<endl;
    //cout<< "trans init :" <<endl<<Mfrom.inverse()*Mto <<endl;
    
    transformation = (Mfrom.inverse()*Mto).cast<float>();
    Eigen::Affine3d initialization_pose(transformation.cast<double>());
    tf::poseEigenToMsg (initialization_pose, initPose.pose);
    //end debug
    
    //Scene compensated for RPH
    pcl::PointCloud<pcl::PointXYZ>::Ptr temp(new pcl::PointCloud<pcl::PointXYZ>); 
/*	Eigen::Affine3d trans;
    double roll, pitch, yaw;
    geometry_msgs::Pose cpose;
    tf::Matrix3x3(q).getEulerYPR(yaw, pitch, roll,1); //get angles
    cpose.orientation = tf::createQuaternionMsgFromRollPitchYaw(roll,pitch,0); //comp roll and pitch
    cpose.position.z = initPose.pose.position.z;
    tf::poseMsgToEigen(cpose, trans);
    pcl::transformPointCloud(*nodeTo.keyframe,*temp, trans); //transform cloud

    edge = icp.doICPMatch(nodeFrom.keyframe, temp, initPose);
    
*/
        

    pcl::PointCloud<pcl::PointXYZ>::Ptr from_cld(new pcl::PointCloud<pcl::PointXYZ>); 
    pcl::PointCloud<pcl::PointXYZ>::Ptr to_cld(new pcl::PointCloud<pcl::PointXYZ>); 
    pcl::VoxelGrid<pcl::PointXYZ> sor;
    sor.setLeafSize (0.5, 0.5, 0.5);
    sor.setInputCloud(nodeFrom.keyframe);
    sor.filter (*from_cld); //data is in temp
    sor.setInputCloud(nodeTo.keyframe);
    sor.filter (*to_cld); //data is in temp
    
    //use reciprocal correspondences?
    //gicp.setUseReciprocalCorrespondences(true);

    gicp.setInputSource(to_cld);
    gicp.setInputTarget(from_cld);
    double start_time = ros::Time::now().toSec();
    
    gicp.align(*temp, transformation);
    transformation =  gicp.getFinalTransformation();
    
    //cout<< "icp trans :" << endl<< transformation <<endl;
    
    pcl::transformPointCloud(*to_cld,*temp, transformation); //transform cloud
    Eigen::Matrix6d edge_information;
    edge_information = computeEdgeInformationLUM(temp,from_cld,0.75); //the max here should be the actual icp dmax
    
     //Eigen::Matrix6d edge_covariance = edge_information.inverse();
     
     //cout<< "edge norm is: " << endl<<edge_covariance.norm()<< endl;
    
    //ncout<<endl<<edge_information<<endl;
    
    double end_time = ros::Time::now().toSec();
    
    //cout<<"reg time: " << (end_time - start_time)*1000 <<endl;

    //float score = gicp.getFitnessScore();
    //std::cout << " Score: " << score << std::endl;
    
    edge.pose.position.x = static_cast<double>(transformation(0,3));
    edge.pose.position.y = static_cast<double>(transformation(1,3));
    edge.pose.position.z = static_cast<double>(transformation(2,3));

    tf::Matrix3x3 tf3d;
    tf3d.setValue(static_cast<double>(transformation(0,0)), static_cast<double>(transformation(0,1)), static_cast<double>(transformation(0,2)), 
        static_cast<double>(transformation(1,0)), static_cast<double>(transformation(1,1)), static_cast<double>(transformation(1,2)), 
        static_cast<double>(transformation(2,0)), static_cast<double>(transformation(2,1)), static_cast<double>(transformation(2,2)));

    
    tf::Quaternion qfin;
    tf3d.getRotation(qfin);
    tf::quaternionTFToMsg(qfin,edge.pose.orientation);     
        

        
    double x_diff = abs(initPose.pose.position.x -  edge.pose.position.x);
    double y_diff = abs(initPose.pose.position.y -  edge.pose.position.y);
    double theta_diff = abs(tf::getYaw(initPose.pose.orientation) -  tf::getYaw(edge.pose.orientation));
    
    if(theta_diff > 2*M_PI)
        theta_diff = theta_diff-2*M_PI;
    else if(theta_diff > M_PI)
        theta_diff = 2*M_PI-theta_diff;

    ROS_DEBUG_STREAM("edge pose: " << edge.pose.position.x << ", " << edge.pose.position.y);
    
    gE.edge = edge;
    //cout<<"edge h: " <<edge.pose.position.z <<endl;
    

    //FIXME: this should be an actual covarience
    //gE.edgeInf  = Eigen::MatrixXd::Identity(6,6); 
    gE.edgeInf  = edge_information; 

   //check if it's a bad match based on dist thresh

   if( (x_diff >  DIST_MOVE_THRESH) || (y_diff > DIST_MOVE_THRESH) || (theta_diff> ROT_MOVE_THRESH) )//reject
   {
	ROS_WARN_STREAM("failed ICP match!");
	return false;
   }
   else
	return true;

   //gE.edgeCov <<eW[0],eW[1],eW[2],eW[3],eW[4],eW[5],eW[6],eW[7],eW[8];
    
}


void graphSlamAddEdgeToPrevious(PoseGraph& pG)
{
    //FIXME: this does nothing.....
/*	unsigned int to = pG.nodes.size()-1;
	unsigned int from = to -1;

	Edge gE;
	gE.from = from;
	gE.to = to;
	gE.ctype = CTYPE_ODOM;
	
	pG.edges.push_back(gE);*/
}

void graphSlamAddEdgeToHome(PoseGraph& pG)
{

	unsigned int to = pG.nodes.size()-1;
	unsigned int from = 0;
	
	if(pG.nodes.size()>=2)
	{

		Edge gE;
		gE.from = from;
		gE.to = to;
	    gE.ctype = CTYPE_HOME;
	
	    //do icp
	   bool isGoodMatch;
	   isGoodMatch= calcEdgeIcp(gE,pG);
	   if(isGoodMatch)
	   {
	       pG.edges.push_back(gE);
	   }
	}
	
}

bool graphSlamAddEdgeToX(PoseGraph& pG, int from, int to)
{
	bool isGoodMatch = false;
	if(pG.nodes.size()>=2)
	{

		Edge gE;
		gE.from = from;
		gE.to = to;
	    gE.ctype = CTYPE_NN;
	
	    //do icp
	    isGoodMatch= calcEdgeIcp(gE,pG);
	   if(isGoodMatch)
	   {
           pG.edges.push_back(gE);
	   }
	}
    return isGoodMatch;
}

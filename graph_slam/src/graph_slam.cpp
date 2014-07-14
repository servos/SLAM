
#include <ros/ros.h>
#include <iostream>
#include <map>
#include <vector>

//g2o headers
#include "g2o/types/slam3d/vertex_se3.h"
#include "g2o/types/slam3d/edge_se3.h"
#include "g2o/types/slam3d/se3quat.h"
#include "g2o/core/sparse_optimizer.h"
#include "g2o/core/block_solver.h"
#include "g2o/core/factory.h"
#include "g2o/core/optimization_algorithm_factory.h"
#include "g2o/core/optimization_algorithm_gauss_newton.h"
#include "g2o/core/optimization_algorithm_levenberg.h"
#include "g2o/solvers/csparse/linear_solver_csparse.h"

//ros/pcl headers
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include "tf/tf.h"
#include <eigen_conversions/eigen_msg.h>


//our libraries
#include "mls/mls.h"

//local headers
#include <graph_slam/Edge.h>
#include "global_mapping_viz.h"
#include "graph_slam.h"
#include "graphSlamTools.h"
#include "ccicp2d/icpTools.h"
#include "nasa_msgs/MapAction.h"
#include <actionlib/server/simple_action_server.h>


using namespace std;
using namespace g2o;

typedef BlockSolver< BlockSolverTraits<-1, -1> >  SlamBlockSolver;
typedef LinearSolverCSparse<SlamBlockSolver::PoseMatrixType> SlamLinearSolver;

SparseOptimizer optimizer;
SlamLinearSolver* linearSolver = new SlamLinearSolver();
SlamBlockSolver* blockSolver = new SlamBlockSolver(linearSolver);
OptimizationAlgorithmLevenberg* solver = new OptimizationAlgorithmLevenberg(blockSolver);

//ROS PUBLISHERS
ros::Publisher vizPub;
ros::Publisher mlsPub;
ros::Publisher posePub;
ros::Publisher globalCloudPub;
ros::Publisher obstacleCloudPub;
ros::Publisher groundCloudPub; 
ros::Publisher drivablilityPub;

//GLOBALS
bool new_info = 0;
int vertex_count = 1;
double start_time = 0;
double loop_time = 0;

PoseGraph pG;

pcl::PointCloud<pcl::PointXYZ>::Ptr current_cloud (new pcl::PointCloud<pcl::PointXYZ>);

//MAPPING
MLS globalMap(1000, 1000, 0.5, false, 1.45); 
//(cells in X, cells in Y, resolution, rolling?, robot height)

geometry_msgs::PoseStamped curPose;
bool got_pose = false;


//Action Server
//ACTION SERVER CALLBACKS
enum {
    FSMINIT = 1,
    FSMHOME = 2,
    FSMFIX = 3
};
actionlib::SimpleActionServer<nasa_msgs::MapAction>* as;
void goalCallback()
{
	nasa_msgs::MapGoal mg;
	mg = *as->acceptNewGoal();
	nasa_msgs::MapResult homeResult;
	
	int command = mg.goal;
	
    geometry_msgs::PoseStamped home_pose;

	homeResult.home_pose = home_pose;

	if(command==FSMINIT) //init phase
	{
	}
	else if (command==FSMHOME) 
	{
        //add initial cloud to map
        Eigen::Affine3d trans;
        pcl::PointCloud<pcl::PointXYZ>::Ptr trans_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        tf::poseMsgToEigen(pG.nodes[0].pose.pose, trans);
        pcl::transformPointCloud(*pG.nodes[0].keyframe,*trans_cloud, trans);

        globalMap.clearMap();
        globalMap.setMinClusterPoints(3); //use fewer point to initalize to get more points for matching
        globalMap.addToMap(trans_cloud, curPose);
        
        //publish initial cloud
      globalMap.visualize(mlsPub, "/global");
      ros::Time time = ros::Time::now();
      pcl::PointCloud<pcl::PointXYZ>::Ptr global_cloud;
      pcl::PointCloud<pcl::PointXYZ>::Ptr obstacle_cloud;
      pcl::PointCloud<pcl::PointXYZ>::Ptr ground_cloud;
      sensor_msgs::PointCloud2 cloud_msg;

      global_cloud = globalMap.getGlobalCloud();
      globalMap.getSegmentedClouds(obstacle_cloud, ground_cloud);

      pcl::toROSMsg(*global_cloud,cloud_msg);
      cloud_msg.header.frame_id = "/global";
      cloud_msg.header.stamp = time;
      globalCloudPub.publish(cloud_msg);

      pcl::toROSMsg(*obstacle_cloud,cloud_msg);
      cloud_msg.header.frame_id = "/global";
      cloud_msg.header.stamp = time;
      obstacleCloudPub.publish(cloud_msg);

      pcl::toROSMsg(*ground_cloud,cloud_msg);
      cloud_msg.header.frame_id = "/global";
      cloud_msg.header.stamp = time;
      groundCloudPub.publish(cloud_msg);

      nav_msgs::OccupancyGrid::Ptr drivability;
      drivability = globalMap.getDrivability();
      drivability->header.frame_id = "/global";
      drivablilityPub.publish(drivability);


	}
	else
	{
		ROS_WARN_STREAM("Unknown FSM Command");
	}

	as->setSucceeded(homeResult);

}	

void goalPreemptCallback()
{
	ROS_DEBUG_STREAM("Entered Goal PECB");
}




//Utility functions
VertexSE3* vertex(int i) 
{
    return dynamic_cast<VertexSE3*>(optimizer.vertex(i));
}

void covArrToMat(const double* covar, MatrixXd& cov)
{
    cov << covar[0], covar[1], covar[2], covar[3], covar[4], covar[5],
        covar[6], covar[7], covar[8], covar[9], covar[10], covar[11],
        covar[12], covar[13], covar[14], covar[15], covar[16], covar[17],
        covar[18], covar[19], covar[20], covar[21], covar[22], covar[23],
        covar[24], covar[25], covar[26], covar[27], covar[28], covar[29],
        covar[30], covar[31], covar[32], covar[33], covar[34], covar[35];
}

bool addVertex(Node& node)
{
    const double pose[7] = {node.pose.pose.position.x, node.pose.pose.position.y, node.pose.pose.position.z, node.pose.pose.orientation.x, node.pose.pose.orientation.y, node.pose.pose.orientation.z, node.pose.pose.orientation.w};
    VertexSE3* robot =  new VertexSE3;
    robot->setId(node.idx);
    robot->setEstimateDataImpl(pose);
    return optimizer.addVertex(robot);
}

bool addEdge(Edge& gE)
{
    const double transf[7] = {gE.edge.pose.position.x,gE.edge.pose.position.y, gE.edge.pose.position.z, gE.edge.pose.orientation.x,gE.edge.pose.orientation.y,gE.edge.pose.orientation.z, gE.edge.pose.orientation.w};

    EdgeSE3* edge = new EdgeSE3;
    edge->vertices()[0] = optimizer.vertex(gE.from);
    edge->vertices()[1] = optimizer.vertex(gE.to);

    edge->setMeasurementData(transf);
    Eigen::MatrixXd info = Eigen::MatrixXd::Identity(6,6);
    //info = gE.edgeCov.inverse().eval();
    info = gE.edgeInf;
    edge->setInformation(info);
    return optimizer.addEdge(edge);
}

void quaternionToRPY(geometry_msgs::Quaternion qmsg, double& roll, double& pitch, double& yaw) 
{
    tf::Quaternion q;
    tf::quaternionMsgToTF(qmsg, q);
    tf::Matrix3x3(q).getRPY(roll, pitch, yaw);
}

////////////////////////
//Callbacks
//

void edge_cb(const graph_slam::EdgeConstPtr& edge)
{
    Edge new_edge;
    //covArrToMat(edge->covariance.data(), new_edge.edgeCov);
    
    ROS_DEBUG_STREAM("Adding EXTERNAL edge from " << edge->from << " to " << edge->to);
    
    //TODO: allow external edges to be added
   /* SE3 new_node = vertex(edge->from)->estimate()*transf;
    
    if(addVertex(edge->to, new_node) == 0) {
        new_info = 1;
    } else {
        //adding vertex was successful resize pG.nodes
        if(edge->to >= pG.nodes.size()) {
            pG.nodes.resize(edge->to+1);
            pG.nodes[edge->to].x = new_node[0];
            pG.nodes[edge->to].y = new_node[1];
            pG.nodes[edge->to].theta = new_node[2];
        }

        //always reoptimize on global_match
        if(edge->from == 0) {
            new_info = 1;
        }
    }
    
    addEdge(edge->from, edge->to, transf, info);*/
}

void pose_cb(const geometry_msgs::PoseStamped& pose)
{
    curPose = pose;
    got_pose = true;
}

void cloud_cb(const sensor_msgs::PointCloud2ConstPtr& input)
{
    pcl::fromROSMsg(*input, *current_cloud);
}


///////////////////////////////
//Regenerate Global Map!
//
void regenerateGlobalMap() 
{	
    ROS_DEBUG_STREAM("Regenerating Global Map");
    //clear the global map
    globalMap.clearMap();

  	//integrate the points
    for(unsigned int i=0; i<pG.nodes.size(); i++)
    {
        //Transform keyframe into grlobal frame
        Eigen::Affine3d trans;
        pcl::PointCloud<pcl::PointXYZ>::Ptr trans_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        tf::poseMsgToEigen(pG.nodes[i].pose.pose, trans);
        pcl::transformPointCloud(*pG.nodes[i].keyframe,*trans_cloud, trans);

		globalMap.addToMap(trans_cloud, pG.nodes[i].pose);
    }	 

    globalMap.filterPointCloud(0.1,0.1); //filter gloibal cloud so it won't kill rviz
    
}

///////////////////////////////////////
//Initizalize Optimization
//

void initOptimizer()
{
    //Creating the optimization problem 
    linearSolver->setBlockOrdering(false);
    optimizer.setAlgorithm(solver);
    optimizer.setVerbose(false);
    
    //fix first point at (0,0,0)
    const double origin[7] = {0,0,0,curPose.pose.orientation.x,curPose.pose.orientation.y,curPose.pose.orientation.z, curPose.pose.orientation.w};
    VertexSE3* robot =  new VertexSE3;
    robot->setId(0);
    robot->setEstimateDataImpl(origin);
    optimizer.addVertex(robot);

    VertexSE3* firstRobotPose = dynamic_cast<VertexSE3*>(optimizer.vertex(0));
    firstRobotPose->setFixed(true);
    Node n;
    n.idx=0; n.pose.pose.position.x=0; n.pose.pose.position.y=0; n.pose.pose.orientation = curPose.pose.orientation;

    *n.keyframe = *current_cloud;
    pG.nodes.push_back(n);

    //add initial cloud to map
    Eigen::Affine3d trans;
    pcl::PointCloud<pcl::PointXYZ>::Ptr trans_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    tf::poseMsgToEigen(pG.nodes[0].pose.pose, trans);
    pcl::transformPointCloud(*pG.nodes[0].keyframe,*trans_cloud, trans);

    globalMap.setMinClusterPoints(5); //use fewer point to initalize to get more points for matching
    globalMap.addToMap(trans_cloud, curPose);
    globalMap.setMinClusterPoints(10); //reset back
}

/////////////////////////////////
//Perform Optimization
//
void optimizeGraph(geometry_msgs::PoseStamped& newPose){
    ROS_DEBUG_STREAM("Optimizing");
    optimizer.initializeOptimization();

    optimizer.computeActiveErrors();
    ROS_DEBUG_STREAM("Initial chi2 = " << FIXED(optimizer.chi2()));
    
    //double chi2 = optimizer.chi2();
    //fout << chi2/edge_count << ", " << ros::Time::now().toSec() - start_time << ", ";

    optimizer.optimize(10);
    
    Node preNode = pG.nodes[pG.nodes.size()-1];
    for (size_t i = 0; i < pG.nodes.size(); ++i) {
        VertexSE3* v = vertex(i);
        double est[7];
        if(v == NULL) {
            continue;
        }

        v->getEstimateData(est);

        //ROS_DEBUG_STREAM( "[" << i << "] " << est[0] << ", "
        //     << est[1] << ", " 
        //     << est[2]);                 
         pG.nodes[i].pose.pose.position.x = est[0];
         pG.nodes[i].pose.pose.position.y = est[1];
         pG.nodes[i].pose.pose.position.z = est[2];
         pG.nodes[i].pose.pose.orientation.x =est[3]; 
         pG.nodes[i].pose.pose.orientation.y =est[4]; 
         pG.nodes[i].pose.pose.orientation.z =est[5]; 
         pG.nodes[i].pose.pose.orientation.w = est[6]; 
    }
    
    //Calculate pose offset
    Node postNode = pG.nodes[pG.nodes.size()-1];
    ros::spinOnce(); 

    double vnx = postNode.pose.pose.position.x -preNode.pose.pose.position.x;
    double vny = postNode.pose.pose.position.y -preNode.pose.pose.position.y;
    double vnz = postNode.pose.pose.position.z -preNode.pose.pose.position.z;
    double vntheta = tf::getYaw(postNode.pose.pose.orientation) - tf::getYaw(preNode.pose.pose.orientation);
    if(vntheta > M_PI) {
         vntheta = -(vntheta-2*M_PI);
    } else if (vntheta <-M_PI) {
        vntheta = -(vntheta+2*M_PI);
    }

    //transform pose
    double vpx = curPose.pose.position.x - preNode.pose.pose.position.x;
    double vpy = curPose.pose.position.y - preNode.pose.pose.position.y;
    double vpz = curPose.pose.position.z - preNode.pose.pose.position.z;
    double vptheta = tf::getYaw(curPose.pose.orientation) - tf::getYaw(preNode.pose.pose.orientation);
    if(vptheta > M_PI) {
         vptheta = -(vptheta-2*M_PI);
    } else if (vptheta <-M_PI) {
        vptheta = -(vptheta+2*M_PI);
    }

    newPose.pose.position.x = (vpx*cos(vntheta) + vpy*sin(vntheta) + vnx) - vpx;
    newPose.pose.position.y = (vpy*cos(vntheta) + vpx*sin(vntheta) + vny) - vpy;
    newPose.pose.position.z = vnz;
    newPose.pose.orientation = tf::createQuaternionMsgFromYaw(vntheta + vptheta);

    ROS_INFO_STREAM("Pose Offset: " << newPose.pose.position.x << ", " << newPose.pose.position.y << ", " << newPose.pose.position.z  << ", " << tf::getYaw(newPose.pose.orientation) );

    ROS_DEBUG_STREAM("optimization done.");

}

////////////////////////////////////////
//MAIN
///////////////////////////////////////
int main(int argc, char **argv)
{
    // Initialize ROS
    ros::init (argc, argv, "graph_slam");
    ros::NodeHandle nh;

    
    ros::Subscriber sub = nh.subscribe("/mapping/graph_slam/edges", 100, edge_cb);
    ros::Subscriber poseSub = nh.subscribe("/mapping/ekf/pose", 1, pose_cb);
    ros::Subscriber cloudSub = nh.subscribe("/velodyne_points", 1, cloud_cb);
    vizPub = nh.advertise<visualization_msgs::MarkerArray>("/mapping/poseGraphMarkers", 1, true);
    mlsPub = nh.advertise<visualization_msgs::MarkerArray>("/mapping/global/mls_viz", 1, true);
    posePub = nh.advertise<geometry_msgs::PoseStamped>("/mapping/graph_slam/pose_offset", 1, true);
    globalCloudPub = nh.advertise<sensor_msgs::PointCloud2>("/mapping/global/full_pointcloud", 1, true);
    obstacleCloudPub = nh.advertise<sensor_msgs::PointCloud2>("/mapping/global/obstacle_pointcloud", 1, true);
    groundCloudPub = nh.advertise<sensor_msgs::PointCloud2>("/mapping/global/ground_pointcloud", 1, true);
    drivablilityPub = nh.advertise<nav_msgs::OccupancyGrid>("/mapping/drivability_map", 1, true);
    

    ros::Publisher fromPub = nh.advertise<sensor_msgs::PointCloud2>("/mapping/from", 1, true);
    ros::Publisher toPub = nh.advertise<sensor_msgs::PointCloud2>("/mapping/to", 1, true);
    ros::Publisher transPub = nh.advertise<sensor_msgs::PointCloud2>("/mapping/trans", 1, true);

    ros::Rate loop_rate(10); //limit to Hz update rate

    //create an action server for FSM

  	ROS_INFO_STREAM("Initializing Mapping Action Server");

	actionlib::SimpleActionServer<nasa_msgs::MapAction> MappingAS(nh,"/nasa/mapping/action", false);
    as = &MappingAS;
    MappingAS.registerGoalCallback(&goalCallback);
    MappingAS.registerPreemptCallback(&goalPreemptCallback);         
    MappingAS.start();
	ROS_DEBUG_STREAM("Success!");


    graphVizInit();

    //wait for first scan
    bool wait = true;
    while(ros::ok() && wait) {
        ros::spinOnce();
        if(current_cloud->size() != 0 && got_pose) {
            wait = false;
        }
    }

    setup_gicp();

    initOptimizer();

    //publish initial cloud
    globalMap.visualize(mlsPub, "/global");
    ros::Time time = ros::Time::now();
    pcl::PointCloud<pcl::PointXYZ>::Ptr global_cloud;
    pcl::PointCloud<pcl::PointXYZ>::Ptr obstacle_cloud;
    pcl::PointCloud<pcl::PointXYZ>::Ptr ground_cloud;
    sensor_msgs::PointCloud2 cloud_msg;

    global_cloud = globalMap.getGlobalCloud();
    globalMap.getSegmentedClouds(obstacle_cloud, ground_cloud);

    pcl::toROSMsg(*global_cloud,cloud_msg);
    cloud_msg.header.frame_id = "/global";
    cloud_msg.header.stamp = time;
    globalCloudPub.publish(cloud_msg);

    pcl::toROSMsg(*obstacle_cloud,cloud_msg);
    cloud_msg.header.frame_id = "/global";
    cloud_msg.header.stamp = time;
    obstacleCloudPub.publish(cloud_msg);

    pcl::toROSMsg(*ground_cloud,cloud_msg);
    cloud_msg.header.frame_id = "/global";
    cloud_msg.header.stamp = time;
    groundCloudPub.publish(cloud_msg);

    nav_msgs::OccupancyGrid::Ptr drivability;
    drivability = globalMap.getDrivability();
    drivability->header.frame_id = "/global";
    drivablilityPub.publish(drivability);


    while(ros::ok()) {
        loop_rate.sleep();
        ros::spinOnce();

        //check for keyframe add
            //check for NN     
        Node gN;
        gN.idx = pG.nodes.size();
        
        //tf::Quaternion q;
        //tf::quaternionMsgToTF(curPose.pose.orientation, q);
        //tf::Matrix3x3 mat = tf::Matrix3x3(q);
        //mat.getEulerYPR(gN.theta, gN.pitch, gN.roll,1);
        gN.pose.pose = curPose.pose;

	    double nearestKF = graphSlamGetNearestKF(pG,gN);

        //FIXME: this is currently only based on distance!!!
        //  should also be based on map confidence?
        if(nearestKF < KNN_DIST_THRESH || current_cloud->size() < 20000) { //check distance
            continue;
        }

        //add keyframe
        *gN.keyframe = *current_cloud;
        pG.nodes.push_back(gN); //local pose graph
        addVertex(gN); //optimizer
        
        //generate new keyframe edges
        vector<int> KFNN = graphSlamGetKNN(pG, gN, GSLAM_KNN);
        

        for(int i = 0; i<KFNN.size(); i++) {
            if(graphSlamAddEdgeToX(pG, KFNN[i], gN.idx)) {
                addEdge(pG.edges.back()); //add most recent edge 
            }
        }

        //force edge to previous
        if(graphSlamAddEdgeToX(pG, gN.idx-1, gN.idx)) {
                addEdge(pG.edges.back()); //add most recent edge 
////////TESTINGGGG
 /*            sensor_msgs::PointCloud2 cloud_msg;
             pcl::toROSMsg(*pG.nodes[gN.idx].keyframe,cloud_msg);
            cloud_msg.header.frame_id = "/nasa";
            cloud_msg.header.stamp = ros::Time::now();
            toPub.publish(cloud_msg);


            pcl::toROSMsg(*pG.nodes[gN.idx-1].keyframe,cloud_msg);
            cloud_msg.header.frame_id = "/nasa";
            cloud_msg.header.stamp = ros::Time::now();
            fromPub.publish(cloud_msg);

            Eigen::Affine3d trans;
            pcl::PointCloud<pcl::PointXYZ>::Ptr trans_cloud(new pcl::PointCloud<pcl::PointXYZ>);
            tf::poseMsgToEigen(pG.edges.back().edge.pose, trans);
            pcl::transformPointCloud(*pG.nodes[gN.idx].keyframe,*trans_cloud, trans);

            pcl::toROSMsg(*trans_cloud,cloud_msg);
            cloud_msg.header.frame_id = "/nasa";
            cloud_msg.header.stamp = ros::Time::now();
            transPub.publish(cloud_msg);

*/
//
        }


        ///////////////////////
        //optimize Pose graph
        geometry_msgs::PoseStamped newPose;
        optimizeGraph(newPose); 
        
        //Output poseGraph and poseOffset
        vizPub.publish(getVisualMsg(pG.nodes, pG.edges));
        //FIXME I disabled pose offset update since it seems to be breaking everything.....
        posePub.publish(newPose);

        /////////////////////////////
        //Global Mapping Update 
        regenerateGlobalMap();
        
        //output maps
        globalMap.visualize(mlsPub, "/global");
        global_cloud = globalMap.getGlobalCloud();
        globalMap.getSegmentedClouds(obstacle_cloud, ground_cloud);
        drivability = globalMap.getDrivability();
        drivability->header.frame_id = "/global";

        time = ros::Time::now();
        pcl::toROSMsg(*global_cloud,cloud_msg);
        cloud_msg.header.frame_id = "/global";
        cloud_msg.header.stamp = time; 
        globalCloudPub.publish(cloud_msg);
        pcl::toROSMsg(*obstacle_cloud,cloud_msg);
        cloud_msg.header.frame_id = "/global";
        cloud_msg.header.stamp = time;
        obstacleCloudPub.publish(cloud_msg);
        pcl::toROSMsg(*ground_cloud,cloud_msg);
        cloud_msg.header.frame_id = "/global";
        cloud_msg.header.stamp = time;
        groundCloudPub.publish(cloud_msg);

        drivablilityPub.publish(drivability);
       
    }

    //clean up
    Factory::destroy();
    OptimizationAlgorithmFactory::destroy();
    HyperGraphActionLibrary::destroy();


    //fout.close();
    
    return 0;
}

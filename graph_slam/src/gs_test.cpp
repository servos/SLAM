#include <ros/ros.h>
#include <iostream>

#include <graph_slam/Node.h>
#include <graph_slam/Edge.h>
#include <graph_slam/NodeArray.h>


void nodeList_cb(const graph_slam::NodeArrayConstPtr& nodeList)
{
    for (size_t i = 0; i < nodeList->nodes.size(); ++i) {   
        std::cout << nodeList->nodes[i].x << ", "
             << nodeList->nodes[i].y << ", " 
             << nodeList->nodes[i].theta << std::endl;
    }

}

graph_slam::Edge newEdge(float x, float y, float theta, int from, int to) 
{
    graph_slam::Edge edge;
    edge.delta.x = x;
    edge.delta.y = y;
    edge.delta.theta = theta;
    edge.to = to;
    edge.from = from;
    edge.covariance[0] = 1;
    edge.covariance[4] = 1;
    edge.covariance[8] = 1;
    
    return edge;
}

int main(int argc, char **argv)
{
    // Initialize ROS
    ros::init (argc, argv, "gs_test");
    ros::NodeHandle nh;
    
    ros::Subscriber sub = nh.subscribe("/graph_slam/nodeArray", 10, nodeList_cb);
    ros::Publisher pub = nh.advertise<graph_slam::Edge> ("/graph_slam/edges", 100);
    
    //generate graph
    graph_slam::Edge edge;
    
    ros::Duration(0.5).sleep();
    
    edge = newEdge(1,0,1.5, 0,1);
    pub.publish(edge);
    
    ros::Duration(0.5).sleep();
    
    edge = newEdge(1,0,1.5, 1,2);
    pub.publish(edge);
    
    ros::Duration(0.5).sleep();
    
    edge = newEdge(1,0,1.5, 2,3);
    pub.publish(edge);
    
    ros::Duration(0.5).sleep();
    
    edge = newEdge(1,0,1.5, 3,0);
    pub.publish(edge);
    
    ros::Duration(0.5).sleep();
    
    edge = newEdge(0,-1,0, 3,6);
    pub.publish(edge);
    
    ros::Duration(0.5).sleep();
        
    edge = newEdge(0,2,-1.5, 6,2);
    pub.publish(edge);
    
    std::cout << "Published edges... spinning" << std::endl;
    ros::spin();
    
}

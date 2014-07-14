#include "global_mapping_viz.h"

std_msgs::ColorRGBA red, green, blue, white, black;
visualization_msgs::MarkerArray marker_array;
visualization_msgs::Marker msg_arrow;
visualization_msgs::Marker msg_dots;
geometry_msgs::Point msg_point;

void graphVizInit() {
	// colours
	red.r = 1; red.g = 0; red.b = 0; red.a = 1;
	green.r = 0; green.g = 1; green.b = 0; green.a = 1;
	blue.r = 0; blue.g = 0; blue.b = 1; blue.a = 1;
	white.r = 1; white.g = 1; white.b = 1; white.a = 1;
	black.r = 0; black.g = 0; black.b = 0; black.a = 0;

	msg_arrow.header.frame_id = "/global";
	msg_arrow.type = visualization_msgs::Marker::ARROW;
	msg_arrow.action = visualization_msgs::Marker::ADD;
	msg_arrow.lifetime = ros::Duration(0);

	msg_dots.header.frame_id = "/global";
	msg_dots.type = visualization_msgs::Marker::SPHERE_LIST;
	msg_dots.action = visualization_msgs::Marker::ADD;
	msg_dots.lifetime = ros::Duration(0);
	msg_dots.pose.orientation.w = 1;

}

visualization_msgs::MarkerArray getVisualMsg (NodeList& pG, EdgeList& edgeList) 
{
	marker_array.markers.clear();
	msg_arrow.points.clear();
	msg_dots.points.clear();
	int id = 0;

	// start vertex_list - Dots for graph nodes
	msg_dots.id = id;
	msg_dots.ns = "nodes";
	msg_dots.scale.x = 1;
	msg_dots.scale.y = 1;
	msg_dots.scale.z = 1;
	msg_dots.color = red;
	for(unsigned int i=0; i < pG.size(); i++) {
		msg_point.x = pG[i].pose.pose.position.x;
		msg_point.y = pG[i].pose.pose.position.y;
		msg_point.z = pG[i].pose.pose.position.z;
		msg_dots.points.push_back(msg_point);
	}
	marker_array.markers.push_back(msg_dots);
	id++;

	// start vertex_list -arrows for graph edges

	for(unsigned int i=0; i < edgeList.size(); i++) {

		//grab the edge 
		Edge pE = edgeList[i];

		msg_arrow.id = id;
		msg_arrow.points.clear();
		msg_arrow.ns = "edges";
		msg_arrow.scale.x = 0.5;
		msg_arrow.scale.y = 1;
		msg_arrow.scale.z = 1;
		if(pE.from == 0)
			msg_arrow.color = white;
		else 
			msg_arrow.color = blue;
			


		//fill in the point
		msg_point.x = pG[pE.from].pose.pose.position.x;
		msg_point.y = pG[pE.from].pose.pose.position.y;
		msg_point.z = pG[pE.from].pose.pose.position.z;
		//ROS_INFO("start adding: %f, %f", msg_point.x,msg_point.y);
		msg_arrow.points.push_back(msg_point);
		msg_point.x = pG[ pE.to].pose.pose.position.x;
		msg_point.y = pG[ pE.to].pose.pose.position.y;
		msg_point.z = pG[ pE.to].pose.pose.position.z;
		//ROS_INFO("end adding: %f, %f", msg_point.x,msg_point.y);
		msg_arrow.points.push_back(msg_point);
		//we have filled in the arrow stuff, push it back to the marker array
		marker_array.markers.push_back(msg_arrow);
		id++;
	}


	return marker_array;

}

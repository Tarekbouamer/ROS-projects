#include "husky_highlevel_controller/HuskyHighlevelController.hpp"

namespace husky_highlevel_controller {

HuskyHighlevelController::HuskyHighlevelController(ros::NodeHandle& nodeHandle) :
  nodeHandle_(nodeHandle)
{

	nodeHandle.getParam("/husky_highlevel_controller/quee",quee);
	nodeHandle.getParam("/husky_highlevel_controller/topic",topic);
	nodeHandle.getParam("/Husky_highlevel_controller/px",px);
	nodeHandle.getParam("/Husky_highlevel_controller/pz",pz);

	laser_scan_subs = nodeHandle.subscribe(topic, quee, &HuskyHighlevelController::laser_scan_Callback, this);

	vel_pub = nodeHandle.advertise<geometry_msgs::Twist>("/cmd_vel",1);

	visual_pub = nodeHandle.advertise<visualization_msgs::Marker>("visualization_marker", 0);

	ROS_INFO_STREAM( " " <<  quee );
	ROS_INFO_STREAM( " " <<  topic );

}

HuskyHighlevelController::~HuskyHighlevelController()
{
}

void HuskyHighlevelController::laser_scan_Callback( const sensor_msgs::LaserScan::ConstPtr& laser_scan_msgs )
{
	float min_distance = 100 ;
	int k;
	int n=0;;

	for (int i=0 ; i< laser_scan_msgs->ranges.size(); i++)
	{
		if (min_distance > laser_scan_msgs->ranges[i])
		{
			min_distance = laser_scan_msgs->ranges[i];
			k = i;
		}
	}

	float angle = laser_scan_msgs->angle_min + k *(2*laser_scan_msgs->angle_max)/laser_scan_msgs->ranges.size() ;

	float xp = min_distance*cos(angle);
	float yp = min_distance*sin(-angle);

	geometry_msgs::Twist vel;
	vel.linear.x= (xp-0.5);
	vel.angular.z= (0 - angle) ;
	vel_pub.publish(vel);

	visualization_msgs::Marker marker;
	marker.header.frame_id = "base_link";
	marker.header.stamp = ros::Time();
	marker.ns = "my_namespace";
	marker.id = 0;
	marker.type = visualization_msgs::Marker::SPHERE;
	marker.action = visualization_msgs::Marker::ADD;
	marker.pose.position.x = xp;
	marker.pose.position.y = yp;
	marker.scale.x = 0.1;
	marker.scale.y = 0.1;
	marker.scale.z = 0.1;
	marker.color.a = 1;
	marker.color.r = 0;
	marker.color.g = 1;
	marker.color.b = 0;
	visual_pub.publish(marker);


	 ROS_INFO(" X =: %f",xp);
	 ROS_INFO(" Y =: %f",yp);
	 ROS_INFO(" px =: %d",px);
	 ROS_INFO(" pz =: %d",pz);
	 ROS_INFO_STREAM("	" << std::endl );

}

} /* namespace */


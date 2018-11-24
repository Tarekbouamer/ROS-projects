#pragma once

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>

#include <vector>
#include <string>


#include <math.h>
#include <geometry_msgs/Twist.h>
#include <visualization_msgs/Marker.h>
#include <std_msgs/Bool.h>




#define pi 3.1419265358

namespace husky_highlevel_controller {

/*!
 * Class containing the Husky Highlevel Controller
 */
class HuskyHighlevelController {
public:
	/*!
	 * Constructor.
	 */
	HuskyHighlevelController(ros::NodeHandle& nodeHandle);

	/*!
	 * Destructor.
	 */
	virtual ~HuskyHighlevelController();

	void laser_scan_Callback( const sensor_msgs::LaserScan::ConstPtr& laser_scan_msgs );
	void getParameter();


private:
	ros::NodeHandle nodeHandle_;
	ros::Subscriber laser_scan_subs;
	ros::Publisher vel_pub;
	ros::Publisher visual_pub;
	int quee;
	int px;
	int pz;
	std::string  topic;

};

} /* namespace */

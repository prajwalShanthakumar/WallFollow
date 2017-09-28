#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "sensor_msgs/PointCloud2.h"
#include <laser_geometry/laser_geometry.h>

sensor_msgs::PointCloud2 out_cloud;

void scan_Callback(const sensor_msgs::LaserScan::ConstPtr& in_scan){
	laser_geometry::LaserProjection projector;
	projector.projectLaser(*in_scan,out_cloud);
}


int main(int argc, char **argv){
	ros::init(argc, argv, "scan_to_pc");
	ros::NodeHandle n;
	ros::Subscriber scan_sub = n.subscribe("/laser/scan", 10, scan_Callback);
	ros::Publisher cloud_pub = n.advertise<sensor_msgs::PointCloud2>("/pc2",10);
	ros::Rate rate(1.0);
	while(ros::ok()){
		ros::spinOnce();
		cloud_pub.publish(out_cloud);
		rate.sleep();
	}
	return 0;
}




#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/PointCloud2.h"
#include "sensor_msgs/PointCloud.h"
#include <pcl_ros/point_cloud.h>
#include <vector>
#include <cmath>
#include <visualization_msgs/Marker.h>
#include "wall_follow/Lines.h"
//#include <transform_datatypes.h>
using namespace std;

struct Line{
int confidence;			// number of inliers from the filtered cloud
float dist;				// value in metres
int angle;			 	// value in degrees

int x_intercept;
int y_intercept;
};

struct XY{
float x;
float y;
};

const float xmin = 0.5;		
const float xmax = 10;
const float ymin = 0.5;
const float ymax = 10;

const int theta_max = 360;			//# degrees
const int r_max = 200;				// rmax = rlim/resolution

const int r_lim = 10;				//# metres
const float resolution = 0.1;			//# metres

const int min_line_sep_dist = 2; 		//# metres
const int min_line_sep_angle = 45;		//# degrees

int threshold;
int lr_theta_min;
int lr_theta_max;

//tf::getYaw

ros::Publisher marker_pub;
ros::Publisher lines_pub;
pcl::PointCloud<pcl::PointXYZ>& cloud = *(new pcl::PointCloud<pcl::PointXYZ>());
std::vector<int> valid_indices(500);
int accumulator[theta_max][r_max];

XY r_theta_to_XY(float r, float theta){
	XY xy;
	xy.x = r * cos(theta * M_PI / 180.0);
	xy.y = r * sin(theta * M_PI / 180.0);
	return xy;
}


void preprocess_cloud(const sensor_msgs::PointCloud2::ConstPtr& msg){
	valid_indices.clear();
	
	
	pcl::fromROSMsg (*msg, cloud);								// converting pointcloud2 to cloud of XYZ points

	for(int i = 0; i < cloud.size(); i++){
	//ROS_INFO("I heard: %d %f %f", i, cloud[i].x, cloud[i].y);

		if(cloud[i].x < xmax && cloud[i].x > xmin && cloud[i].y < ymax && cloud[i].y > ymin){
			valid_indices.push_back(i);
		}

		else if(cloud[i].x < -1*xmin && cloud[i].x > -1*xmax && cloud[i].y < -1*ymin && cloud[i].y > -1*ymax){
			valid_indices.push_back(i);
		}

		else if(cloud[i].x < xmax && cloud[i].x > xmin && cloud[i].y < -1*xmin && cloud[i].y > -1*xmax){
			valid_indices.push_back(i);
		}

		else if(cloud[i].x < -1*xmin && cloud[i].x > -1*xmax && cloud[i].y < ymax && cloud[i].y > ymin){
			valid_indices.push_back(i);
		}
	}

	//ROS_INFO("xmin:%f xmax:%f ymin:%f ymax:%f",xmin, xmax, ymin, ymax);
	//ROS_INFO("cloud size: %d", cloud.size());
	//ROS_INFO("count: %u", valid_indices.size());
}





void clear_accumulator(){
	for(int theta = 0; theta < theta_max; theta++){
		for(int r = 0; r < r_max; r++){
			accumulator[theta][r] = 0;
		}
	}
}
void compute_r_theta(){

	for(int i = 0; i < valid_indices.size(); i++){		
		for(int iter_theta = lr_theta_min; iter_theta < lr_theta_max; iter_theta++){
			int theta;
			if(iter_theta < 0)				// using theta = 0-360, so avoid negative values
				theta = 360 + iter_theta;		// eg.: -10 transforms to 360 + (-10) = 350; -10 and 350 are the same thing...
			else
				theta = iter_theta;

			int r =   (cloud[valid_indices[i]].x * cos(theta*M_PI/180.0) + cloud[valid_indices[i]].y * sin(theta*M_PI/180.0)) / resolution;	// scale by resolution because we can't index into an array with floats

			if(r > 0)												// only allow positive values of r to eliminate errors because of duplicates (since we are considering 0 to 360)
				accumulator[theta][r] = accumulator[theta][r] + 1;
		}	
	}
}

Line find_best_line_and_remove(){
	int max = 0;										// finding the r,theta of best line
	int acc_max_theta = 0;
	int acc_max_r = 0;
	for(int theta = 0; theta < theta_max; theta++){
		for(int r = 0; r < r_max; r++){
			if(accumulator[theta][r] >= max){
				acc_max_theta = theta; 
				acc_max_r = r;
				max = accumulator[theta][r];	
			}
		}
	}
	
	Line line;
	line.confidence = max;
	line.dist =  (acc_max_r) * resolution;
	line.angle = acc_max_theta;
	
	ROS_INFO("dist: %f angle: %d confidence: %d",line.dist, line.angle, line.confidence);

	// throwing away all other similar lines; angle within 30 degrees of best AND distance within 1m of best

	
	for(int theta = acc_max_theta - min_line_sep_angle; theta < acc_max_theta + min_line_sep_angle; theta++){		
		int temp_theta = theta;
		
		if(theta < 0){
			temp_theta = theta + 360;				
		}

		else if(theta >= 360){
			temp_theta = theta - 360;
		}	
	
		for(int r = acc_max_r - min_line_sep_dist/resolution; r < acc_max_r + min_line_sep_dist/resolution; r++){		
			if(r <= 0 || r >= r_max){
				//ROS_INFO("warning: no line found");
				continue;
			}
			//ROS_INFO("theta:%d, r:%f, accum:%d",temp_theta,(r-1000)/10.0,accumulator[temp_theta][r]);
			accumulator[temp_theta][r] = 0;
		}
	}
	
	return line;
	 	
}

void hough_transform(){
	clear_accumulator();
	compute_r_theta();
	
	visualization_msgs::Marker points;
	points.header.frame_id = "/laser_frame";
	points.header.stamp = ros::Time::now();
	points.ns = "points_and_lines";
	points.pose.orientation.w = 1.0;
	points.id = 0;
	points.type = visualization_msgs::Marker::POINTS;
	points.scale.x = 0.2;
	points.scale.y = 0.2;
	points.color.g = 1.0;
    points.color.a = 1.0;

	visualization_msgs::Marker line_list;
	line_list.header.frame_id = "/laser_frame";
	line_list.header.stamp = ros::Time::now();
	line_list.ns = "points_and_lines";
	line_list.action = visualization_msgs::Marker::ADD;
	line_list.pose.orientation.w = 1.0;
	line_list.id = 2;
	line_list.type = visualization_msgs::Marker::LINE_LIST;
	line_list.scale.x = 0.1;
	line_list.color.r = 1.0;
	line_list.color.a = 1.0;

	wall_follow::Lines my_lines;
	my_lines.num_lines = 0;
	//my_lines.dist[0] = 5;
	//my_lines.angle[0] = 60;
	

	for(int i = 0; i < 1; i++){
		
		Line line1 = find_best_line_and_remove();

		//if(line1.confidence < threshold)
		//	break;

		my_lines.num_lines++;
		my_lines.dist[i] = line1.dist;
		my_lines.angle[i] = line1.angle;
		my_lines.confidence[i] = line1.confidence;

		geometry_msgs::Point p, p1, p2;
		p.x = r_theta_to_XY(line1.dist,line1.angle).x;
		p.y = r_theta_to_XY(line1.dist,line1.angle).y;
		p.z = 0;
		points.points.push_back(p);
	
		p1.x = p.x + 10 * cos( (line1.angle + 90) * M_PI / 180.0);
		p1.y = p.y + 10 * sin( (line1.angle + 90) * M_PI / 180.0);
		p1.z = 0;
		p2.x = p.x - 10 * cos( (line1.angle + 90) * M_PI / 180.0);
		p2.y = p.y - 10 * sin( (line1.angle + 90) * M_PI / 180.0);
		p2.z = 0;
		line_list.points.push_back(p1);
		line_list.points.push_back(p2);
	}

	marker_pub.publish(points);
	marker_pub.publish(line_list);
	lines_pub.publish(my_lines);
	
	

	//Line line2 = find_best_line_and_remove();
	//Line line3 = find_best_line_and_remove();

}


void pc_Callback(const sensor_msgs::PointCloud2::ConstPtr& msg){
	//ros::Time begin = ros::Time::now();
	preprocess_cloud(msg);
	hough_transform();
	//ros::Time end = ros::Time::now();
	//ROS_INFO("callback (line extraction) time = %f", end.toSec() - begin.toSec());
	
}

void getAllParams(ros::NodeHandle n){

	//n.getParam("/sensing_range/resolution",resolution);
	n.getParam("/line_region/theta_min",lr_theta_min);
	n.getParam("/line_region/theta_max",lr_theta_max);
	n.getParam("/line_region/threshold",threshold);
	ROS_INFO("line region theta min: %d",lr_theta_min);
	ROS_INFO("line region theta max: %d",lr_theta_max);
	ROS_INFO("line threshold: %d", threshold);
}

int main(int argc, char **argv){
	ros::init(argc, argv, "pc_subscriber");
	ros::NodeHandle n;
	getAllParams(n);
	ros::Subscriber sub = n.subscribe("pc2", 10, pc_Callback);
	marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 10);
	lines_pub = n.advertise<wall_follow::Lines>("ho/li",10);
	ros::spin();
	return 0;
}

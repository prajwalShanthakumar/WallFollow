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

//------------------------------------------------------------------------------------------------- Structures

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

//------------------------------------------------------------------------------------------------- Parameters


/*float xmin = 0.5;		// for preprocess cloud		
float xmax = 10;
float ymin = 0.5;
float ymax = 10;*/

float xmin[2];
float xmax[2];
float ymin[2];
float ymax[2];
				// for line extraction
int theta_max = 360;			//# degrees
int r_lim = 15;				//# metres
float resolution = 0.1;			//# metres
int r_max = 150;			// rmax = rlim/resolution

int lr_theta_min[2];
int lr_theta_max[2];
int threshold;
int min_line_sep_dist = 2; 		//# metres
int min_line_sep_angle = 45;		//# degrees


//------------------------------------------------------------------------------------------------- Global variables
//tf::getYaw

ros::Publisher marker_pub;
ros::Publisher lines_pub;
pcl::PointCloud<pcl::PointXYZ>& cloud = *(new pcl::PointCloud<pcl::PointXYZ>());
std::vector<int> valid_indices(500);
int accumulator[360][2000];

//------------------------------------------------------------------------------------------------- Functions

XY r_theta_to_XY(float r, float theta){
	XY xy;
	xy.x = r * cos(theta * M_PI / 180.0);
	xy.y = r * sin(theta * M_PI / 180.0);
	return xy;
}


void preprocess_cloud(const sensor_msgs::PointCloud2::ConstPtr& msg){
	//valid_indices.clear();
	
	
	pcl::fromROSMsg (*msg, cloud);								// converting pointcloud2 to cloud of XYZ points

	for(int i = 0; i < cloud.size(); i++){
	//ROS_INFO("I heard: %d %f %f", i, cloud[i].x, cloud[i].y);

		
		/*if( sqrt(pow(cloud[i].x,2)  + pow(cloud[i].y,2)) > 1 && sqrt(pow(cloud[i].x,2)  + pow(cloud[i].y,2)) < 15){
			valid_indices.push_back(i);
		}*/
		/*

		if(cloud[i].x < xmax && cloud[i].x > xmin && cloud[i].y < ymax && cloud[i].y > ymin){
			//if(atan2 is between limits)
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
		*/

	}

	//ROS_INFO("xmin:%f xmax:%f ymin:%f ymax:%f",xmin, xmax, ymin, ymax);
	//ROS_INFO("cloud size: %d", cloud.size());
	ROS_INFO("initial count: %u", valid_indices.size());
}



void filter_cloud(int id){

	valid_indices.clear();

	for(int i = 0; i < cloud.size(); i++){
		if( sqrt(pow(cloud[i].x,2)  + pow(cloud[i].y,2)) > 1 && sqrt(pow(cloud[i].x,2)  + pow(cloud[i].y,2)) < 14){
			
			if(cloud[i].x < xmax[id] && cloud[i].x > xmin[id] && cloud[i].y < ymax[id] && cloud[i].y > ymin[id]){
				valid_indices.push_back(i);

			}
			
		}
	}
}

void clear_accumulator(){
	for(int theta = 0; theta < theta_max; theta++){
		for(int r = 0; r < r_max; r++){
			accumulator[theta][r] = 0;
		}
	}
}
void compute_r_theta(int line_id){

	for(int i = 0; i < valid_indices.size(); i++){		
		for(int iter_theta = lr_theta_min[line_id]; iter_theta < lr_theta_max[line_id]; iter_theta++){
			int theta;
			if(iter_theta < 0)				// using theta = 0-360, so avoid negative values
				theta = 360 + iter_theta;		// eg.: -10 transforms to 360 + (-10) = 350; -10 and 350 are the same thing...
			else
				theta = iter_theta;

			int r =  round( (cloud[valid_indices[i]].x * cos(theta*M_PI/180.0) + cloud[valid_indices[i]].y * sin(theta*M_PI/180.0)) / resolution);	// scale by resolution because we can't index into an array with floats

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

	//clear_accumulator();
	//compute_r_theta();
	
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
	

	for(int i = 0; i < 2; i++){


		filter_cloud(i);

		clear_accumulator();
		
		compute_r_theta(i);

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
		//points.points.push_back(p);
	
		int num_points = 0;
		float xmax = -20;
		float xmin = 20;
		float ymax = -20;
		float ymin = 20;
		for(int j = 0; j < valid_indices.size(); j++){		// iterate over all valid points; find the end points of the line	
				float r =   (cloud[valid_indices[j]].x * cos(line1.angle*M_PI/180.0) + cloud[valid_indices[j]].y * sin(line1.angle*M_PI/180.0));
				if(r > 0 && ( fabs(r - line1.dist) < resolution ) ){
					num_points++;
					if( cloud[valid_indices[j]].x > xmax )
						xmax = cloud[valid_indices[j]].x;
					if( cloud[valid_indices[j]].x < xmin )
						xmin = cloud[valid_indices[j]].x;
					if( cloud[valid_indices[j]].y > ymax )
						ymax = cloud[valid_indices[j]].y;
					if( cloud[valid_indices[j]].y < ymin )
						ymin = cloud[valid_indices[j]].y;	
				}
				
		}

		if(line1.angle >= 180) line1.angle = line1.angle - 180;		// tan(180 + theta) = tan(theta)

		if(line1.angle == 0){		// perpendicular to line from origin is at 0 degrees (X-axis). The line itself is parallel to Y axis
			ROS_INFO("Points are (%.2f, %.2f) and (%.2f, %.2f)", xmin, ymin, xmin, ymax);// xmin = xmax (ideally) is not equal to line1.dist necessarily!!! (+-)
			p1.x = xmin; p1.y = ymin; p2.x = xmin; p2.y = ymax;
		}
		else if(line1.angle == 90){
			ROS_INFO("Points are (%.2f, %.2f) and (%.2f, %.2f)", xmin, line1.dist, xmax, line1.dist);
			p1.x = xmin; p1.y = ymin; p2.x = xmax; p2.y = ymin;
		}
		else if(line1.angle < 90){	// perpendicular to line less than 90. Line itself greater than 90 (-ve slope) 
			ROS_INFO("Points are (%.2f, %.2f) and (%.2f, %.2f)", xmin, ymax, xmax, ymin);
			p1.x = xmin; p1.y = ymax; p2.x = xmax; p2.y = ymin;
		}
		else{				// line1.angle > 90
			ROS_INFO("Points are (%.2f, %.2f) and (%.2f, %.2f)", xmin, ymin, xmax, ymax);
			p1.x = xmin; p1.y = ymin; p2.x = xmax; p2.y = ymax;
		}
		ROS_INFO("num_points = %d",num_points);

		//p1.x = p.x + 10 * cos( (line1.angle + 90) * M_PI / 180.0);
		//p1.y = p.y + 10 * sin( (line1.angle + 90) * M_PI / 180.0);
		p1.z = 0;
		//p2.x = p.x - 10 * cos( (line1.angle + 90) * M_PI / 180.0);
		//p2.y = p.y - 10 * sin( (line1.angle + 90) * M_PI / 180.0);
		p2.z = 0;
		line_list.points.push_back(p1);
		line_list.points.push_back(p2);

		points.points.push_back(p1);
		points.points.push_back(p2);

		my_lines.x1[i] = p1.x;
		my_lines.x2[i] = p2.x;
		my_lines.y1[i] = p1.y;
		my_lines.y2[i] = p2.y;

		
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
	
	n.getParam("sensing/xmin",xmin[0]);
	n.getParam("sensing/xmax",xmax[0]);
	n.getParam("sensing/ymin",ymin[0]);
	n.getParam("sensing/ymax",ymax[0]);

	n.getParam("sensing/xmin2",xmin[1]);
	n.getParam("sensing/xmax2",xmax[1]);
	n.getParam("sensing/ymin2",ymin[1]);
	n.getParam("sensing/ymax2",ymax[1]);
	
	n.getParam("line/theta_min",lr_theta_min[0]);
	n.getParam("line/theta_max",lr_theta_max[0]);
	n.getParam("line/theta_min2",lr_theta_min[1]);
	n.getParam("line/theta_max2",lr_theta_max[1]);
	n.getParam("line/resolution",resolution);
	n.getParam("line/threshold",threshold);
	n.getParam("line/min_line_separation_dist",min_line_sep_dist);
	n.getParam("line/min_line_separation_angle",min_line_sep_angle);

	ROS_INFO("xmin: %f", xmin[0]);
	ROS_INFO("xmax: %f", xmax[0]);
	ROS_INFO("ymin: %f", ymin[0]);
	ROS_INFO("ymax: %f", ymax[0]);

	ROS_INFO("xmin2: %f", xmin[1]);
	ROS_INFO("xmax2: %f", xmax[1]);
	ROS_INFO("ymin2: %f", ymin[1]);
	ROS_INFO("ymax2: %f", ymax[1]);
	
	
	ROS_INFO("line region theta min1: %d",lr_theta_min[0]);
	ROS_INFO("line region theta max1: %d",lr_theta_max[0]);
	ROS_INFO("line region theta min2: %d",lr_theta_min[1]);
	ROS_INFO("line region theta max2: %d",lr_theta_max[1]);

	ROS_INFO("line resolution: %f", resolution);
	ROS_INFO("line threshold: %d", threshold);
	ROS_INFO("min_line_sep_dist: %d", min_line_sep_dist);
	ROS_INFO("min_line_sep_angle: %d", min_line_sep_angle);

	r_max = r_lim/resolution;
	ROS_INFO("r_lim: %d",r_lim);	
	ROS_INFO("r_max: %d",r_max);
	
	
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

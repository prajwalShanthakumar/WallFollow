#include "ros/ros.h"
#include "std_msgs/String.h"
#include <cmath>
#include <visualization_msgs/Marker.h>


using namespace std;


int chooser = 0;
ros::Publisher marker_pub;
/*
hough_transform(){

	//clear_accumulator();
	//compute_r_theta();

	if m.type 
	
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

}

*/

void vm_Callback(const visualization_msgs::Marker::ConstPtr& m){
	
	if (m->type == visualization_msgs::Marker::POINTS){
		
		visualization_msgs::Marker points;
		points.header.frame_id = "/laser_frame";
		points.header.stamp = ros::Time::now();
		points.ns = "points_and_lines";
		points.pose.orientation.w = 1.0;
		points.id = 0;
		points.type = visualization_msgs::Marker::POINTS;
		//points.action = visualization_msgs::Marker::DELETE;
		points.scale.x = 0.2;
		points.scale.y = 0.2;
		points.color.g = 1.0;
	    	points.color.a = 1.0;


		points.points.push_back(m->points[0]);
		points.points.push_back(m->points[1]);
		marker_pub.publish(points);
		
	}

	else{
		visualization_msgs::Marker line_list;
		line_list.header.frame_id = "/laser_frame";
		line_list.header.stamp = ros::Time::now();
		line_list.ns = "points_and_lines";
		//line_list.action = visualization_msgs::Marker::DELETEALL;
		line_list.pose.orientation.w = 1.0;
		line_list.id = 2;
		line_list.type = visualization_msgs::Marker::LINE_LIST;
		line_list.scale.x = 0.1;
		line_list.color.r = 1.0;
		line_list.color.a = 1.0;

		line_list.points.push_back(m->points[0]);
		line_list.points.push_back(m->points[1]);

		marker_pub.publish(line_list);
	}

	ROS_INFO("m callback");
	//if( (chooser % 4) < 1){
	//	marker_pub.publish(mm);
	//}
	chooser++;
}

int main(int argc, char **argv){
	ros::init(argc, argv, "marker_fix");
	ros::NodeHandle n;

	ros::Subscriber sub = n.subscribe("visualization_marker", 10, vm_Callback);
	marker_pub = n.advertise<visualization_msgs::Marker>("vm_fixed", 10);
	
	ros::spin();
	return 0;
}

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/PositionTarget.h>
#include <vector>
#include "std_msgs/Float32MultiArray.h"
#include "wall_follow/Lines.h"
#include <cmath>
#include "tf/transform_datatypes.h"

struct PID{
float Kp;
float Kd;
float Ki;
};


enum Prev{Initial,Below,Above};
Prev prev = Initial;

enum Altitude_Mode{FromBelow,FromAbove,Centroid};
Altitude_Mode alt_mode = FromAbove;

int laser_rf_offset = 0;						// lidar (Sweep) leads robot x axis (east) by 0 degrees in simulation and 0 degrees on the actual quad

float max_vel_h = 0.5;							// max velocities
float max_vel_z = 0.5;
float nominal_vel = 0;

PID h_pid = {0.5,0.05,0.0};						// PID parameters	
PID z_pid = {0.5,0.05,0.0};						// kp = 1.0 ; kd = 0.15;

float desired_wall_dist = 5.0;
float desired_buffer = 1.5;
float move_threshold_vertical = 0.7;
float move_threshold_horizontal = 0.7;
	
int confidence_threshold = 5;

									

// get rid of these
float hold_error;				// hold errors 
float va_hold_error;
float altitude_error;

float prev_hold_errors[5];
float prev_va_hold_errors[5];
float prev_altitude_errors[5];

// get rid of these
float hold_velocity;				// velocities 		(can be updated faster? because yaw updates are coming fast???
float va_hold_velocity;

float x_rf_hold;				
float y_rf_hold;
float va_x_rf_hold;
float va_y_rf_hold;
float altitude_velocity;
float x_rf_move;
float y_rf_move;

geometry_msgs::PoseStamped local_pose;		// mavros		
mavros_msgs::State current_state;

wall_follow::Lines hor_lines;			// lines
wall_follow::Lines vert_lines;

bool new_hor_data = 0;
bool new_vert_data = 0;				// flags

using namespace std;



void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

void local_pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg){
	local_pose = *msg;
}

void hor_lines_cb(const wall_follow::Lines::ConstPtr& lines){
	//ROS_INFO("first line: dist:%2.4f, angle:%d, confidence:%d",lines->dist[0], lines->angle[0], lines->confidence[0]);
	hor_lines = *lines;
	new_hor_data = 1;
}

void vert_lines_cb(const wall_follow::Lines::ConstPtr& lines){
	//ROS_INFO("first line: dist:%2.4f, angle:%d, confidence:%d",lines->dist[0], lines->angle[0], lines->confidence[0]);
	vert_lines = *lines;
	new_vert_data = 1;
}

float limit_velocity(float vel, float max_vel){
	if(vel > max_vel)
		vel = max_vel;
	else if(vel < -1 * max_vel)
		vel = -1 * max_vel;
	return vel;
}

float computePID(float error, float prev_error[], PID pid, float max_vel){
	float P = error * pid.Kp;
	float D = (error - prev_error[0]) * pid.Kd;
	float I = (prev_error[0] +  prev_error[1] + prev_error[2] + prev_error[3] + prev_error[4]) * pid.Ki;
	
	ROS_INFO("cur_error = %f, prev_error = %f, difference = %f", error, prev_error[0], error - prev_error[0]);

	prev_error[4] = prev_error[3];
	prev_error[3] = prev_error[2];
	prev_error[2] = prev_error[1];
	prev_error[1] = prev_error[0];
	prev_error[0] = error;

	
	ROS_INFO("\n P = %f    D = % f \n", P, D);
	return limit_velocity(P + I + D, max_vel);
}



mavros_msgs::PositionTarget computeTargetVel(){
	mavros_msgs::PositionTarget target_vel;
	target_vel.header.stamp = ros::Time::now();
	target_vel.header.frame_id = "local_frame";
	target_vel.coordinate_frame = mavros_msgs::PositionTarget::FRAME_BODY_NED;
	target_vel.type_mask = 0b111111000111;
	target_vel.yaw_rate = 1;
	int orientation = (int)(tf::getYaw(local_pose.pose.orientation) * 180 / M_PI);

	// horizontal control:	// maybe increase update rate because yaw updates are coming fast???
	
	if(new_hor_data){			// else, keep controlling with old values 	
		if(hor_lines.confidence[0] > confidence_threshold){
			int theta_rf_hold = hor_lines.angle[0] + laser_rf_offset;
			hold_error = (hor_lines.dist[0] - desired_wall_dist);
			hold_velocity = computePID(hold_error, prev_hold_errors, h_pid, max_vel_h);

			x_rf_hold = hold_velocity * cos (theta_rf_hold * M_PI / 180.0);
			y_rf_hold = hold_velocity * sin (theta_rf_hold * M_PI / 180.0);
			
			int theta_rf_move = theta_rf_hold - 90;
			x_rf_move = nominal_vel * cos (theta_rf_move * M_PI / 180.0);
			y_rf_move = nominal_vel * sin (theta_rf_move * M_PI / 180.0);

		}
		else{
			prev_hold_errors[0] = 0; 	prev_hold_errors[1] = 0; 	prev_hold_errors[2] = 0; 	prev_hold_errors[3] = 0; 	prev_hold_errors[4] = 0;	
		}
		new_hor_data = 0;
	}
		


	// altitude control:

	if(new_vert_data){
		if(vert_lines.confidence[0] > confidence_threshold){
			
			float z_offset_from_desired;
			if(alt_mode == FromAbove){
				z_offset_from_desired = vert_lines.x2[0] - desired_buffer;
			}
			else if(alt_mode == FromBelow){
				z_offset_from_desired = vert_lines.x1[0] + desired_buffer;
			}
			else{	// alt_mode == Centroid
				z_offset_from_desired = (vert_lines.x1[0] + vert_lines.x2[0]) / 2.0;
			}
			ROS_INFO("z_offset_from_desired = %f", z_offset_from_desired);
			altitude_error = z_offset_from_desired;
			altitude_velocity = computePID(altitude_error,prev_altitude_errors, z_pid, max_vel_z);

			// vert assisted horizontal control
			va_hold_error = -1 * (vert_lines.dist[0] - desired_wall_dist);
			va_hold_velocity = computePID(va_hold_error, prev_va_hold_errors, h_pid, max_vel_h);
			va_x_rf_hold = va_hold_velocity;
			va_y_rf_hold = 0;
		}
		else{
			prev_altitude_errors[0] = 0;	prev_altitude_errors[1] = 0;	prev_altitude_errors[2] = 0;	prev_altitude_errors[3] = 0;	prev_altitude_errors[4] = 0;
			prev_va_hold_errors[0] = 0;	prev_va_hold_errors[1] = 0;	prev_va_hold_errors[2] = 0;	prev_va_hold_errors[3] = 0;	prev_va_hold_errors[4] = 0;
		}
		new_vert_data = 0;
	}

	// state machine:
	
	// horizontal:
	if(fabs(altitude_error) < move_threshold_vertical && fabs(hold_error) < move_threshold_horizontal && vert_lines.confidence[0] > confidence_threshold && hor_lines.confidence[0] > confidence_threshold){
		target_vel.velocity.x = x_rf_hold + x_rf_move;
		target_vel.velocity.y = y_rf_hold + y_rf_move;
	}
	else{
		if(hor_lines.confidence[0] > confidence_threshold){
			target_vel.velocity.x = x_rf_hold;
			target_vel.velocity.y = y_rf_hold;
		}
		else if(vert_lines.confidence[0] > confidence_threshold){
			target_vel.velocity.x = va_x_rf_hold;
			target_vel.velocity.y = va_y_rf_hold;
		}
		else{
			target_vel.velocity.x = 0;
			target_vel.velocity.y = 0;
		}	
	}

	// vertical:
	if(vert_lines.confidence[0] > confidence_threshold){
		target_vel.velocity.z = altitude_velocity;
	}
	else{
		target_vel.velocity.z = 0;
	}

	ROS_INFO("altitude_error %f",altitude_error);
	ROS_INFO("dist_error %f", hold_error);
	ROS_INFO("target vel x = %f,  y = %f,  z = %f", target_vel.velocity.x, target_vel.velocity.y, target_vel.velocity.z);
	return target_vel;
}


void getAllParams(ros::NodeHandle n){

	n.getParam("/control/Kp",h_pid.Kp);
	n.getParam("/control/Kd",h_pid.Kd);
	n.getParam("/control/Ki",h_pid.Ki);
	n.getParam("/control/Z_Kp",z_pid.Kp);
	n.getParam("/control/Z_Kd",z_pid.Kd);
	n.getParam("/control/Z_Ki",z_pid.Ki);
	
	n.getParam("/control/max_vel_h",max_vel_h);
	n.getParam("/control/max_vel_z",max_vel_z);
	n.getParam("/control/nominal_vel",nominal_vel);

	n.getParam("/control/laser_rf_offset",laser_rf_offset);

	n.getParam("/hor/line/threshold",confidence_threshold);
	n.getParam("/vert/line/threshold",confidence_threshold);

	n.getParam("/flight/desired_wall_dist", desired_wall_dist);
	n.getParam("/flight/desired_buffer", desired_buffer);
	n.getParam("/flight/move_threshold_vertical", move_threshold_vertical);
	n.getParam("/flight/move_threshold_horizontal", move_threshold_horizontal);


	ROS_INFO("Kp: %f", h_pid.Kp);
	ROS_INFO("Kd: %f", h_pid.Kd);
	ROS_INFO("Ki: %f", h_pid.Ki);
	ROS_INFO("z_Kp: %f", z_pid.Kp);
	ROS_INFO("z_Kd: %f", z_pid.Kd);
	ROS_INFO("z_Ki: %f", z_pid.Ki);

	ROS_INFO("max_vel_h: %f",max_vel_h);
	ROS_INFO("max_vel_z: %f",max_vel_z);
	ROS_INFO("nominal_vel: %f",nominal_vel);

	ROS_INFO("laser_rf_offset: %d",laser_rf_offset);
	
	ROS_INFO("hor line threshold: %d", confidence_threshold);
	ROS_INFO("vert line threshold: %d", confidence_threshold);

	ROS_INFO("desired_wall_dist: %f",desired_wall_dist);
	ROS_INFO("desired_buffer: %f",desired_buffer);
	ROS_INFO("move_threshold_horizontal: %f", move_threshold_horizontal);
	ROS_INFO("move_threshold_vertical: %f", move_threshold_vertical);

}



int main(int argc, char **argv){
//_____________________________________________________________________________________________________#InitializeNodePubSubAndRate

	ros::init(argc, argv, "offb_node");
	ros::NodeHandle nh;

	getAllParams(nh);

	ros::Subscriber vert_lines_sub = nh.subscribe<wall_follow::Lines>("/vert/ho/li",10,vert_lines_cb);
	ros::Subscriber hor_lines_sub = nh.subscribe<wall_follow::Lines>("/hor/ho/li",10,hor_lines_cb);

	ros::Publisher velocity_pub = nh.advertise<mavros_msgs::PositionTarget>("/mavros/setpoint_raw/local",10);
	ros::Subscriber pose_sub = nh.subscribe<geometry_msgs::PoseStamped>("mavros/local_position/pose", 10, local_pose_cb);
	ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 10, state_cb);

	ros::Rate rate(20.0);		//the setpoint publishing rate MUST be faster than 2Hz
								
	while(ros::ok() && current_state.connected){	// wait for FCU connection	
	ros::spinOnce();
	rate.sleep();
	}


//___________________________________________________________________________________________________________________#dummysetpoints

													// body: yaw is relative
													// offset: translation is relative
													// local frame: whereever we entered offboard? When system switched on?
	mavros_msgs::PositionTarget target_vel;
	target_vel.header.stamp = ros::Time::now();
	target_vel.header.frame_id = "local_frame";
	target_vel.coordinate_frame = mavros_msgs::PositionTarget::FRAME_BODY_NED;
	target_vel.type_mask = 0b111111000111;
	target_vel.yaw_rate = 0;
	target_vel.velocity.x = 0;
	target_vel.velocity.y = 0;
	target_vel.velocity.z = 0;

	for(int i = 100; ros::ok() && i > 0; --i){
		velocity_pub.publish(target_vel);
		rate.sleep();
	}



//__________________________________________________________________________________________________________________ #mainloop

    while(ros::ok()){
        ros::spinOnce();
	velocity_pub.publish(computeTargetVel());
        rate.sleep();
    }

    return 0;
}

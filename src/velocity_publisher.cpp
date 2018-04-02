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

enum Mode {follow_wall, recover_altitude};
Mode current_mode = recover_altitude;

const float desired_wall_dist = 5.0;
const float desired_altitude = 5.0;

const float altitude_entry_tolerance = 0.2;	
const float altitude_exit_tolerance = 1;	
const float confidence_threshold = 10;

const int laser_rf_offset = 90;		// lidar (Sweep) leads robot x axis (east) by 90 degrees in simulation and 180 degrees on the actual quad
						// verify and make sure that both are anticlockwise

float hold_error;				// hold errors 
float hold_velocity;
float prev_hold_errors[5];
float altitude_error;
float altitude_velocity;
float prev_altitude_errors[5];

float x_rf_hold;
float y_rf_hold;
float x_rf_move;
float y_rf_move;

const float Kp = 1;				// PID parameters
const float Kd = 0.15;
const float Ki = 0;
float max_velocity = 0.5;
float nominal_velocity = 0;

geometry_msgs::PoseStamped local_pose;		
mavros_msgs::State current_state;

wall_follow::Lines hough_lines;			// lines
wall_follow::Lines hor_lines;
wall_follow::Lines vert_lines;

bool new_data = 0;
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

float limit_velocity(float vel){
	if(vel > max_velocity)
		vel = max_velocity;
	else if(vel < -1 * max_velocity)
		vel = -1 * max_velocity;
	return vel;
}

float PID(float error, float prev_error[]){
	float P = error * Kp;
	float D = (error - prev_error[0]) * Kd;
	float I = (prev_error[0] +  prev_error[1] + prev_error[2] + prev_error[3] + prev_error[4]) * Ki;
	
	ROS_INFO("cur_error = %f, prev_error = %f, difference = %f", error, prev_error[0], error - prev_error[0]);

	prev_error[4] = prev_error[3];
	prev_error[3] = prev_error[2];
	prev_error[2] = prev_error[1];
	prev_error[1] = prev_error[0];
	prev_error[0] = error;

	
	ROS_INFO("\n P = %f    D = % f \n", P, D);
	return limit_velocity(P + I + D);
}



mavros_msgs::PositionTarget computeTargetVel(){
	mavros_msgs::PositionTarget target_vel;
	target_vel.header.stamp = ros::Time::now();
	target_vel.header.frame_id = "local_frame";
	target_vel.coordinate_frame = mavros_msgs::PositionTarget::FRAME_BODY_NED;
	target_vel.type_mask = 0b111111000111;
	target_vel.yaw_rate = 1;
	int orientation = (int)(tf::getYaw(local_pose.pose.orientation) * 180 / M_PI);
	//if(orientation < 0)
		//orientation = 360 + orientation;
	//ROS_INFO("yaw = %d", orientation);

	//int hough_angle_rf = hough_lines.angle[0];					//lf = local frame, not laser frame!!!

	//ROS_INFO("altitude: %f", local_pose.pose.position.z);
	//ROS_INFO("confidence: %d", hough_lines.confidence[0]);
 
	//_______________________________________________________________________________________________State machine
	/*if(current_mode == recover_altitude){
		if( fabs(local_pose.pose.position.z - desired_altitude) < altitude_entry_tolerance &&
		hough_lines.confidence[0] >= confidence_threshold ){
			current_mode = follow_wall;
		}
	}
	
	else if(current_mode == follow_wall){
		if( fabs(local_pose.pose.position.z - desired_altitude) > altitude_exit_tolerance ||
		hough_lines.confidence[0] < confidence_threshold ){
			current_mode = recover_altitude;
		}
	}*/

	//_______________________________________________________________________________________________Control
	/*if(current_mode == follow_wall){
		ROS_INFO("follow wall mode");
		
		// hold velocity computation in x and y directions based on robot's current orientation; PID regulation used;
		int theta_rf_hold = hough_lines.angle[0] + laser_rf_offset;	// theta_rf_hold is the angle of the wall from the robot x-axis (East)
		hold_error = (hough_lines.dist[0] - desired_wall_dist);
		if(new_data){
			hold_velocity = PID(hold_error,prev_hold_errors);
			new_data = 0;
		}
		float x_rf_hold = hold_velocity * cos (theta_rf_hold * M_PI / 180.0);
		float y_rf_hold = hold_velocity * sin (theta_rf_hold * M_PI / 180.0);


		// move velocity computation in x and y direction based on robot's current orientation; open loop constant nominal velocity;
		int theta_rf_move = theta_rf_hold - 90;
		float x_rf_move = nominal_velocity * cos (theta_rf_move * M_PI / 180.0);
		float y_rf_move = nominal_velocity * sin (theta_rf_move * M_PI / 180.0);

		// combining hold velocity and move velocity (alternatively using one or the other)
		float x_rf, y_rf;

		if(hold_error < 1){						// follow wall (move) and correct distance from wall
			x_rf = x_rf_hold + x_rf_move;
			y_rf = y_rf_hold + y_rf_move;
			ROS_INFO("move");
		}

		else{								// only correct distance to wall
			x_rf = x_rf_hold;
			y_rf = y_rf_hold;
			ROS_INFO("stay");
		}

		ROS_INFO("x_rf_hold: %f 	y_rf_hold: %f", x_rf_hold, y_rf_hold);

		target_vel.velocity.x = x_rf; // * 0.25;
		target_vel.velocity.y = y_rf; // * 0.25;
		target_vel.velocity.z = 0;
		//target_vel.velocity.x = limit_velocity(target_vel.velocity.x);
		//target_vel.velocity.y = limit_velocity(target_vel.velocity.y);
		//target_vel.velocity.z = limit_velocity(target_vel.velocity.z);
		ROS_INFO("current wall distance: %f", hough_lines.dist[0]);
		ROS_INFO("target vel x = %f,  y = %f,  z = %f", target_vel.velocity.x, target_vel.velocity.y, target_vel.velocity.z);
		ROS_INFO("\n\n\n");

		
	}


	else{
		//recover_altitude;
		ROS_INFO("recover altitude mode");
		
		target_vel.velocity.x = 0;
		target_vel.velocity.y = 0;
		target_vel.velocity.z = desired_altitude - local_pose.pose.position.z;

		//if(local_pose.pose.position.z < 1){
		//	target_vel.velocity.x = 0;
		//	target_vel.velocity.y = 0;
		//	target_vel.yaw_rate = 0;
		//}
		//target_vel.velocity.x = limit_velocity(target_vel.velocity.x);
		//target_vel.velocity.y = limit_velocity(target_vel.velocity.y);
		target_vel.velocity.z = limit_velocity(target_vel.velocity.z);

		ROS_INFO("current altitude: %f", local_pose.pose.position.z);
		ROS_INFO("target vel x = %f,  y = %f,  z = %f", target_vel.velocity.x, target_vel.velocity.y, target_vel.velocity.z);
		ROS_INFO("\n\n\n");
	}
*/
	
	// horizontal control:
	
	if(new_hor_data){			// else, keep controlling with old values 
		if(hor_lines.confidence[0] > confidence_threshold){
			int theta_rf_hold = hough_lines.angle[0] + laser_rf_offset;
			hold_error = (hor_lines.dist[0] - desired_wall_dist);
			hold_velocity = PID(hold_error,prev_hold_errors);
			x_rf_hold = hold_velocity * cos (theta_rf_hold * M_PI / 180.0);
			y_rf_hold = hold_velocity * sin (theta_rf_hold * M_PI / 180.0);
			
			int theta_rf_move = theta_rf_hold - 90;
			x_rf_move = nominal_velocity * cos (theta_rf_move * M_PI / 180.0);
			y_rf_move = nominal_velocity * sin (theta_rf_move * M_PI / 180.0);

		}
		else{
			x_rf_hold = 0; y_rf_hold = 0;
			x_rf_move = 0; y_rf_move = 0;
			
		}
		new_hor_data = 0;
	}
		


	// altitude control:

	if(new_vert_data){
		if(vert_lines.confidence[0] > confidence_threshold){
			altitude_error = (vert_lines.y1[0] + 0.5);
			altitude_velocity = PID(altitude_error,prev_altitude_errors);
		}
		else{
			altitude_velocity = 0;
		}
		new_vert_data = 0;
	}
	
	target_vel.velocity.x = x_rf_hold + x_rf_move;
	target_vel.velocity.y = y_rf_hold + y_rf_move;
	target_vel.velocity.z = altitude_velocity;

	ROS_INFO("target vel x = %f,  y = %f,  z = %f", target_vel.velocity.x, target_vel.velocity.y, target_vel.velocity.z);
	return target_vel;
}


int main(int argc, char **argv){
//_____________________________________________________________________________________________________#InitializeNodePubSubAndRate

    ros::init(argc, argv, "offb_node");
    ros::NodeHandle nh;

	ros::Subscriber vert_lines_sub = nh.subscribe<wall_follow::Lines>("/vert/ho/li",10,vert_lines_cb);
	ros::Subscriber hor_lines_sub = nh.subscribe<wall_follow::Lines>("/hor/ho/li",10,hor_lines_cb);

	ros::Publisher velocity_pub = nh.advertise<mavros_msgs::PositionTarget>("/mavros/setpoint_raw/local",10);
	ros::Subscriber pose_sub = nh.subscribe<geometry_msgs::PoseStamped>("mavros/local_position/pose", 10, local_pose_cb);
    	ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 10, state_cb);
	//ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 10);

    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");

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
	target_vel.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
	target_vel.type_mask = 0b111111111000;
	target_vel.position.x = 0;
	target_vel.position.y = 0;
	target_vel.position.z = 2;

	for(int i = 100; ros::ok() && i > 0; --i){
		velocity_pub.publish(target_vel);
		rate.sleep();
	}


//______________________________________________________________________________________________________________ #prepareToArmAndOffboard

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();

//__________________________________________________________________________________________________________________ #mainloop

    while(ros::ok()){
       /* if( current_state.mode != "OFFBOARD" &&
            (ros::Time::now() - last_request > ros::Duration(5.0))){
            if( set_mode_client.call(offb_set_mode) &&
                offb_set_mode.response.mode_sent){
                ROS_INFO("Offboard enabled");
            }
            last_request = ros::Time::now();
        } else {
            if( !current_state.armed &&
                (ros::Time::now() - last_request > ros::Duration(5.0))){
                if( arming_client.call(arm_cmd) &&
                    arm_cmd.response.success){
                    ROS_INFO("Vehicle armed");
                }
                last_request = ros::Time::now();
            }
        }
	*/

        ros::spinOnce();
	velocity_pub.publish(computeTargetVel());
        rate.sleep();
    }

    return 0;
}

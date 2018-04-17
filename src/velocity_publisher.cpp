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


enum Mode {follow_wall, recover_altitude};
Mode current_mode = recover_altitude;

enum Prev{Initial,Below,Above};
Prev prev = Initial;

enum Altitude_Mode{FromBelow,FromAbove,Centroid};
Altitude_Mode alt_mode = FromAbove;

const float desired_wall_dist = 5.0;
const float desired_altitude = 5.0;

float desired_buffer = 1.5;

const float altitude_entry_tolerance = 0.2;	
const float altitude_exit_tolerance = 1;	
const float confidence_threshold = 5;

const int laser_rf_offset = 0;						// lidar (Sweep) leads robot x axis (east) by 0 degrees in simulation and 0 degrees on the actual quad
									// verify and make sure that both are anticlockwise

float hold_error;				// hold errors 
float prev_hold_errors[5];
float altitude_error;
float prev_altitude_errors[5];

float hold_velocity;				// velocities 		(can be updated faster? because yaw updates are coming fast???
float altitude_velocity;
float x_rf_hold;				
float y_rf_hold;
float x_rf_move;
float y_rf_move;

PID h_pid = {0.5,0.08,0.0};			// PID parameters	
PID z_pid = {0.5,0.08,0.0};						// kp = 1.0 ; kd = 0.15;

float max_vel_h = 0.5;				// max velocities
float max_vel_z = 0.3;
float nominal_vel = 0.3;

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
			x_rf_hold = 0; y_rf_hold = 0;
			x_rf_move = 0; y_rf_move = 0;
			
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
		}
		else{
			altitude_error = 0;
			altitude_velocity = 0;
		}
		new_vert_data = 0;
	}

	// velocities:
	
	if(fabs(altitude_error) < 1.0 && fabs(hold_error) < 1.0){
		target_vel.velocity.x = x_rf_hold + x_rf_move;
		target_vel.velocity.y = y_rf_hold + y_rf_move;
	}
	else{
		target_vel.velocity.x = x_rf_hold;
		target_vel.velocity.y = y_rf_hold;
	}
	target_vel.velocity.z = altitude_velocity;

	ROS_INFO("altitude_error %f",altitude_error);
	ROS_INFO("dist_error %f", hold_error);
	ROS_INFO("target vel x = %f,  y = %f,  z = %f", target_vel.velocity.x, target_vel.velocity.y, target_vel.velocity.z);
	return target_vel;
}


void getAllParams(ros::NodeHandle n){

	//n.getParam("/sensing_range/resolution",resolution);
	/*n.getParam("/control/Kp",Kp);
	n.getParam("/control/Kd",Kd);
	n.getParam("/control/Ki",Ki);
	
	n.getParam("/control/max_velocity",max_velocity);
	n.getParam("/control/nominal_velocity",nominal_velocity);
	n.getParam("/line_region/threshold",CONFIDENCE_THRESHOLD);

	n.getParam("/flight/desired_wall_dist", desired_wall_dist);
	n.getParam("/flight/desired_altitude", desired_altitude);
	n.getParam("/sensing_range/laser_rf_offset", laser_rf_offset);

	n.getParam("/flight/NO_WALL_THRESHOLD", NO_WALL_THRESHOLD);
	n.getParam("/flight/DESIRED_MOVE_SECONDS", DESIRED_MOVE_SECONDS);
	n.getParam("/flight/DESIRED_BUFFER", DESIRED_BUFFER);
	n.getParam("/flight/LOWER_ALTITUDE_LIMIT", LOWER_ALTITUDE_LIMIT);
	n.getParam("/control/NOMINAL_Z", NOMINAL_Z);
	n.getParam("/flight/move_threshold_vertical", move_threshold_vertical);
	n.getParam("/flight/move_threshold_horizontal", move_threshold_horizontal);


	ROS_INFO("Kp: %f", Kp);
	ROS_INFO("Kd: %f", Kd);
	ROS_INFO("Ki: %f", Ki);
	ROS_INFO("max_velocity: %f",max_velocity);
	ROS_INFO("nominal_velocity: %f",nominal_velocity);
	ROS_INFO("line threshold: %d", CONFIDENCE_THRESHOLD);
	ROS_INFO("desired_wall_dist: %f",desired_wall_dist);
	ROS_INFO("desired_altitude: %f",desired_altitude);
	ROS_INFO("laser_rf_offset: %d",laser_rf_offset);
	ROS_INFO("NO_WALL_THRESHOLD: %d", NO_WALL_THRESHOLD);
	ROS_INFO("DESIRED_MOVE_SECONDS: %d", DESIRED_MOVE_SECONDS);
	ROS_INFO("DESIRED_BUFFER: %f", DESIRED_BUFFER);
	ROS_INFO("LOWER_ALTITUDE_LIMIT: %f", LOWER_ALTITUDE_LIMIT);
	ROS_INFO("NOMINAL_Z: %f", NOMINAL_Z);
	ROS_INFO("move_threshold_horizontal: %f", move_threshold_horizontal);
	ROS_INFO("move_threshold_vertical: %f", move_threshold_vertical);

	DESIRED_MOVE_iters = DESIRED_MOVE_SECONDS * 20; // desired num of seconds * velocity publishing rate
	*/
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

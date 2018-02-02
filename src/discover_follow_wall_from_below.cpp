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


// TODO
// 1. Identify switch to offboard in code
// 2. Yaw compensation
struct HOR_VEL{
	float vel_x;
	float vel_y;
};

enum MODE {DISCOVER_WALL, SETTLE_IN, NOMINAL_VELOCITY, REFIND_WALL, DO_NOTHING};
MODE mode = DISCOVER_WALL;

float desired_wall_dist = 1.7;
float desired_altitude = 1.0;

int CONFIDENCE_THRESHOLD = 17;
int laser_rf_offset = 180;				// lidar (Sweep) leads robot x axis (east) by 90 degrees in simulation and 180 degrees on the actual quad
						// verify and make sure that both are anticlockwise
float move_threshold_vertical = 1.0;			// metre
float move_threshold_horizontal = 1.0;

float LOWER_ALTITUDE_LIMIT = 2.0;
float UPPER_ALTITUDE_LIMIT = 10;
float DESIRED_BUFFER = 1.0;
float NOMINAL_Z = 0.4;
int NO_WALL_THRESHOLD = 5;
int DESIRED_MOVE_SECONDS = 10;

int DESIRED_MOVE_iters = DESIRED_MOVE_SECONDS * 20; // desired num of seconds * velocity publishing rate

int move_iters = 0;
int no_wall_count = 0;

bool new_data = 0;

float hold_error;// = 1000;		// initialize to very large number so that start off in "stay" rather than "move". Not a problem if sweep data appears before flight
float hold_velocity;
float prev_hold_errors[5];
float altitude_error;
float altitude_velocity;
float prev_altitude_errors[5];

float Kp = 1.75;
float Kd = 0.4;
float Ki = 0.1;
float max_velocity = 1.35;
float nominal_velocity = 0.5;

float Z_Kp = 1.75;
float Z_Kd = 0.4;
float Z_Ki = 0;
float max_Z_velocity = 1.0;

geometry_msgs::PoseStamped local_pose;
mavros_msgs::State current_state;
wall_follow::Lines hough_lines;

using namespace std;



void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

void local_pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg){
	local_pose = *msg;
}

void lines_cb(const wall_follow::Lines::ConstPtr& lines){
	//ROS_INFO("first line: dist:%2.4f, angle:%d, confidence:%d",lines->dist[0], lines->angle[0], lines->confidence[0]);
	hough_lines = *lines;
	
		new_data = 1;
	
}

float limit_velocity(float vel){
	if(vel > max_velocity)
		vel = max_velocity;
	else if(vel < -1 * max_velocity)
		vel = -1 * max_velocity;
	return vel;
}

float limit_Z_velocity(float vel){
	if(vel > max_Z_velocity)
		vel = max_Z_velocity;
	else if(vel < -1 * max_Z_velocity)
		vel = -1 * max_Z_velocity;
	return vel;
}

float PID(float error, float prev_error[]){
	float P = error * Kp;
	float D = (error - prev_error[0]) * Kd;
	float I = (prev_error[0] +  prev_error[1] + prev_error[2] + prev_error[3] + prev_error[4]) * Ki;
	
	//ROS_INFO("cur_error = %f, prev_error = %f, difference = %f", error, prev_error[0], error - prev_error[0]);

	prev_error[4] = prev_error[3];
	prev_error[3] = prev_error[2];
	prev_error[2] = prev_error[1];
	prev_error[1] = prev_error[0];
	prev_error[0] = error;

	
	ROS_INFO("\n P = %f    D = % f \n", P, D);
	return limit_velocity(P + I + D);
}

float Z_PID(float error, float prev_error[]){
	float P = error * Z_Kp;
	float D = (error - prev_error[0]) * Z_Kd;
	float I = (prev_error[0] +  prev_error[1] + prev_error[2] + prev_error[3] + prev_error[4]) * Z_Ki;
	
	//ROS_INFO("cur_error = %f, prev_error = %f, difference = %f", error, prev_error[0], error - prev_error[0]);

	prev_error[4] = prev_error[3];
	prev_error[3] = prev_error[2];
	prev_error[2] = prev_error[1];
	prev_error[1] = prev_error[0];
	prev_error[0] = error;

	
	ROS_INFO("\n P = %f    D = % f \n", P, D);
	return limit_Z_velocity(P + I + D);
}

HOR_VEL compute_hold_velocity(int angle, float dist, int confidence){
	int angle_rf = angle + laser_rf_offset;
	hold_error = dist - desired_wall_dist;
	if(new_data){
			if(confidence > CONFIDENCE_THRESHOLD){
				hold_velocity = PID(hold_error,prev_hold_errors);
				no_wall_count = 0;
			}
			else{									// UGLY		// make it 0 or keep the prev value???!!!
				hold_velocity = 0;
				no_wall_count++;
			}
			new_data = 0;
	}
	
	HOR_VEL hold_vel;
	hold_vel.vel_x = hold_velocity * cos (angle_rf * M_PI / 180.0);
	hold_vel.vel_y = hold_velocity * sin (angle_rf * M_PI / 180.0);
	return hold_vel;
	
}

/*float compute_yaw_rate(float angle){	// working in LIDAR frame; the idea is to have the x-axis of LIDAR frame always face the wall;
	float scaling = 60; // error of 30 degrees produces a yaw velocity of 30/60 = 0.5 rad/s = 30 degrees/second
	if(angle > 180)
		angle = angle - 360;

	return angle/scaling;	
}*/

float compute_altitude_velocity(float altitude){
	altitude_error = desired_altitude - local_pose.pose.position.z;
	//altitude_velocity = PID(altitude_error, prev_altitude_errors);
	altitude_velocity = Z_PID(altitude_error, prev_altitude_errors);
	return altitude_velocity;
}

HOR_VEL compute_move_velocity(int angle){
	int theta_rf_move = angle + laser_rf_offset - 90;
	HOR_VEL move_vel;
	move_vel.vel_x = nominal_velocity * cos (theta_rf_move * M_PI / 180.0);
	move_vel.vel_y = nominal_velocity * sin (theta_rf_move * M_PI / 180.0);
	return move_vel;
}


mavros_msgs::PositionTarget computeTargetVel(){
	mavros_msgs::PositionTarget target_vel;
	target_vel.header.stamp = ros::Time::now();
	target_vel.header.frame_id = "local_frame";
	target_vel.coordinate_frame = mavros_msgs::PositionTarget::FRAME_BODY_NED;
	target_vel.type_mask = 0b111111000111;
	target_vel.yaw_rate = 1;
	int orientation = 	(int)(tf::getYaw(local_pose.pose.orientation) * 180 / M_PI);
	

	ROS_INFO("altitude: %f", local_pose.pose.position.z);
	ROS_INFO("desired altitude: %f", desired_altitude);
	ROS_INFO("wall_dist: %f", hough_lines.dist[0]);
	ROS_INFO("confidence: %d", hough_lines.confidence[0]);
	

	// STATE MACHINE

 	if(mode == DISCOVER_WALL){
		if(local_pose.pose.position.z < UPPER_ALTITUDE_LIMIT ){		// safe altitude
			if(hough_lines.confidence[0] > CONFIDENCE_THRESHOLD){	// bridge found
				desired_altitude = local_pose.pose.position.z + DESIRED_BUFFER;
				mode = SETTLE_IN;
			}
			else{							// look for bridge
				target_vel.velocity.x = 0;
				target_vel.velocity.y = 0;
				target_vel.velocity.z = NOMINAL_Z;
			}	
		}

		else{ 								// altitude is less than LOWER_ALTITUDE_LIMIT, give up
			//mode = DO_NOTHING;	
			target_vel.velocity.x = 0;
			target_vel.velocity.y = 0;
			target_vel.velocity.z = 0;
		}	
	}

	else if(mode == SETTLE_IN){
		
		HOR_VEL hold_vel = compute_hold_velocity(hough_lines.angle[0], hough_lines.dist[0], hough_lines.confidence[0]);	// maintain desired distance

		float altitude_vel = compute_altitude_velocity(local_pose.pose.position.z);	// reach desired altitude

		target_vel.velocity.x = hold_vel.vel_x;
		target_vel.velocity.y = hold_vel.vel_y;
		target_vel.velocity.z = altitude_vel;

		if(no_wall_count > NO_WALL_THRESHOLD){
			mode = DISCOVER_WALL;
		}

		else if( (fabs(hold_error) < move_threshold_horizontal) && (fabs(altitude_error) < move_threshold_vertical) ){
			mode = NOMINAL_VELOCITY;
		}
		
	}

	else if(mode == NOMINAL_VELOCITY){
		
		if(move_iters <= DESIRED_MOVE_iters * 2){		
			move_iters++;
		}

		if(move_iters == DESIRED_MOVE_iters){	// change direction
			nominal_velocity = nominal_velocity * -1;
		}

		if(move_iters >= DESIRED_MOVE_iters * 2){	// stop
			nominal_velocity = 0;
		}

		ROS_INFO("Nom: %f", nominal_velocity);
		HOR_VEL hold_vel = compute_hold_velocity(hough_lines.angle[0], hough_lines.dist[0], hough_lines.confidence[0]);	// maintain desired distance

		float altitude_vel = compute_altitude_velocity(local_pose.pose.position.z);	// reach desired altitude

		HOR_VEL move_vel = compute_move_velocity(hough_lines.angle[0]);

		target_vel.velocity.x = hold_vel.vel_x + move_vel.vel_x;
		target_vel.velocity.y = hold_vel.vel_y + move_vel.vel_y;
		target_vel.velocity.z = altitude_vel;

		if(no_wall_count > NO_WALL_THRESHOLD){
			mode = DISCOVER_WALL;
		}

		else if( (fabs(hold_error) > move_threshold_horizontal) || (fabs(altitude_error) > move_threshold_vertical) ){
			mode = SETTLE_IN;
		}
	}

	/*else if(mode == REFIND_WALL){
		target_vel.velocity.x = 0;
		target_vel.velocity.y = 0;
		target_vel.velocity.z = 0;
	}*/

	/*else if(mode == DO_NOTHING){
		target_vel.velocity.x = 0;
		target_vel.velocity.y = 0;
		target_vel.velocity.z = 0;
	}
	*/

	else{	// SAFETY; WILL NEVER COME INTO THIS BLOCK;
		target_vel.velocity.x = 0;
		target_vel.velocity.y = 0;
		target_vel.velocity.z = 0;
	} 	// MODE == DO_NOTHING


	ROS_INFO("mode: %d", mode);
	ROS_INFO("vel x: %f", target_vel.velocity.x);
	ROS_INFO("vel y: %f", target_vel.velocity.y);
	ROS_INFO("vel z: %f\n\n\n", target_vel.velocity.z);

	return target_vel;
}


void getAllParams(ros::NodeHandle n){

	//n.getParam("/sensing_range/resolution",resolution);
	n.getParam("/control/Kp",Kp);
	n.getParam("/control/Kd",Kd);
	n.getParam("/control/Ki",Ki);
	n.getParam("/control/Z_Kp",Z_Kp);
	n.getParam("/control/Z_Kd",Z_Kd);
	n.getParam("/control/Z_Ki",Z_Ki);
	
	n.getParam("/control/max_velocity",max_velocity);
	n.getParam("/control/max_Z_velocity",max_Z_velocity);
	n.getParam("/control/nominal_velocity",nominal_velocity);
	n.getParam("/control/NOMINAL_Z", NOMINAL_Z);

	n.getParam("/line_region/threshold",CONFIDENCE_THRESHOLD);

	n.getParam("/flight/desired_wall_dist", desired_wall_dist);
	n.getParam("/flight/desired_altitude", desired_altitude);
	n.getParam("/sensing_range/laser_rf_offset", laser_rf_offset);

	n.getParam("/flight/NO_WALL_THRESHOLD", NO_WALL_THRESHOLD);
	n.getParam("/flight/DESIRED_MOVE_SECONDS", DESIRED_MOVE_SECONDS);
	n.getParam("/flight/DESIRED_BUFFER", DESIRED_BUFFER);
	n.getParam("/flight/UPPER_ALTITUDE_LIMIT", UPPER_ALTITUDE_LIMIT);
	
	n.getParam("/flight/move_threshold_vertical", move_threshold_vertical);
	n.getParam("/flight/move_threshold_horizontal", move_threshold_horizontal);


	ROS_INFO("Kp: %f", Kp);
	ROS_INFO("Kd: %f", Kd);
	ROS_INFO("Ki: %f", Ki);
	ROS_INFO("Z_Kp: %f", Z_Kp);
	ROS_INFO("Z_Kd: %f", Z_Kd);
	ROS_INFO("Z_Ki: %f", Z_Ki);

	ROS_INFO("max_velocity: %f",max_velocity);
	ROS_INFO("max_Z_velocity: %f",max_Z_velocity);
	ROS_INFO("nominal_velocity: %f",nominal_velocity);
	ROS_INFO("NOMINAL_Z: %f", NOMINAL_Z);

	ROS_INFO("line threshold: %d", CONFIDENCE_THRESHOLD);
	ROS_INFO("desired_wall_dist: %f",desired_wall_dist);
	ROS_INFO("desired_altitude: %f",desired_altitude);
	ROS_INFO("laser_rf_offset: %d",laser_rf_offset);
	ROS_INFO("NO_WALL_THRESHOLD: %d", NO_WALL_THRESHOLD);
	ROS_INFO("DESIRED_MOVE_SECONDS: %d", DESIRED_MOVE_SECONDS);
	ROS_INFO("DESIRED_BUFFER: %f", DESIRED_BUFFER);
	ROS_INFO("UPPER_ALTITUDE_LIMIT: %f", UPPER_ALTITUDE_LIMIT);
	
	ROS_INFO("move_threshold_horizontal: %f", move_threshold_horizontal);
	ROS_INFO("move_threshold_vertical: %f", move_threshold_vertical);

	DESIRED_MOVE_iters = DESIRED_MOVE_SECONDS * 20; // desired num of seconds * velocity publishing rate
}


int main(int argc, char **argv){
//_____________________________________________________________________________________________________#InitializeNodePubSubAndRate

    	ros::init(argc, argv, "discover_follow_wall");
   	ros::NodeHandle nh;
	getAllParams(nh);
	ros::Subscriber lines_sub = nh.subscribe<wall_follow::Lines>("ho/li",10,lines_cb);	// CHECK THIS!!! BUFFER MEANS OLD DATA!
	ros::Publisher velocity_pub = nh.advertise<mavros_msgs::PositionTarget>("/mavros/setpoint_raw/local",10);
	ros::Subscriber pose_sub = nh.subscribe<geometry_msgs::PoseStamped>("mavros/local_position/pose", 10, local_pose_cb);
    	ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 10, state_cb);

	//ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 10);

    	//ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
   	 //ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");

	//the setpoint publishing rate MUST be faster than 2Hz
    	ros::Rate rate(20.0);		
										
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

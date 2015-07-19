#ifndef _RC_NAV_HH_
#define _RC_NAV_HH_

#include <cmath>
#include <vector>
#include <iostream>

#include <ros/ros.h>

#include <utils.hh>
#include <rc_proc.hh>
#include <com_msgs/RC.h>
#include <cont_msgs/Heading.h>

#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>

using namespace std;

// Max norm of the velocity vector and yawing speed
double max_lin_speed,
	   max_rot_speed,
		max_x_speed,
		max_y_speed,
		max_z_speed;
// The bounding box constrains the extreme coordinates of the space that 
// rc_nav generates commands to fly towards. The first odometry reading
// defines the center of the box.
double bounding_box[8];
// Flags for the node
double refresh_rate;
bool   debug_mode;
string control_frame;	// This can be either 'world' or 'robot'.
						// In the former case the control inputs are 
						// interpreted in the world frame. Similar for the
						// latter case. The latter option is preferrable
						// for an experience similar to manual flight.

int  process_inputs(const ros::NodeHandle &n);
int  setup_messaging_interface(ros::NodeHandle &n);
int  loop(const ros::NodeHandle &n);
void odom_callback(const nav_msgs::Odometry &msg);
void imu_callback(const sensor_msgs::Imu &msg);
void rc_callback(const com_msgs::RC &msg);

#endif

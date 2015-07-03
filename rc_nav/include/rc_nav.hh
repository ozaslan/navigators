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
	   max_rot_speed;
// Flags for the node
double refresh_rate;
bool   debug_mode;

int  process_inputs(const ros::NodeHandle &n);
int  setup_messaging_interface(ros::NodeHandle &n);
int  loop(const ros::NodeHandle &n);
void odom_callback(const nav_msgs::Odometry &msg);
void imu_callback(const sensor_msgs::Imu &msg);
void rc_callback(const com_msgs::RC &msg);

#endif

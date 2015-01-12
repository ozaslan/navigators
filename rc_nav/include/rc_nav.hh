#ifndef _RC_NAV_HH_
#define _RC_NAV_HH_

#include <cmath>
#include <vector>
#include <iostream>

#include <ros/ros.h>

#include <utilities.hh>
#include <com_msgs/RC.h>
#include <cont_msgs/Heading.h>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/OccupancyGrid.h>
#include <sensor_msgs/PointCloud2.h>

using namespace std;

ros::Time curr_time;
ros::Time prev_time;
double dt;

// This is for 2D and NULL maps
double max_elevation, min_elevation;
// Radius of the spherical/circular robot approximation
double safety_margin;
// Max norm of the velocity vector and yawing speed
double max_lin_speed,
	   max_rot_speed;
// Margin around the middle of sticks inside which
// zero command is applied
double	dead_zone_x,
		dead_zone_y,
		dead_zone_z,
		dead_zone_psi;
// RC device specific values
double	min_rc_value,
		max_rc_value;

// Flags for the node
double refresh_rate;
bool   debug_mode;

int process_inputs(const ros::NodeHandle &n);
int setup_messaging_interface(ros::NodeHandle &n);
int generate_heading();
// Pose estimate (This is treated as GT by the navigator)
void odom_callback(const nav_msgs::Odometry &msg);
// RC commands
void rc_callback(const com_msgs::RC &msg);
void pcl_callback(const sensor_msgs::PointCloud2 &msg);
void gridmap_callback(const nav_msgs::OccupancyGrid &msg);
int loop(const ros::NodeHandle &n);


#endif

#include "rc_nav.hh"

#define Dt(t2, t1) (((double)t2.sec  - t1.sec ) + \
					((double)t2.nsec - t1.nsec) / 1e9);


ros::Subscriber odom_subs;
ros::Subscriber gridmap_subs;
ros::Subscriber pcl_subs;
ros::Subscriber rc_subs;
ros::Publisher  heading_publ;
// The below two are for visualization purposes only
// Has no effect on the flow of the algorithm
ros::Publisher  path_publ;
ros::Publisher  goal_publ; // odom message

nav_msgs::Odometry		 odom_msg;
nav_msgs::OccupancyGrid  gridmap_msg;
sensor_msgs::PointCloud2 pcl_msg;
cont_msgs::Heading		 heading_msg;
com_msgs::RC			 rc_msg;

bool got_odom_msg = false;

/*
-- loop

-- inside the RC_CMD callback
	-- Check if the hypotetical sphere/circle is touching the 2D/3D map
	-- Update 'to' paramteres of the cont_msgs
	-- publish
*/

int loop(const ros::NodeHandle &n);

int main(int argc, char* argv[]){

	ros::init(argc, argv, "carters_estimator");
	ros::NodeHandle n("~");

	process_inputs(n);

	setup_messaging_interface(n);
        
	loop(n);        
        
    return 1;
}

int process_inputs(const ros::NodeHandle &n)
{
	n.param("safety_margin", safety_margin, 1.0);
	n.param("min_elevation", min_elevation, 0.0);
	n.param("max_elevation", max_elevation, 1.0);
	n.param("max_lin_speed", max_lin_speed, 1.0);
	n.param("max_rot_speed", max_rot_speed, 0.50);
	n.param("min_rc_value", min_rc_value,    0.0);
	n.param("max_rc_value", max_rc_value, 2000.0);
	
	n.param("refresh_rate"    , refresh_rate    , 100.0);
	n.param("debug_mode"      , debug_mode      , false);

	n.param("dead_zone_x"   , dead_zone_x  ,  75.0); // forward
	n.param("dead_zone_y"   , dead_zone_y  ,  75.0); // lateral
	n.param("dead_zone_z"   , dead_zone_z  , 400.0); // vertical
	n.param("dead_zone_psi" , dead_zone_psi,  75.0);

	ROS_INFO(" ---------------- RC NAVIGATOR ------------------");
	ROS_INFO("[debug_mode] ----------- : [%s]", debug_mode ? "TRUE" : "FALSE");
	ROS_INFO("[refresh_rate] --------- : [%.3lf]", refresh_rate);
	ROS_INFO("[safety_margin] -------- : [%.3lf]", safety_margin);
	ROS_INFO("[min, max]_elevation --- : [%.3lf, %.3lf]", min_elevation, max_elevation);
	ROS_INFO("max_[lin, rot]_speed --- : [%.3lf, %.3lf]", max_lin_speed , max_rot_speed);
	ROS_INFO("[min, max]_rc_value ---- : [%.3lf, %.3lf]", min_rc_value , max_rc_value);
	ROS_INFO("dead_zone_[x, y, z, psi] : [%.3lf, %.3lf, %.3lf, %.3lf]", 
														  dead_zone_x, 
														  dead_zone_y,
														  dead_zone_z,
														  dead_zone_psi);
	ROS_INFO(" ------------------------------------------------");
	return 0;
}

int setup_messaging_interface(ros::NodeHandle &n)
{
	odom_subs    = n.subscribe("odom" , 10,    odom_callback, ros::TransportHints().tcpNoDelay());
	rc_subs 	 = n.subscribe("rc"   , 10,		 rc_callback, ros::TransportHints().tcpNoDelay());
	pcl_subs 	 = n.subscribe("3Dmap", 10,	    pcl_callback, ros::TransportHints().tcpNoDelay());
	gridmap_subs = n.subscribe("2Dmap", 10, gridmap_callback, ros::TransportHints().tcpNoDelay());
	heading_publ = n.advertise<cont_msgs::Heading>("heading", 10);
	path_publ	 = n.advertise<nav_msgs::Path    >("path"   , 10);
	goal_publ	 = n.advertise<nav_msgs::Odometry>("goal"   , 10);

	return 0;
}

void odom_callback(const nav_msgs::Odometry &msg){
	got_odom_msg = true;
	odom_msg = msg;
	if(debug_mode)
		ROS_INFO("RC NAV : Got ODOM message!");
}

void rc_callback(const com_msgs::RC &msg){
	// -- generate the 'heading_msg'
	// -- ### check if it is inside the map / there is any collision
	// -- generate the 'path_msg'
	// -- generate the 'goal_msg'
	// -- publish all above

	if(got_odom_msg == false)
		return;

	rc_msg = msg;

	prev_time = curr_time;
	curr_time = ros::Time::now();

	static int rc_vals_span = (max_rc_value - min_rc_value) / 2;
	static int mid_rc_value = (max_rc_value + min_rc_value) / 2;
	// Calculate the velocities
	int x_stick_pos   = rc_msg.right_fb - mid_rc_value;
	int y_stick_pos   = rc_msg.right_rl - mid_rc_value;
	int z_stick_pos   =  rc_msg.left_fb - mid_rc_value;
	int psi_stick_pos =  rc_msg.left_rl - mid_rc_value;
	// ---
	if(x_stick_pos > dead_zone_x)
		x_stick_pos -= dead_zone_x;
	else if(x_stick_pos < -dead_zone_x)
		x_stick_pos += dead_zone_x;
	else
		x_stick_pos = 0;
	// ---
	if(y_stick_pos > dead_zone_y)
		y_stick_pos -= dead_zone_y;
	else if(y_stick_pos < -dead_zone_y)
		y_stick_pos += dead_zone_y;
	else
		y_stick_pos = 0;
	// ---
	if(z_stick_pos > dead_zone_z)
		z_stick_pos -= dead_zone_z;
	else if(z_stick_pos < -dead_zone_z)
		z_stick_pos += dead_zone_z;
	else
		z_stick_pos = 0;
	// ---
	if(psi_stick_pos > dead_zone_psi)
		psi_stick_pos -= dead_zone_psi;
	else if(psi_stick_pos < -dead_zone_psi)
		psi_stick_pos += dead_zone_psi;
	else
		psi_stick_pos = 0;

	double x_vel   = (double)x_stick_pos   / (rc_vals_span - dead_zone_x) * max_lin_speed;
	double y_vel   = (double)y_stick_pos   / (rc_vals_span - dead_zone_y) * max_lin_speed;
	double z_vel   = (double)z_stick_pos   / (rc_vals_span - dead_zone_z) * max_lin_speed;
	double psi_vel = (double)psi_stick_pos / (rc_vals_span - dead_zone_x) * max_rot_speed;

	dt = Dt(curr_time, prev_time);

	heading_msg.header.stamp = curr_time;
	heading_msg.header.frame_id = "world";
	heading_msg.header.seq++;
	
	heading_msg.pos_from = odom_msg.pose.pose.position;
	heading_msg.vel_from = odom_msg.twist.twist.linear;
	heading_msg.acc_from.x = 
		heading_msg.acc_from.y = 
		heading_msg.acc_from.z = 0;
	heading_msg.quat_from  = odom_msg.pose.pose.orientation;
	heading_msg.omega_from = odom_msg.twist.twist.angular;
	heading_msg.domega_from.x = 
		heading_msg.domega_from.y = 
		heading_msg.domega_from.z = 0;
	
	heading_msg.pos_to.x = heading_msg.pos_from.x + x_vel * dt;
	heading_msg.pos_to.y = heading_msg.pos_from.y + y_vel * dt;
	heading_msg.pos_to.z = heading_msg.pos_from.z + z_vel * dt;
	heading_msg.vel_to.x = x_vel;
	heading_msg.vel_to.y = y_vel;
	heading_msg.vel_to.z = z_vel;
	heading_msg.acc_to = heading_msg.acc_from;
	// quat comes here
	Vector4d quat;
	quat(0) = heading_msg.quat_from.w;
	quat(1) = heading_msg.quat_from.x;
	quat(2) = heading_msg.quat_from.y;
	quat(3) = heading_msg.quat_from.z;
	
	Vector3d rpy = quat2rpy(quat);
	rpy(2) += dt * psi_vel;
	quat = rpy2quat(rpy);
	heading_msg.quat_to.w = quat(0);
	heading_msg.quat_to.x = quat(1);
	heading_msg.quat_to.y = quat(2);
	heading_msg.quat_to.z = quat(3);
	//
	heading_msg.omega_to.x = 
		heading_msg.omega_to.y = 0; 
	heading_msg.omega_to.z = psi_vel;
	heading_msg.domega_to = heading_msg.domega_from;

	heading_publ.publish(heading_msg);
	
	if(debug_mode)
		ROS_INFO("RC NAV : Got RC message!");
}

void pcl_callback(const sensor_msgs::PointCloud2 &msg){
	pcl_msg = msg;
	if(debug_mode)
		ROS_INFO("RC NAV : Got PCL message!");
}

void gridmap_callback(const nav_msgs::OccupancyGrid &msg){
	gridmap_msg = msg;
	if(debug_mode)
		ROS_INFO("RC NAV : Got GRIDMAP message!");
}

int loop(const ros::NodeHandle &n)
{
	ros::Rate r(refresh_rate); 

	prev_time = ros::Time::now();
	curr_time = ros::Time::now();

	ros::spin();

	return 0;
}

#undef Dt

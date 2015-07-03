#include "rc_nav.hh"

ros::Subscriber odom_subs;
ros::Subscriber imu_subs;
ros::Subscriber rc_subs;
ros::Publisher  heading_publ;

nav_msgs::Odometry odom_msg;
sensor_msgs::Imu imu_msg;
RCProc rc_proc;

/*
  - process the inputs
  - setup messaging interface
  - setup callbacks
  - loop
*/

int main(int argc, char* argv[]){

  ros::init(argc, argv, "rc_nav_node");
  ros::NodeHandle n("~");

  process_inputs(n);

  setup_messaging_interface(n);

  loop(n);        

  return 1;
}

int process_inputs(const ros::NodeHandle &n)
{
  string rc_config_path;

  n.param("max_lin_speed", max_lin_speed, 1.0);
  n.param("max_rot_speed", max_rot_speed, DEG2RAD(23));

  n.param("refresh_rate"    , refresh_rate    , 300.0);
  n.param("debug_mode"      , debug_mode      , false);
  n.param("rc_config_path"  , rc_config_path  , string(""));

  // RC Related params
  rc_proc.load_params(rc_config_path);
  rc_proc.normalize() = true;

  ROS_INFO(" ---------------- RC NAVIGATOR ------------------");
  ROS_INFO("[debug_mode] ----------- : [%s]", debug_mode ? "TRUE" : "FALSE");
  ROS_INFO("[refresh_rate] --------- : [%.3lf]", refresh_rate);
  ROS_INFO("[rc_config_path] ------- : [%s]" , rc_config_path.c_str());
  ROS_INFO("max_[lin, rot]_speed --- : [%.3lf, %.3lf]", max_lin_speed , max_rot_speed);
  rc_proc.print_params();
  ROS_INFO(" ------------------------------------------------");

  odom_msg.pose.pose.position.x = 0;
  odom_msg.pose.pose.position.y = 0;
  odom_msg.pose.pose.position.z = 0;
  odom_msg.pose.pose.orientation.w = 1;
  odom_msg.pose.pose.orientation.x = 0;
  odom_msg.pose.pose.orientation.y = 0;
  odom_msg.pose.pose.orientation.z = 0;
  odom_msg.twist.twist.linear.x = 0;
  odom_msg.twist.twist.linear.y = 0;
  odom_msg.twist.twist.linear.z = 0;
  odom_msg.twist.twist.angular.x = 0;
  odom_msg.twist.twist.angular.y = 0;
  odom_msg.twist.twist.angular.z = 0;

  return 0;
}

int setup_messaging_interface(ros::NodeHandle &n)
{
  imu_subs      = n.subscribe("imu"  , 10,  imu_callback, ros::TransportHints().tcpNoDelay());
  odom_subs     = n.subscribe("odom" , 10, odom_callback, ros::TransportHints().tcpNoDelay());
  rc_subs       = n.subscribe("rc"   , 10,   rc_callback, ros::TransportHints().tcpNoDelay());
  heading_publ  = n.advertise<cont_msgs::Heading>("heading", 10);

  return 0;
}

int loop(const ros::NodeHandle &n)
{
  ros::Rate r(refresh_rate); 

  ros::spin();

  return 0;
}

void imu_callback(const sensor_msgs::Imu &msg){
  imu_msg = msg;
  if(debug_mode)
    ROS_INFO("RC NAV : Got IMU message!");
}

void odom_callback(const nav_msgs::Odometry &msg){
  odom_msg = msg;
  if(debug_mode)
    ROS_INFO("RC NAV : Got ODOM message!");
}

void rc_callback(const com_msgs::RC &msg){
  double rc_x, rc_y, rc_z, rc_psi;
 
  rc_proc.process(msg, rc_x, rc_y, rc_z, rc_psi);

  /*
  cout << "rc_x   = " << rc_x   << endl;
  cout << "rc_y   = " << rc_y   << endl;
  cout << "rc_z   = " << rc_z   << endl;
  cout << "rc_psi = " << rc_psi << endl;
  */

  double x_vel   = rc_x;
  double y_vel   = rc_y;
  double z_vel   = rc_z;
  double psi_vel = rc_psi * max_rot_speed;

  double vel_norm = sqrt(rc_x * rc_x + rc_y * rc_y + rc_z * rc_z) + 1e-6;

  if(vel_norm > max_lin_speed){
    x_vel *= max_lin_speed / vel_norm;
    y_vel *= max_lin_speed / vel_norm;
    z_vel *= max_lin_speed / vel_norm;
  }

  static ros::Time prev_time = ros::Time::now();
  ros::Time curr_time = ros::Time::now();
  
  double dt = (curr_time - prev_time).toSec();
  prev_time = curr_time;
  static cont_msgs::Heading heading_msg;

  heading_msg.header.stamp = curr_time;
  heading_msg.header.frame_id = "world";
  heading_msg.header.seq++;

  heading_msg.pos_from = odom_msg.pose.pose.position;
  heading_msg.vel_from = odom_msg.twist.twist.linear;
  heading_msg.acc_from.x = 
    heading_msg.acc_from.y = 
    heading_msg.acc_from.z = 0;
  heading_msg.quat_from  = imu_msg.orientation;
  heading_msg.omega_from = odom_msg.twist.twist.angular;
  heading_msg.domega_from.x = 
    heading_msg.domega_from.y = 
    heading_msg.domega_from.z = 0;

  heading_msg.pos_to = heading_msg.pos_from;
  //heading_msg.pos_to.x = heading_msg.pos_from.x;
  //heading_msg.pos_to.y = heading_msg.pos_from.y;
  //heading_msg.pos_to.z = heading_msg.pos_from.z;
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

  Vector3d rpy = utils::trans::quat2rpy(quat);
  rpy(2) += dt * psi_vel;

	rpy(2) = 0; //###

  quat = utils::trans::rpy2quat(rpy);
  heading_msg.quat_to.w = quat(0);
  heading_msg.quat_to.x = quat(1);
  heading_msg.quat_to.y = quat(2);
  heading_msg.quat_to.z = quat(3);

	heading_msg.quat_from = heading_msg.quat_to; //###
  //
  heading_msg.omega_to.x = 
    heading_msg.omega_to.y = 0; 
  heading_msg.omega_to.z = psi_vel;
  heading_msg.domega_to = heading_msg.domega_from;

  heading_publ.publish(heading_msg);

  if(debug_mode)
    ROS_INFO("RC NAV : Got RC message!");
}




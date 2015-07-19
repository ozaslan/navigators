#include "rc_nav.hh"

ros::Subscriber odom_subs;
ros::Subscriber imu_subs;
ros::Subscriber rc_subs;
ros::Publisher  heading_publ;
ros::Publisher  odom_publ;

nav_msgs::Odometry odom_msg;
Eigen::Vector3d traj_pos;
double traj_yaw;
sensor_msgs::Imu imu_msg;
RCProc rc_proc;

Eigen::Matrix4d init_pose_inv;
double controller_yaw_offset = 0;
double controller_yaw = 0;

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
  n.param("max_x_speed", max_x_speed, 0.5);
  n.param("max_y_speed", max_y_speed, 2.0);
  n.param("max_z_speed", max_z_speed, 2.0);
  n.param("max_rot_speed", max_rot_speed, DEG2RAD(23));

  n.param("refresh_rate"    , refresh_rate    , 300.0);
  n.param("debug_mode"      , debug_mode      , false);
  n.param("rc_config_path"  , rc_config_path  , string(""));
  n.param("control_frame"   , control_frame   , string("world"));

  n.param("min_x_coor", bounding_box[0], -1.0);
  n.param("max_x_coor", bounding_box[1], +1.0);
  n.param("min_y_coor", bounding_box[2], -1.0);
  n.param("max_y_coor", bounding_box[3], +1.0);
  n.param("min_z_coor", bounding_box[4], -1.0);
  n.param("max_z_coor", bounding_box[5], +1.0);
  n.param("min_yaw"   , bounding_box[6], DEG2RAD(-20));
  n.param("max_yaw"   , bounding_box[7], DEG2RAD(+20));

  std::transform(control_frame.begin(), control_frame.end(), control_frame.begin(), ::tolower);

  if(control_frame != "robot" && control_frame != "world"){
    ROS_WARN("Invalid 'control_frame' parameter. Setting to default value 'world'");
    control_frame = "world";
  }

  // RC Related params
  rc_proc.load_params(rc_config_path);
  rc_proc.normalize() = true;

  ROS_INFO(" ---------------- RC NAVIGATOR ------------------");
  ROS_INFO("[debug_mode] ----------- : [%s]", debug_mode ? "TRUE" : "FALSE");
  ROS_INFO("[refresh_rate] --------- : [%.3lf]", refresh_rate);
  ROS_INFO("[rc_config_path] ------- : [%s]" , rc_config_path.c_str());
  ROS_INFO("max_[lin, rot]_speed --- : [%.3lf, %.3lf]", max_lin_speed , max_rot_speed);
  ROS_INFO("[min, max]_x_coord ----- : [%.3lf, %.3lf]", bounding_box[0] , bounding_box[1]);
  ROS_INFO("[min, max]_y_coord ----- : [%.3lf, %.3lf]", bounding_box[2] , bounding_box[3]);
  ROS_INFO("[min, max]_z_coord ----- : [%.3lf, %.3lf]", bounding_box[4] , bounding_box[5]);
  ROS_INFO("max_[x, y, z]_speed ---- : [%.3lf, %.3lf, %.3lf]", max_x_speed, max_y_speed, max_z_speed);
  ROS_INFO("[control_frame] -------- : [%s]", control_frame.c_str());
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
  odom_publ     = n.advertise<nav_msgs::Odometry>("goal", 10);

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

  double yaw = utils::trans::imu2rpy(imu_msg)(2);

  static bool first_imu_msg = true;
  if(first_imu_msg == true){
    first_imu_msg = false;
    controller_yaw_offset = yaw;
  }

  controller_yaw = utils::fix_angle(yaw - controller_yaw_offset);

}

void odom_callback(const nav_msgs::Odometry &msg){
  static bool first_odom_msg = true;
  
  Eigen::Matrix4d se3 = utils::trans::odom2se3(msg);
  Eigen::Vector4d quat;
  // Set the initial yaw with the very first odometry message
  if(first_odom_msg == true){
      init_pose_inv = se3.inverse();
      init_pose_inv.topRightCorner<3, 1>().fill(0);

      traj_pos(0) = msg.pose.pose.position.x;
      traj_pos(1) = msg.pose.pose.position.y;
      traj_pos(2) = msg.pose.pose.position.z;
      traj_yaw  = utils::trans::quat2rpy(utils::trans::quat2quat(msg.pose.pose.orientation))(2);

      first_odom_msg = false;
      bounding_box[0] += se3(0, 3);
      bounding_box[1] += se3(0, 3);
      bounding_box[2] += se3(1, 3);
      bounding_box[3] += se3(1, 3);
      bounding_box[4] += se3(2, 3);
      bounding_box[5] += se3(2, 3);
  }

  odom_msg = utils::trans::se32odom(Eigen::Matrix4d(init_pose_inv * se3));

  // Transform the velocity into the initial frame
  Eigen::Vector3d vel;
  vel(0) = odom_msg.twist.twist.linear.x;
  vel(1) = odom_msg.twist.twist.linear.y;
  vel(2) = odom_msg.twist.twist.linear.z;
  vel = init_pose_inv.topLeftCorner<3, 3>() * vel;
  odom_msg.twist.twist.linear.x = vel(0);
  odom_msg.twist.twist.linear.y = vel(1);
  odom_msg.twist.twist.linear.z = vel(2);

  odom_msg.twist.twist.angular = msg.twist.twist.angular;

  if(debug_mode)
    ROS_INFO("RC NAV : Got ODOM message!");
}

void rc_callback(const com_msgs::RC &msg){
  double rc_x, rc_y, rc_z, rc_psi;

  static ros::Time prev_time = msg.header.stamp;
  ros::Time curr_time = msg.header.stamp;
  double dt = (curr_time - prev_time).toSec();
  prev_time = curr_time;

  rc_proc.process(msg, rc_x, rc_y, rc_z, rc_psi);

  double x_vel   = rc_x;
  double y_vel   = rc_y;
  double z_vel   = rc_z;
  double psi_vel = rc_psi * max_rot_speed;

  if(control_frame == "robot"){
    // Transform velocity vector into "robot" frame
    Eigen::Vector3d vel(x_vel, y_vel, z_vel);
    Eigen::Matrix3d dcm = utils::trans::odom2se3(odom_msg).topLeftCorner<3, 3>();
    vel = dcm.transpose() * vel;
    x_vel = vel(0);
    y_vel = vel(1);
    z_vel = vel(2);
  }

  double vel_norm = sqrt(rc_x * rc_x + rc_y * rc_y + rc_z * rc_z) + 1e-6;

	x_vel = utils::clamp(x_vel, -max_x_speed, max_x_speed);
	y_vel = utils::clamp(y_vel, -max_y_speed, max_y_speed);
	z_vel = utils::clamp(z_vel, -max_z_speed, max_z_speed);

  if(vel_norm > max_lin_speed){
    x_vel *= max_lin_speed / vel_norm;
    y_vel *= max_lin_speed / vel_norm;
    z_vel *= max_lin_speed / vel_norm;
  }
 
  static cont_msgs::Heading heading_msg;

  heading_msg.header.stamp = curr_time;
  heading_msg.header.frame_id = "world";
  heading_msg.header.seq++;

  // -------------------------------------------------------------------------------------------- //
  // Correct the yaw angle to controller yaw (funny :) )
  Vector3d rpy = utils::trans::quat2rpy(utils::trans::quat2quat(odom_msg.pose.pose.orientation));
  rpy(2) = controller_yaw;
  odom_msg.pose.pose.orientation = utils::trans::quat2quat(utils::trans::rpy2quat(rpy));
  // -------------------------------------------------------------------------------------------- //

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

  //heading_msg.pos_to = heading_msg.pos_from;

  traj_pos(0) = utils::clamp(traj_pos(0) + dt * x_vel, bounding_box[0], bounding_box[1]);
  traj_pos(1) = utils::clamp(traj_pos(1) + dt * y_vel, bounding_box[2], bounding_box[3]);
  traj_pos(2) = utils::clamp(traj_pos(2) + dt * z_vel, bounding_box[4], bounding_box[5]);

  heading_msg.pos_to.x = traj_pos(0);
  heading_msg.pos_to.y = traj_pos(1);
  heading_msg.pos_to.z = traj_pos(2);
  heading_msg.vel_to.x = x_vel;
  heading_msg.vel_to.y = y_vel;
  heading_msg.vel_to.z = z_vel;
  heading_msg.acc_to = heading_msg.acc_from;

  //heading_msg.quat_to = heading_msg.quat_from;
  traj_yaw = utils::clamp(traj_yaw + dt * psi_vel, bounding_box[6], bounding_box[7]);
  rpy = utils::trans::quat2rpy(utils::trans::quat2quat(heading_msg.quat_from));
  rpy(2) = traj_yaw;
  heading_msg.quat_to = utils::trans::quat2quat(utils::trans::rpy2quat(rpy));

  heading_msg.omega_to.x = 
    heading_msg.omega_to.y = 0; 
  heading_msg.omega_to.z = psi_vel;
  heading_msg.domega_to = heading_msg.domega_from;

  heading_publ.publish(heading_msg);

  static nav_msgs::Odometry goal;
  goal.header.seq++;
  goal.header.stamp = curr_time;
  goal.header.frame_id = "world";

  goal.pose.pose.position    = heading_msg.pos_to;
  goal.pose.pose.orientation = heading_msg.quat_to;
  goal.twist.twist.linear    = heading_msg.vel_to;
  goal.twist.twist.angular   = heading_msg.omega_to;

  odom_publ.publish(goal);

  if(debug_mode)
    ROS_INFO("RC NAV : Got RC message!");
}

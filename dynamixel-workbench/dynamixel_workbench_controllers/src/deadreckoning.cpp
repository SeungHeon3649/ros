#include "deadreckoning.h"

Deadreckoning::Deadreckoning():
    last_vel_time_(0),
    vel_dt_(0),
    x_pos_(0),
    y_pos_(0),
    heading_(0),
    init_encoder(true)
{
    odom_publisher_ = nh_.advertise<nav_msgs::Odometry>("odom", 50);
    imu_sub = nh_.subscribe<sensor_msgs::Imu>("imu", 100, &Deadreckoning::imu_callback,this);
    dynamixel_state_sub = nh_.subscribe<dynamixel_workbench_msgs::DynamixelStateList>("dynamixel_workbench/dynamixel_state", 100, &Deadreckoning::state_callback,this);
}
void Deadreckoning::updateMotorInfo(int32_t left_tick, int32_t right_tick)
{
  int32_t current_tick = 0;
  
  if (init_encoder)
  {
    for (int index = 0; index < 2; index++)
    {
      last_diff_tick[index] = 0;
      last_tick[index]      = 0;
      last_rad[index]       = 0.0;

      last_velocity[index]  = 0.0;
    }  

    last_tick[LEFT] = left_tick;
    last_tick[RIGHT] = right_tick;

    init_encoder = false;
    return;
  }

  current_tick = left_tick;

  last_diff_tick[LEFT] = current_tick - last_tick[LEFT];
  last_tick[LEFT]      = current_tick;
  last_rad[LEFT]       += TICK2RAD * (double)last_diff_tick[LEFT];

  current_tick = right_tick;

  last_diff_tick[RIGHT] = current_tick - last_tick[RIGHT];
  last_tick[RIGHT]      = current_tick;
  last_rad[RIGHT]       += TICK2RAD * (double)last_diff_tick[RIGHT];
}
void Deadreckoning::imu_callback(const sensor_msgs::Imu::ConstPtr& msg){
  q_t2.x = msg->orientation.x;
  q_t2.y = msg->orientation.y;
  q_t2.z = msg->orientation.z;
  q_t2.w = msg->orientation.w;
}
void Deadreckoning::state_callback(const dynamixel_workbench_msgs::DynamixelStateList::ConstPtr& msg)
{
    ros::Time current_time = ros::Time::now();
    double wheel_l, wheel_r;      // rotation value of wheel [rad]
    double delta_s, theta, delta_theta;
    static double last_theta = 0.0;
    double v, w;      
    wheel_l = wheel_r = 0.0;
    delta_s = delta_theta = theta = 0.0;
    v = w = 0.0;
    vel_dt_ = (current_time - last_vel_time_).toSec();
    //ROS_INFO("vel_dt = %f", vel_dt_);
    last_vel_time_ = current_time;

    updateMotorInfo(msg->dynamixel_state[0].present_position,msg->dynamixel_state[1].present_position);

    if (vel_dt_ == 0)
      return;

    wheel_l = TICK2RAD * (double)last_diff_tick[LEFT];
    wheel_r = -1*TICK2RAD * (double)last_diff_tick[RIGHT];

    if (isnan(wheel_l))
      wheel_l = 0.0;

    if (isnan(wheel_r))
      wheel_r = 0.0;

    delta_s     = WHEEL_RADIUS * (wheel_r + wheel_l) / 2.0;

    // theta       = -1*atan2f(q_t2.x*q_t2.y + q_t2.w*q_t2.z, 
    //               0.5f - q_t2.y*q_t2.y - q_t2.z*q_t2.z);
    
    theta = WHEEL_RADIUS * (wheel_r - wheel_l) / WHEEL_SEPARATION;

    delta_theta = theta;
    // delta_theta = theta - last_theta;
    odom_pose[0] = delta_s * cos(heading_ + (delta_theta / 2.0));
    odom_pose[1] = delta_s * sin(heading_ + (delta_theta / 2.0));
    odom_pose[2] = delta_theta;

    x_pos_ += odom_pose[0];
    y_pos_ += odom_pose[1];
    heading_ += odom_pose[2];

    v = delta_s / vel_dt_;
    w = delta_theta / vel_dt_;

    odom_vel[0] = v;
    odom_vel[1] = 0.0;
    odom_vel[2] = w;

    last_velocity[LEFT]  = wheel_l / vel_dt_;
    last_velocity[RIGHT] = wheel_r / vel_dt_;
    last_theta = theta;

    odom_quat = tf::createQuaternionMsgFromYaw(heading_);

    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_footprint";
    //robot's position in x,y, and z
    odom_trans.transform.translation.x = x_pos_;
    odom_trans.transform.translation.y = y_pos_;
    odom_trans.transform.translation.z = 0.0;
    //robot's heading in quaternion
    odom_trans.transform.rotation = odom_quat;
    odom_trans.header.stamp = current_time;
    //publish robot's tf using odom_trans object
    odom_broadcaster_.sendTransform(odom_trans);

    odom.header.stamp = current_time;
    odom.header.frame_id = "odom";
    odom.child_frame_id = "base_footprint";

    //robot's position in x,y, and z
    odom.pose.pose.position.x = x_pos_;
    odom.pose.pose.position.y = y_pos_;
    odom.pose.pose.position.z = 0.0;
    //robot's heading in quaternion
    odom.pose.pose.orientation = odom_quat;
    odom.pose.covariance[0] = 0.001;
    odom.pose.covariance[7] = 0.001;
    odom.pose.covariance[35] = 0.001;

    //linear speed from encoders
    odom.twist.twist.linear.x = odom_vel[0];
    odom.twist.twist.linear.y = odom_vel[1];
    odom.twist.twist.linear.z = 0.0;

    odom.twist.twist.angular.x = 0.0;
    odom.twist.twist.angular.y = 0.0;
    //angular speed from encoders
    odom.twist.twist.angular.z = odom_vel[2];
    odom.twist.covariance[0] = 0.0001;
    odom.twist.covariance[7] = 0.0001;
    odom.twist.covariance[35] = 0.0001;

    odom_publisher_.publish(odom);
}
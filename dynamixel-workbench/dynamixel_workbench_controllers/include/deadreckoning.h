#ifndef LINO_BASE_H
#define LINO_BASE_H
#define WHEEL_RADIUS                     0.033           // meter
#define WHEEL_SEPARATION                 0.287           // meter (BURGER : 0.160, WAFFLE : 0.287)
#define TICK2RAD                         0.001533981  // 0.087890625[deg] * 3.14159265359 / 180 = 0.001533981f
#define LEFT                        0
#define RIGHT                       1

#include <dynamixel_workbench_msgs/DynamixelStateList.h>
#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/TransformStamped.h>

class Deadreckoning
{
public:
    Deadreckoning();
    void imu_callback(const sensor_msgs::Imu::ConstPtr& msg);
    void state_callback(const dynamixel_workbench_msgs::DynamixelStateList::ConstPtr& msg);
    void updateMotorInfo(int32_t left_tick, int32_t right_tick);
private:
    ros::NodeHandle nh_;
    ros::Publisher odom_publisher_;
    ros::Subscriber imu_sub;
    ros::Subscriber dynamixel_state_sub;
    tf::TransformBroadcaster odom_broadcaster_;
    geometry_msgs::Quaternion odom_quat;
    geometry_msgs::TransformStamped odom_trans;
    nav_msgs::Odometry odom;
    geometry_msgs::Quaternion q_t2;

    bool init_encoder;
    int32_t last_diff_tick[2];
    int32_t last_tick[2];
    double last_rad[2];
    double last_velocity[2];
    double odom_pose[3];
    double odom_vel[3];
    ros::Time last_vel_time_;
    float vel_dt_;
    float x_pos_;
    float y_pos_;
    float heading_;
};

#endif
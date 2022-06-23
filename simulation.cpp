#include <iostream>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <nav_msgs/Odometry.h>
#include <tf/tf.h>
#include <tf/transform_datatypes.h>
using namespace std;

ros::Publisher move_pub;
geometry_msgs::Twist vel;
double goal_pose_x, goal_pose_y;
double distance_tolerance;
void chatterCallback(const nav_msgs::Odometry::ConstPtr& odom)
{
  double x = odom->pose.pose.position.x;
  double y = odom->pose.pose.position.y;
  double quatx= odom->pose.pose.orientation.x;
  double quaty= odom->pose.pose.orientation.y;
  double quatz= odom->pose.pose.orientation.z;
  double quatw= odom->pose.pose.orientation.w;
  tf::Quaternion q(quatx, quaty, quatz, quatw);
  tf::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);
  double theta = yaw;
  //Yaw 값을 받아오는 또 다른 방법
  //theta = tf::getYaw(odom->pose.pose.orientation);
  double euclidean_distance;
  euclidean_distance = (sqrt((goal_pose_x-x)*(goal_pose_x-x)
  +(goal_pose_y-y)*(goal_pose_y-y)));
  if(euclidean_distance > 0.5) euclidean_distance = 0.5;
  if((euclidean_distance >= distance_tolerance))
  {
    vel.linear.x = euclidean_distance;
    vel.angular.z = atan2(goal_pose_y - y, goal_pose_x - x) - theta;
    cout << "선속도 : " << vel.linear.x << endl;
    cout << "각속도 : " << vel.angular.z <<endl;
    cout << endl;
  }
  else
  {
    vel.linear.x = 0;
    vel.angular.z = 0;
  }
  move_pub.publish(vel);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "drive_base");
  ros::NodeHandle nh;
  vel.linear.x = vel.linear.y = vel.linear.z = 0;
  vel.angular.x = vel.angular.y = vel.linear.z = 0;
  cout << "Set your x goal : ";
  cin >> goal_pose_x;
  cout << "Set your y goal : "; 
  cin >> goal_pose_y;
  cout << "Set your tolerance : ";
  cin >> distance_tolerance;
  move_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
  ros::Subscriber sub = nh.subscribe("/odom", 10, chatterCallback);
  ros::spin();
}

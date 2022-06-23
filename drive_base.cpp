#include <iostream>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <turtlesim/Pose.h>
#include <geometry_msgs/Pose.h>
using namespace std;

ros::Publisher cmd_vel_pub_;
geometry_msgs::Twist base_cmd;
double goal_pose_x, goal_pose_y;
double distance_tolerance;
turtlesim::PoseConstPtr g_pose;
void chatterCallback(const turtlesim::PoseConstPtr& pose)
{
  g_pose = pose;
  double euclidean_distance;
  euclidean_distance = sqrt((goal_pose_x-g_pose->x)*(goal_pose_x-g_pose->x)
  +(goal_pose_y-g_pose->y)*(goal_pose_y-g_pose->y));
  if(euclidean_distance >= distance_tolerance)
  {
    base_cmd.linear.x = 1.5 * euclidean_distance;
    base_cmd.angular.z = 6*(atan2(goal_pose_y - g_pose->y, goal_pose_x - g_pose->x)-g_pose->theta);
  }
  else
  {
    base_cmd.linear.x = 0;
    base_cmd.angular.z = 0;
  }
  cout <<base_cmd.angular.z <<endl;
  cmd_vel_pub_.publish(base_cmd);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "robot_driver");
  ros::NodeHandle nh;
  base_cmd.linear.x = base_cmd.linear.y = base_cmd.angular.z = 0;
  cout << "Set your x goal : ";
  cin >> goal_pose_x;
  cout << "Set your y goal : "; 
  cin >> goal_pose_y;
  cout << "Set your tolerance : ";
  cin >> distance_tolerance;
  cmd_vel_pub_ = nh.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 1);
  ros::Subscriber sub = nh.subscribe("/turtle1/pose", 10, chatterCallback);
  ros::spin();
  
}

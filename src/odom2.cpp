#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <gazebo_msgs/ModelStates.h>
#include <sensor_msgs/JointState.h>
#include <iostream>

using namespace std;

ros::Publisher G_publisher_odom;
nav_msgs::Odometry G_odom;
double G_totalX = 0.0, G_totalY = 0.0;
double G_XB = 0.0, G_YB = 0.0;
ros::Time G_wall_begin;

void cbVel(const geometry_msgs::Twist::ConstPtr& vel) {
  cout << "Linear :" << vel->linear.x << endl;
  cout << "Angular:" << vel->angular.z << endl;
}

void cbOdom(const nav_msgs::Odometry::ConstPtr& msg)
{
  ROS_INFO("Seq: %d", msg->header.seq);
  ROS_INFO("/odom Pos (x:%f, y:%f, z:%f)", msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);

  tf::Quaternion q(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);  
  tf::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);

  ROS_INFO("/odom Pose (roll:%f, pitch:%f, yaw:%f) ", roll, pitch, yaw);
  ROS_INFO("Vel (Linear:%f, Angular:%f)", msg->twist.twist.linear.x, msg->twist.twist.angular.z);             
}

void cbModelStates(const gazebo_msgs::ModelStates::ConstPtr& msg)
{
  ROS_INFO("Real Pos (x:%f, y:%f, z:%f)", msg->pose[1].position.x, msg->pose[1].position.y, msg->pose[1].position.z);

  tf::Quaternion q(msg->pose[1].orientation.x, msg->pose[1].orientation.y, msg->pose[1].orientation.z, msg->pose[1].orientation.w);
  tf::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);

  ROS_INFO("Real Pose (roll:%f, pitch:%f, yaw:%f) ", roll, pitch, yaw);             
}

void cbMyOdom(const sensor_msgs::JointState::ConstPtr& jointstate)
{
  double wheel_right_joint_pos = jointstate->position[0];
  double wheel_left_joint_pos  = jointstate->position[1];

  double nt, positionr, positionl, positionrb, positionlb, velr, vell, vel, positionx, positiony, s, X, Y, Vx, Vy, W;
  
  ros::Time wall_now = ros::Time::now();

  static int counter = 0;

  if(counter <= 0){
    positionrb = jointstate->position[0];
    positionlb = jointstate->position[1];
    counter++;
  } else {
    nt = wall_now.toSec() - G_wall_begin.toSec();
    positionr = (jointstate->position[0] - positionrb) / nt;
    positionl = (jointstate->position[1] - positionlb) / nt;

    velr = positionr * 0.033;
    vell = positionl * 0.033;
    vel  = (velr + vell) / 2;

    W    = (vell - velr) / 0.16;

    s    = W * nt;

    Vx   = vel * sin(s);
    Vy   = vel * cos(s);

    X    = Vx * nt;
    Y    = Vy * nt;

    G_totalX = 0.5 * (X + G_XB) * nt + G_totalX;
    G_totalY = 0.5 * (Y + G_YB) * nt + G_totalY;

    positionrb = jointstate->position[0];
    positionlb = jointstate->position[1];

    G_odom.pose.pose.position.x = G_totalX;
    G_odom.pose.pose.position.y = G_totalY;

    G_XB = X;
    G_YB = Y;    

    G_wall_begin = ros::Time::now();

    G_publisher_odom.publish(G_odom);
  }

  ROS_INFO("Whell Pos (r:%f, l:%f)", wheel_right_joint_pos, wheel_left_joint_pos);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "odom");
  ros::NodeHandle nh;

  ros::Subscriber sub  = nh.subscribe("/cmd_vel", 10, cbVel);
  ros::Subscriber sub2 = nh.subscribe("/odom", 100, cbOdom);
  ros::Subscriber sub3 = nh.subscribe("/gazebo/model_states", 100, cbModelStates);
  ros::Subscriber sub4 = nh.subscribe("/joint_states", 100, cbMyOdom);
  
  G_publisher_odom = nh.advertise<nav_msgs::Odometry>("/dead_reckoning", 10);

  ros::Rate rate(100);

  G_wall_begin = ros::Time::now();
  ros::spin();

  return 0;
}

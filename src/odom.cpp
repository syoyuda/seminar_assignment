// ファイル名　my_odom3.cpp
#include <ros/ros.h>  // rosで必要はヘッダーファイル
#include <geometry_msgs/Twist.h> // ロボットを動かすために必要
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <gazebo_msgs/ModelStates.h>
#include <sensor_msgs/JointState.h>
#include <iostream>



using namespace std;
ros::Publisher G_publisher_odom;
nav_msgs::Odometry G_odom;
ros::Time G_wall_begin ;

// コールバック関数。並進、回転速度の表示。
void cbVel(const geometry_msgs::Twist::ConstPtr& vel) {
  cout << "Linear :" << vel->linear.x << endl;
  cout << "Angular:" << vel->angular.z << endl;
}


// /odomトピックから位置posと姿勢poseを表示
void cbOdom(const nav_msgs::Odometry::ConstPtr& msg)
{
  ROS_INFO("Seq: %d", msg->header.seq);
  ROS_INFO("/odom Pos (x:%f, y:%f, z:%f)", msg->pose.pose.position.x,msg->pose.pose.position.y, msg->pose.pose.position.z);

  tf::Quaternion q(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);  
  // tf::Quaternion q(quat.x, quat.y, quat.z, quat.w);
  tf::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);

  ROS_INFO("/odom Pose (roll:%f, pitch:%f, yaw:%f) ", roll, pitch, yaw);
  ROS_INFO("Vel (Linear:%f, Angular:%f)", msg->twist.twist.linear.x,msg->twist.twist.angular.z);
              
}

// /gazebo/model_statesトピックから真の位置Pos(x,y,z)と姿勢Pose(roll, pitch ,yaw)を表示
void cbModelStates(const gazebo_msgs::ModelStates::ConstPtr& msg)
{
  ROS_INFO("Real Pos (x:%f, y:%f, z:%f)", msg->pose[1].position.x,msg->pose[1].position.y, msg->pose[1].position.z);

  tf::Quaternion q(msg->pose[1].orientation.x, msg->pose[1].orientation.y, msg->pose[1].orientation.z, msg->pose[1].orientation.w);  
  // tf::Quaternion q(quat.x, quat.y, quat.z, quat.w);
  tf::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);

  ROS_INFO("Real Pose (roll:%f, pitch:%f, yaw:%f) ", roll, pitch, yaw);
              
}

// cbMyOdom：この関数に自分のオドメトリを実装しよう！
// /joint_statesトピックから左右のjoint(車輪回転軸)の位置（回転角度)[rad]を表示
// 参考：Turtlebot3の車輪直径0.066 [m]
void cbMyOdom(const sensor_msgs::JointState::ConstPtr& jointstate)
{
  double wheel_right_joint_pos = jointstate->position[0]; // 右車軸の位置[rad]
  double wheel_left_joint_pos  = jointstate->position[1]; // 左車軸の位置[rad]

  //自分で変更したところ
  double nt, positionr, positionl, positionrb, positionlb, velr, vell, vel, positionx, positiony, s, X, Y, Vx, Vy, W;
  ros::Time wall_now = ros::Time::now();
  

  if(wall_now.toSec() <= 0){
    positionrb = jointstate->position[0];
    positionlb = jointstate->position[1];
  }else{

    nt = wall_now.toSec() - G_wall_begin.toSec();  // dt
    positionr = (jointstate->position[0] - positionrb) /nt;  // 右車輪を微分してる　→車輪の回転速度
    positionl = (jointstate->position[1] - positionlb) /nt;   // 左車輪を微分してる
    
    velr = positionr * 0.033 ;                   // 回転速度と半径をかけて車輪の進行速度を求めてる
    vell = positionl * 0.033;
    vel  = (velr + vell) / 2;                    //  車体の併進速度

    W    = (vell - velr) / 0.158;                    // 車輪の進行速度で車体本体の回転速度を求めてる

    s    = W * nt;                               // 車体のθを回転速度を微分して求めてる

    Vx   = vel * sin(s);                         // 車体のx軸の速度に分解してる
    Vy   = vel * cos(s);

    X    = Vx * nt;                              // 位置
    Y    = Vy * nt;

    G_odom.pose.pose.position.x += X;
    G_odom.pose.pose.position.y += Y;
    G_wall_begin = wall_now;

    G_publisher_odom.publish(G_odom);
  }
    
    


  
  
  
  // 車軸の位置は積算される
  ROS_INFO("Whell Pos (r:%f, l:%f)", wheel_right_joint_pos,wheel_left_joint_pos);
}




int main(int argc, char **argv)
{
  ros::init(argc, argv, "my_odom3");
  ros::NodeHandle nh;

  //subscriberの作成。トピック/cmd_velを購読する。
  ros::Subscriber sub  = nh.subscribe("/cmd_vel", 10, cbVel);
  ros::Subscriber sub2 = nh.subscribe("/odom", 100, cbOdom);
  ros::Subscriber sub3 = nh.subscribe("/gazebo/model_states", 100, cbModelStates);
  ros::Subscriber sub4 = nh.subscribe("/joint_states", 100, cbMyOdom);
  
  G_publisher_odom = nh.advertise<nav_msgs::Odometry>("/dead_reckoning", 10);

  
  // コールバック関数を繰り返し呼び出す。
  ros::Rate rate(100);

  G_wall_begin = ros::Time::now();
  ros::spin();

  return 0;
}
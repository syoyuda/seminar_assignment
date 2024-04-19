// ファイル名　my_odom3.cpp
#include <ros/ros.h>  // rosで必要はヘッダーファイル
#include <geometry_msgs/Twist.h> // ロボットを動かすために必要
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <gazebo_msgs/ModelStates.h> 
#include <sensor_msgs/JointState.h>
#include <iostream>
#include <stdbool.h>



using namespace std;
ros::Publisher G_publisher_odom;
nav_msgs::Odometry G_odom;
ros::Time G_wall_previous;
double G_totalX = 0, G_totalY = 0, G_X=0, G_Y=0;
double G_XB = 0, G_YB = 0;                                  //ここで初期位置を0として考えている
double G_positionrb, G_positionlb;
double G_dttotal=0;
bool G_is_first = true;
double G_s = 0;

// コールバック関数。並進、回転速度の表示。
void cbVel(const geometry_msgs::Twist::ConstPtr& vel) {
  //cout << "Linear :" << vel->linear.x << endl;
  //cout << "Angular:" << vel->angular.z << endl;
}


// /odomトピックから位置posと姿勢poseを表示
void cbOdom(const nav_msgs::Odometry::ConstPtr& msg)
{
  //ROS_INFO("Seq: %d", msg->header.seq);
  //ROS_INFO("/odom Pos (x:%f, y:%f, z:%f)", msg->pose.pose.position.x,msg->pose.pose.position.y, msg->pose.pose.position.z);

  tf::Quaternion q(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);  
  // tf::Quaternion q(quat.x, quat.y, quat.z, quat.w);
  tf::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);

  //ROS_INFO("/odom Pose (roll:%f, pitch:%f, yaw:%f) ", roll, pitch, yaw);
  //ROS_INFO("Vel (Linear:%f, Angular:%f)", msg->twist.twist.linear.x,msg->twist.twist.angular.z);
												  
}

// /gazebo/model_statesトピックから真の位置Pos(x,y,z)と姿勢Pose(roll, pitch ,yaw)を表示
void cbModelStates(const gazebo_msgs::ModelStates::ConstPtr& msg)
{
  //ROS_INFO("Real Pos (x:%f, y:%f, z:%f)", msg->pose[1].position.x,msg->pose[1].position.y, msg->pose[1].position.z);

  tf::Quaternion q(msg->pose[1].orientation.x, msg->pose[1].orientation.y, msg->pose[1].orientation.z, msg->pose[1].orientation.w);  
  // tf::Quaternion q(quat.x, quat.y, quat.z, quat.w);
  tf::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);

  //ROS_INFO("Real Pose (roll:%f, pitch:%f, yaw:%f) ", roll, pitch, yaw);
												  
}
















// cbMyOdom：この関数に自分のオドメトリを実装しよう！
// /joint_statesトピックから左右のjoint(車輪回転軸)の位置（回転角度)[rad]を表示
// 参考：Turtlebot3の車輪直径0.066 [m]
void cbMyOdom(const sensor_msgs::JointState::ConstPtr& jointstate)
{
  double wheel_right_joint_pos = jointstate->position[0]; // 右車軸の位置[rad]
  double wheel_left_joint_pos  = jointstate->position[1]; // 左車軸の位置[rad]

   //自分で変更したところ
  double dt, positionr, positionl, velr, vell, vel, positionx, positiony, X, Y, Vx, Vy, W;
                                       
  ros::Time wall_now = ros::Time::now();



  if(G_is_first == true){               //0から最初の処理までの移動距離を無視して計算してる → 改善点

    G_positionrb = jointstate->position[0];
    G_positionlb = jointstate->position[1];
    G_wall_previous = ros::Time::now();

    G_is_first = false;

  }else{

    //ROS_INFO("time333333 (now:%lf,begin:%lf)",wall_now.toSec(),G_wall_previous.toSec()); //実際に時間がどうなっているのか確認
    
    dt =wall_now.toSec() - G_wall_previous.toSec();
    //ROS_INFO("dt:%lf",dt);
    G_wall_previous = ros::Time::now();


    positionr = (jointstate->position[0] - G_positionrb) /dt;  // 右車輪を微分してる　→車輪の回転速度
    positionl = (jointstate->position[1] - G_positionlb) /dt;   // 左車輪を微分してる
    

    velr = positionr * 0.033;                   // 回転速度と半径をかけて車輪の進行速度を求めてる
    vell = positionl * 0.033;
    vel  = (velr + vell) / 2;                    //  車体の併進速度

    W    = (vell - velr) / 0.16;                    // 車輪の進行速度で車体本体の回転速度を求めてる

    G_s  = W * dt + G_s;                       // 車体のθを回転速度を積分して求めてる

    Vx   = vel * cos(G_s);                         // 車体のx軸の速度に分解してる
    Vy   = vel * sin(G_s);

    X    = Vx * dt;                              // 位置
    Y    = Vy * dt;

    G_X = G_X + X;
    G_Y = G_Y + Y;

    G_totalX = 0.5 * (X + G_XB) * dt + G_totalX;             //台形近似
    G_totalY = 0.5 * (Y + G_YB) * dt + G_totalY;

    G_positionrb = jointstate->position[0];                   //ポジションの更新
    G_positionlb = jointstate->position[1];

    G_odom.pose.pose.position.x = G_X;               //G_odomのなかの.poseの.poseの.positionの.xにtotalXを送ってる？
    G_odom.pose.pose.position.y = G_Y;

    G_XB = X;                                             //台形近似のための前の位置を保存してる
    G_YB = Y;    

    //ROS_INFO("time (dt:%lf)", dt);


    G_publisher_odom.publish(G_odom);                   //G_odomをG_publisher_odomのグローバル関数に送ってる
  }
    
  
  // 車軸の位置は積算される
  //ROS_INFO("Whell Pos (r:%f, l:%f)", wheel_right_joint_pos,wheel_left_joint_pos);
  ROS_INFO("Position (x:%lf, y:%lf)", G_X, G_Y);
  //ROS_INFO("jointstate (x:%lf, y:%lf)", jointstate->position[0], jointstate->position[1]);
  //ROS_INFO("time (now:%lf,begin:%lf)",wall_now.toSec(),G_wall_previous.toSec());
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
  
  G_publisher_odom = nh.advertise<nav_msgs::Odometry>("/dead_reckoning", 10);       /*上の計算で求めた台形近似の数値を  
                                                  G_publisher_odomに入れたものを/dead_reckoningのtopicで流してるはず*/

  
  // コールバック関数を繰り返し呼び出す。
  ros::Rate rate(100);

  ros::spin();

  


  return 0;
}
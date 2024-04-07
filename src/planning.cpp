/* -------------------------
  FFのみの制御：動作計画
  callbackを未使用
  2022.4.8 by K. Hidaka
-----------------------------*/
#include "ros/ros.h"  // rosで必要なヘッダファイル
#include <geometry_msgs/Twist.h> // ロボットを動かすために必要
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h> //初期位置関係
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <time.h> //計測時間
#include "unistd.h"
#include<iostream> //C++用入出力関係
#include<fcntl.h> //ファイル読み書き用
#include<math.h> //計算用
#define DATA_FILE_PATH "./test_data.txt" //保存データパス設定
using namespace std; // C++の命令 cout << に必要

/*
  ロボットの移動命令部分
  sampleでは並進速度 = v_vel/回転速度 a_velを事前に設定
*/
int main(int argc, char **argv)
{
  /* ------------------------------
            初期設定
--------------------------------*/
    ros::init(argc, argv, "my_robot_cont2");
    ros::NodeHandle nh; // 送受信用
    ros::Publisher pub;
    geometry_msgs::Twist vel;
    pub= nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);


    double linear_vel = 0.0; // 速度[m/s]
    double angular_vel = 0.0; //回転速度[rad/s]
    ros::WallDuration w_t_t; //WallTimeは壁時計時間=実時間のこと、w_t_tを宣言
    
    // 処理ループ周期 T = 1/10 = 0.1[s] <= 10[Hz]
    ros::Rate rate(10);
    


    //ファイル読み込み用
     FILE *fi;
    /* ファイルのオープン
      データ保存場所を設定しておく．
      ここでは /home/home/ubuntu/catkin_ws/の場所に
      testd.txt　の名前で保存
      w+ : 読み書き可能：上書きされる.
      参考HP：https://programming.pc-note.net/c/file2.html
     */
      fi = fopen("/home/ubuntu/catkin_ws/testd.txt", "w+");
        if(fi == NULL){
          printf("file open error:input.txt\n");
        return -1;
        }



    // 初期速度/走行時間
       linear_vel = 0; // 速度[m/s]
       angular_vel = 0; //回転速度[rad/s]
       //ROS_INFO("moveSecond:begin V%d,Q%d", linear_vel, angular_vel);

     /* 計測スタート */
        ros::WallTime wall_begin = ros::WallTime::now();
         //クラス::壁時計で宣言、変数wall_biginに関数now()の値を代入
  
      /* 無限ループ:while{ros::ok())を利用して動作制御入力を発生
         無限ループで送信する場合：ros::spinOnce();を利用して1回ごとに呼び出す */
          vel.linear.x  = linear_vel; //並進速度命令
          vel.angular.z = angular_vel; //回転速度命令
          //時間設定
          
          ros::WallDuration first_t = ros::WallDuration(2.0);
          ros::WallDuration second_t = ros::WallDuration(5.0);
          ros::WallDuration third_t = ros::WallDuration(7.0);
          ros::WallDuration fourth_t = ros::WallDuration(17.0);
          ros::WallDuration fifth_t = ros::WallDuration(19.0);
          ros::WallDuration sixth_t = ros::WallDuration(22.0);
          ros::WallDuration stop_t = ros::WallDuration(24.0);
          ros::WallDuration f_t = ros::WallDuration(34.0);

          cout << "start_time %u \n" << f_t << endl;


/* ここから制御ループ */
      while(ros::ok())
      {
          ros::WallTime wall_now = ros::WallTime::now();//時間計測と更新
          w_t_t = wall_now - wall_begin; //経過時間を現在時刻 - 開始時刻と設定
    /*  --------------------------------------------------
        以下に動作計画用速度式を
        プログラムする.
        ifを使用/swichを使用
        参考HP：https://demura.net/education/16287.html
        以下のprogramは一例です．完成版ではありません．
      ---------------------------------------------------- */
    
        
     if (w_t_t < first_t) //0秒から2秒(加速)
      {
        cout << "Time %u" << w_t_t << endl;
        int t_t = w_t_t.sec;//t_tに経過時間の実数値を代入
          vel.linear.x = 0.1*t_t; //加速の式
          vel.angular.z = 0.0;
        cout << "v1\n" << vel.linear.x << endl;
      }
      else if (first_t <= w_t_t && w_t_t < second_t)//2秒から5秒(定速)
      {
            cout << "Time %u" << w_t_t << endl;
            vel.linear.x = 0.2;
            vel.angular.z = 0.0;
            cout << "v2\n" << vel.linear.x << endl;
      }
      else if (second_t <= w_t_t && w_t_t < third_t)//5秒から7秒(減速)
      {
            cout << "Time %u" << w_t_t << endl;
            int t_t = w_t_t.sec;
            vel.linear.x = -0.1*t_t+0.7;
            vel.angular.z = 0.0;
            cout << "v2\n" << vel.linear.x << endl;
      }
      else if (third_t <= w_t_t && w_t_t < fourth_t)//回転(7~17秒)
      {
            int t_t_2 = w_t_t.sec -7;
            cout << "Time %u" << w_t_t << endl;
            vel.linear.x = 0.0;
            vel.angular.z = 0.1*M_PI;
            cout << "v3\n" << vel.linear.x << endl;
      }
       else if (fourth_t <= w_t_t && w_t_t < fifth_t) //17秒から19秒(加速)
      {
        cout << "Time %u" << w_t_t << endl;
        int t_t = w_t_t.sec - 17;//t_tに経過時間の実数値を代入
          vel.linear.x = 0.1*t_t; //加速の式
          vel.angular.z = 0.0;
        cout << "v1\n" << vel.linear.x << endl;
      }
      else if (fifth_t <= w_t_t && w_t_t < sixth_t)//19秒から22秒(定速)
      {
            cout << "Time %u" << w_t_t << endl;
            vel.linear.x = 0.2;
            vel.angular.z = 0.0;
            cout << "v2\n" << vel.linear.x << endl;
      }
      else if (sixth_t <= w_t_t && w_t_t < stop_t)//22秒から24秒(減速)
      {
            cout << "Time %u" << w_t_t << endl;
            int t_t = w_t_t.sec - 17;
            vel.linear.x = -0.1*t_t+0.7;
            vel.angular.z = 0.0;
            cout << "v2\n" << vel.linear.x << endl;
      }
      else if (stop_t <= w_t_t && w_t_t < f_t) //24秒から34秒まで回転
      {
        cout << "Time %u" << w_t_t << endl;
            vel.linear.x = 0.0;
            vel.angular.z = 0.1*M_PI;
            cout << "v2\n" << vel.linear.x << endl;
      }
      else if (f_t <= w_t_t)
      {
        cout << "Time %u" << w_t_t << endl;
            vel.linear.x = 0.0;
            vel.angular.z = 0.0;
            cout << "v2\n" << vel.linear.x << endl;
      }


      pub.publish(vel);
       /* 制御入力データ保存 */
      fprintf(fi,"Time data = %u.%u ", w_t_t.sec, w_t_t.nsec);
      fprintf(fi,"V = %f, Q = %f \n", vel.linear.x, vel.angular.z);
      ros::spinOnce();
       rate.sleep();        // 指定した周期でループするよう寝て待つ
      }
    fclose(fi);
    return 0;
}
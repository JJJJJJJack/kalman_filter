#include "ros/ros.h"
#include "std_msgs/String.h"
#include <iostream>
#include <fstream>
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/PoseStamped.h"

#include <time.h>
#include <Eigen/Eigen>
#include "kalman_lib/kalman.hpp"


using namespace std;
using namespace Eigen;

double freq = 50;
double dt = 1/freq;
bool Sensor_update = false;

void slam_callback(const geometry_msgs::PoseStamped &new_message)
{
  Sensor_update = true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "kalman_filter_pose3D");
  ros::NodeHandle nh;
	
  ros::Rate loop_rate(freq);
  
  //ros::Subscriber subscribe_roomba_command = n.subscribe("iRobot_0/odom",1,roomba_callback);
  //ros::Subscriber subscribe_vo_command = n.subscribe("/odom_mono",1,vo_callback);
  ros::Subscriber subscribe_slam_command = nh.subscribe("/slam_out_pose",1,slam_callback);
  

  struct timeval tvstart, tvend;
  gettimeofday(&tvstart,NULL);

  int count = 0;
  srand((int)time(0));
  bool Start_timer = true;

  int n = 6; // Number of states
  int m = 3; // Number of measurements
  int r = 3; // Number of inputs
  
  Eigen::MatrixXd A(n, n); // System dynamics matrix
  Eigen::MatrixXd B(n, r); // System inputs matrix
  Eigen::MatrixXd C(m, n); // Output matrix
  Eigen::MatrixXd R(n, n); // Process noise covariance
  Eigen::MatrixXd Q(m, m); // Measurement noise covariance
  Eigen::MatrixXd P(n, n); // Estimate error covariance

  A.topLeftCorner(n/2,n/2) = MatrixXd::Identity(n/2,n/2);
  A.topRightCorner(n/2,n/2) = dt*MatrixXd::Identity(n/2,n/2);
  A.bottomLeftCorner(n/2,n/2) = MatrixXd::Zero(n/2,n/2);
  A.bottomRightCorner(n/2,n/2) = MatrixXd::Identity(n/2,n/2);

  B << 0,0,0,
    0,0,0,
    0,0,0,
    dt,0,0,
    0,dt,0,
    0,0,dt;
  
  C.topLeftCorner(m,m) = MatrixXd::Identity(m,m);
  C.topRightCorner(n-m,n-m) = MatrixXd::Zero(n-m,n-m);

  R = 0.001*MatrixXd::Identity(n,n);
  Q = 0.05*MatrixXd::Identity(m,m);

  P = MatrixXd::Constant(n,n, 0.05);

  KalmanFilter kf(dt, A, B, C, R, Q, P);

  Eigen::VectorXd x0(n);
  x0 << 0,0,0,0,0,0;
  kf.init(0, x0);

  while (ros::ok())
  {
    if(Sensor_update){
      Sensor_update = false;
      Eigen::VectorXd y(3), u(3);
      y << 1,1,1;
      u << 0.01, 0.01, 0.01;
      
      kf.update(y, u);
    }
    
    
    gettimeofday(&tvend,NULL);
    double totaltime = tvend.tv_sec - tvstart.tv_sec + 1e-6 * (tvend.tv_usec - tvstart.tv_usec);

    ros::spinOnce();

    loop_rate.sleep();
    ++count;
  }
  return 0;
}

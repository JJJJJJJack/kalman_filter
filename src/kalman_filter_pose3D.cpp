#include "ros/ros.h"
#include "std_msgs/String.h"
#include <iostream>
#include <fstream>
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/PoseStamped.h"
#include "sensor_msgs/Imu.h"
#include <tf/transform_datatypes.h>

#include <time.h>
#include <math.h>
#include <Eigen/Eigen>
#include "kalman_lib/kalman.hpp"


using namespace std;
using namespace Eigen;

double freq = 50;
double dt = 1/freq;
bool imu_update = false, pose_update = false;
sensor_msgs::Imu imu_msg;
geometry_msgs::TransformStamped pose_msg, kf_pose;
double roll, pitch, yaw;

void imu_callback(const sensor_msgs::Imu &new_message)
{
  imu_update = true;
  imu_msg = new_message;
}

void pose_callback(const geometry_msgs::TransformStamped &new_message)
{
  pose_update = true;
  pose_msg = new_message;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "kalman_filter_pose3D");
  ros::NodeHandle nh;
	
  ros::Rate loop_rate(freq);
  
  ros::Subscriber subscribe_imu  = nh.subscribe("/crazyflie/imu",1,imu_callback);
  ros::Subscriber subscribe_pose = nh.subscribe("/crazyflie/pose",1,pose_callback);
  ros::Publisher  publish_pose   = nh.advertise<geometry_msgs::TransformStamped>("crazyflie/kf_pose", 5);

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
    pose_update = true;
    if(imu_update && pose_update){
      imu_update = false;
      // Get roll pitch yaw from imu
      tf::Quaternion q_tf(imu_msg.orientation.x, imu_msg.orientation.y, imu_msg.orientation.z, imu_msg.orientation.w);
      tf::Matrix3x3 mat(q_tf);
      mat.getEulerYPR(yaw, pitch, roll);
      //ROS_INFO("Roll: %f, Pitch: %f, Yaw: %f", roll/M_PI*180.0, pitch/M_PI*180.0, yaw/M_PI*180.0);
      // Convert roll pitch to rotation matrix
      Eigen::AngleAxisd rollAngle(roll, Eigen::Vector3d::UnitX());
      Eigen::AngleAxisd pitchAngle(pitch, Eigen::Vector3d::UnitY());
      Eigen::Quaternion<double> q = rollAngle * pitchAngle;
      Eigen::Matrix3d rotationMatrix = q.matrix();

      pose_update = false;
      
      Eigen::VectorXd y(3), u(3);
      y << pose_msg.transform.translation.x,pose_msg.transform.translation.y,pose_msg.transform.translation.z;
      u << -imu_msg.linear_acceleration.x, imu_msg.linear_acceleration.y, imu_msg.linear_acceleration.z;
      //ROS_INFO("IMU_MAG: %f", sqrt(u(0)*u(0)+u(1)*u(1)+u(2)*u(2)));
      //ROS_INFO("acc_body %f %f %f",u(0),u(1),u(2));
      u = rotationMatrix*u;
      // Compensate for the static gravity acceleration
      u(2) = u(2)-9.7;
      u(0) = -u(0);
      u(1) = -u(1);
      //ROS_INFO("acc_world %f %f %f",u(0),u(1),u(2));
            
      kf.update(y, u);
      VectorXd filtered_pose = kf.state();
      //cout<<kf.state().transpose()<<endl;
      kf_pose.header.stamp = ros::Time::now();
      kf_pose.header.frame_id = "kf_pose";
      kf_pose.child_frame_id = "/world";
      kf_pose.transform.translation.x = filtered_pose(0);
      kf_pose.transform.translation.y = filtered_pose(1);
      kf_pose.transform.translation.z = filtered_pose(2);
      kf_pose.transform.rotation = pose_msg.transform.rotation;
      publish_pose.publish(kf_pose);
    }
    
    
    gettimeofday(&tvend,NULL);
    double totaltime = tvend.tv_sec - tvstart.tv_sec + 1e-6 * (tvend.tv_usec - tvstart.tv_usec);

    ros::spinOnce();

    loop_rate.sleep();
    ++count;
  }
  return 0;
}

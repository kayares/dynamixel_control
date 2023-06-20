#ifndef CALLBACK_H
#define CALLBACK_H

#include <iostream>
#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Int32.h"
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/Bool.h"
#include "dynamixel.hpp"
#include <sensor_msgs/Imu.h>
#include <tf/tf.h>
#include "Walkingpattern_generator.hpp"

using Eigen::VectorXd;


class Callback
{
public:
  Callback();

  //Function
  virtual void JointStatesCallback(const sensor_msgs::JointState::ConstPtr &joint_command);
  virtual void FSRsensorCallback(const std_msgs::Int32ConstPtr &FSR);
  virtual void IMUsensorCallback(const sensor_msgs::Imu::ConstPtr &IMU);
  virtual void SelectMotion(const std_msgs::Float32Ptr &msg);
  void MotionMaker();
  //Variable
  VectorXd Goal_joint_ = VectorXd::Zero(NUMBER_OF_DYNAMIXELS);
  int32_t fsr_value = 0;
  VectorXd quaternion = VectorXd::Zero(4);
  VectorXd RPY = VectorXd::Zero(3); //Roll Pitch Yaw
  VectorXd Accel = VectorXd::Zero(3); // Accel_x, Accel_y, Accel_z 
  VectorXd Gyro = VectorXd::Zero(3); // Gyro_x, Gyro_y, Gyro_z 
  float mode = 0;
  MatrixXd RL_motion;
  MatrixXd LL_motion;
  MatrixXd RL_motion0;
  MatrixXd LL_motion0;
  MatrixXd RL_motion1;
  MatrixXd LL_motion1;
  MatrixXd RL_motion2;
  MatrixXd LL_motion2;
  MatrixXd RL_motion3;
  MatrixXd LL_motion3;
  MatrixXd RL_motion4;
  MatrixXd LL_motion4;
  MatrixXd RL_motion5;
  MatrixXd LL_motion5;
  MatrixXd RL_motion6;
  MatrixXd LL_motion6;
  MatrixXd RL_motion7;
  MatrixXd LL_motion7;

  // tf2::Quaternion quaternion;
  
};

#endif // CALLBACK_H
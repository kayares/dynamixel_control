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
#include "Walkingpattern_generator.hpp"

using Eigen::VectorXd;
using Eigen::MatrixXd;


class Callback
{
public:
  Callback();

  //Function
  virtual void JointStatesCallback(const sensor_msgs::JointState::ConstPtr &joint_command);
  virtual void sensorCallback(const std_msgs::Int32ConstPtr &FSR);
  virtual void SelectMotion(const std_msgs::Float32Ptr &msg);
  void MotionMaker();
  
  //Variable
  VectorXd Goal_joint_ = VectorXd::Zero(NUMBER_OF_DYNAMIXELS);
  int32_t fsr_value = 0;
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
};

#endif // CALLBACK_H
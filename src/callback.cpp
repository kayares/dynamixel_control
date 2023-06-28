#include "callback.hpp"

extern Dxl dxl;
extern Motions motion;

Callback::Callback() {}

sensor_msgs::JointState joint_state;

void Callback::JointStatesCallback(const sensor_msgs::JointState::ConstPtr &joint_command)
{
    for (int i = 0; i < NUMBER_OF_DYNAMIXELS; i++)
    {
        Goal_joint_[i] = joint_command->position[i];
        dxl.SetThetaRef(Goal_joint_);
    }
}

void Callback::FSRsensorCallback(const std_msgs::UInt8::ConstPtr &FSR)
{
    L_value = FSR->data; // Left_foot_FSR
    R_value = FSR->data; // Right_foot_FSR
}

void Callback::IMUsensorCallback(const sensor_msgs::Imu::ConstPtr &IMU)
{
    // ROS_INFO("Accel: %.3f,%.3f,%.3f [m/s^2] - Ang. vel: %.3f,%.3f,%.3f [deg/sec] - Orient. Quat: %.3f,%.3f,%.3f,%.3f",
    //  IMU->linear_acceleration.x, IMU->linear_acceleration.y, IMU->linear_acceleration.z,
    //  IMU->angular_velocity.x, IMU->angular_velocity.y, IMU->angular_velocity.z,
    //  IMU->orientation.x, IMU->orientation.y, IMU->orientation.z, IMU->orientation.w);
    IMU->linear_acceleration.x, IMU->linear_acceleration.y, IMU->linear_acceleration.z,
        IMU->angular_velocity.x, IMU->angular_velocity.y, IMU->angular_velocity.z,
        IMU->orientation.x, IMU->orientation.y, IMU->orientation.z, IMU->orientation.w;
    
    Accel(0) = IMU->linear_acceleration.x;
    Accel(1) = IMU->linear_acceleration.y;
    Accel(2) = IMU->linear_acceleration.z;

    Gyro(0) = IMU->angular_velocity.x;
    Gyro(1) = IMU->angular_velocity.y;
    Gyro(2) = IMU->angular_velocity.z;


    quaternion(0) = IMU->orientation.x;
    quaternion(1) = IMU->orientation.y;
    quaternion(2) = IMU->orientation.z;
    quaternion(3) = IMU->orientation.w;
   
}



void Callback::SelectMotion(const std_msgs::Float32Ptr &msg)
{
    mode = msg ->data;
    // ROS_INFO("mode(%f)",mode);
    if (mode == 0 ){
    RL_motion = RL_motion0;
    LL_motion = LL_motion0;
    }
    else if (mode == 1){
    RL_motion = RL_motion1;
    LL_motion = LL_motion1;
    }
    else if (mode == 2){
    RL_motion = RL_motion2;
    LL_motion = LL_motion2;
    }
    else if (mode == 3){
    RL_motion = RL_motion3;
    LL_motion = LL_motion3;
    }
    else if (mode ==4){
    RL_motion = RL_motion4;
    LL_motion = LL_motion4;
    }
    else if (mode ==5){
    RL_motion = RL_motion5;
    LL_motion = LL_motion5;
    }
    else if (mode == 6){
    RL_motion = RL_motion6;
    LL_motion = LL_motion6;
    }
    else if (mode == 7){
    RL_motion = RL_motion7;
    LL_motion = LL_motion7;
    }
    else {
    RL_motion = RL_motion0;
    LL_motion = LL_motion0;
    }
}


void Callback::MotionMaker(){
    

    motion.Motion0();
    LL_motion0 = motion.Return_Motion0_LL();
    RL_motion0 = motion.Return_Motion0_RL();

    motion.Motion1();
    LL_motion1 = motion.Return_Motion1_LL();
    RL_motion1 = motion.Return_Motion1_RL();

    motion.Motion2();
    LL_motion2 = motion.Return_Motion2_LL();
    RL_motion2 = motion.Return_Motion2_RL();

    motion.Motion3();
    LL_motion3 = motion.Return_Motion3_LL();
    RL_motion3 = motion.Return_Motion3_RL();

    motion.Motion4();
    LL_motion4 = motion.Return_Motion4_LL();
    RL_motion4 = motion.Return_Motion4_RL();

    motion.Motion5();
    LL_motion5 = motion.Return_Motion5_LL();
    RL_motion5 = motion.Return_Motion5_RL();

    motion.Motion6();
    LL_motion6 = motion.Return_Motion6_LL();
    RL_motion6 = motion.Return_Motion6_RL();

    motion.Motion7();
    LL_motion7 = motion.Return_Motion7_LL();
    RL_motion7 = motion.Return_Motion7_RL();

    LL_motion = LL_motion0;
    RL_motion = RL_motion0;


}

#include "callback.hpp"

extern Dxl dxl;


Callback::Callback() {
}

sensor_msgs::JointState joint_state;

void Callback::JointStatesCallback(const sensor_msgs::JointState::ConstPtr &joint_command)
{
    for (int i=0; i<NUMBER_OF_DYNAMIXELS;i++)
    {
        Goal_joint_[i] = joint_command->position[i];
        dxl.SetThetaRef(Goal_joint_);
    } 
}


void Callback::sensorCallback(const std_msgs::Int32ConstPtr &FSR)
{
    fsr_value = FSR->data;
}


void Callback::SelectMotion(const std_msgs::Float32Ptr &msg)
{
    mode = msg ->data;
    ROS_INFO("mode(%f)",mode);
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
    else {
    RL_motion = RL_motion0;
    LL_motion = LL_motion0;
    }
}


void Callback::MotionMaker(){
    
    Motions motion;
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

    LL_motion = LL_motion0;
    RL_motion = RL_motion0;


}
#include <iostream>
#include "dynamixel.hpp"
#include "callback.hpp"
#include "dynamixel_controller.hpp"
#include "Walkingpattern_generator.hpp"

Dxl dxl;
Callback callback;
Dxl_Controller dxl_ctrl;
Motions motion;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "daynmixel_current_2port");
    ros::Time::init();
    ros::Rate loop_rate(300);
    ros::NodeHandle nh;
    // ros::AsyncSpinner spinner(0); // Multi-threaded spinning
    // spinner.start(); // Multi-threaded spinning


    ros::Publisher joint_state_publisher_;   ///< Publishes joint states from reads
    joint_state_publisher_ = nh.advertise<sensor_msgs::JointState>("KWJ_joint_states", 100);
    
    ros::Subscriber joint_state_subscriber_; ///< Gets joint states for writes
    joint_state_subscriber_ = nh.subscribe("KWJ_desired_joint_states", 1000, &Callback::JointStatesCallback, &callback);

    ros::Subscriber FSR_sensor_subscriber_; ///< Gets FSR Sensor data from Arduino FSR 
    FSR_sensor_subscriber_ = nh.subscribe("FSR", 1000, &Callback::sensorCallback, &callback);

    ros::Subscriber Motion_Selector_; ///< Gets FSR Sensor data from Arduino FSR 
    Motion_Selector_= nh.subscribe("Select_Motion", 1000, &Callback::SelectMotion, &callback);

    // ros::waitForShutdown(); // Multi-threaded spinning

    VectorXd A(12);

    dxl.initActuatorValues();
    callback.MotionMaker();
    FILE* file1 = fopen("/home/jaemin/matlab_codes/fordebugdgree/motion1.dat","w");
    FILE* file2 = fopen("/home/jaemin/matlab_codes/fordebugdgree/motion3.dat","w");
    motion.Motion7();
    MatrixXd motion1 = motion.Return_Motion7_LL();
    motion.Motion3();
    MatrixXd motion3 = motion.Return_Motion3_LL();
    for (int i = 1; i < motion1.rows(); i++)
    {
        fprintf(file1,"%d ", i );
        for (int j = 0; j < 5; j++)
        {
            fprintf(file1, "%lf ", motion1(i, j));
        }
        fprintf(file1, "%lf\n", motion1(i, 5));
    }
    for (int i = 1; i < motion3.rows(); i++)
    {
        fprintf(file2, "%d ", i);

        for (int j = 0; j < 5; j++)
        {
            fprintf(file2, "%lf ", motion3(i, j));
        }
        fprintf(file2, "%lf\n", motion3(i, 5));
    }
    int indext = 0;

    while (ros::ok())
    {
        //About motion
        indext = 506;
         indext = 709;
         indext += 1;
        A[0] = callback.RL_motion(indext, 0);
        A[1] = callback.RL_motion(indext, 1) - 2 * DEG2RAD;
        A[2] = callback.RL_motion(indext, 2) - 24.22 * DEG2RAD;
        A[3] = -callback.RL_motion(indext, 3) + 24.58 * DEG2RAD;
        A[4] = -callback.RL_motion(indext, 4) + 24.22 * DEG2RAD;
        A[5] = -callback.RL_motion(indext, 5);
        A[6] = callback.LL_motion(indext, 0);
        A[7] = callback.LL_motion(indext, 1);
        A[8] = -callback.LL_motion(indext, 2) + 24.22 * DEG2RAD;
        A[9] = callback.LL_motion(indext, 3) - 24.58 * DEG2RAD;
        A[10] = callback.LL_motion(indext, 4) - 24.22 * DEG2RAD;
        A[11] = -callback.LL_motion(indext, 5);
        if (indext >= callback.RL_motion.rows()-1)
            {indext = 0;}

        dxl.SetThetaRef(A);
        dxl.syncWriteTheta();

        sensor_msgs::JointState msg;
        msg.header.stamp = ros::Time::now();

        std::vector<std::string> joint_name = {"j1", "j2", "j3", "j4", "j5", "j6", "j7", "j8", "j9", "j10", "j11", "j12"};
    

        for (uint8_t i = 0; i < NUMBER_OF_DYNAMIXELS; i++)
        {
            msg.name.push_back(joint_name.at(i));
            // dxl.syncReadTheta();
            // msg.position.push_back(dxl.th_[i]);
        }
        joint_state_publisher_.publish(msg);
        dxl.FSR_flag();  
        dxl.syncWriteTheta();
        // std::cout << callback.fsr_value << std::endl;

        ros::spinOnce();
        loop_rate.sleep();
    }

    // ROS_INFO("daynmixel_current_2port!");
    // dxl.~Dxl();
    return 0;
}





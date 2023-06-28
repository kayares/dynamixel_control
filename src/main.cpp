#include <iostream>
#include "dynamixel.hpp"
#include "callback.hpp"
#include "dynamixel_controller.hpp"
#include "Walkingpattern_generator.hpp"
#include "sensor.hpp"

Dxl dxl;
Callback callback;
Dxl_Controller dxl_ctrl;
Motions motion;

FILE *imu_accel;
FILE *imu_gyro;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "daynmixel_current_2port");
    ros::Time::init();
    ros::Rate loop_rate(500);
    ros::NodeHandle nh;
    Sensor sensor;

    ros::AsyncSpinner spinner(0); // Multi-threaded spinning
    spinner.start();              // Multi-threaded spinning

    // IMU
    //  imu_accel = fopen("/home/woojin/imu_Accel_0613_(1).dat", "w");
    //  imu_gyro = fopen("/home/woojin/imu_gyro1_0613_(1).dat", "w");

    ros::Publisher joint_state_publisher_; ///< Publishes joint states from reads
    joint_state_publisher_ = nh.advertise<sensor_msgs::JointState>("KWJ_joint_states", 100);

    ros::Subscriber joint_state_subscriber_; ///< Gets joint states for writes
    joint_state_subscriber_ = nh.subscribe("KWJ_desired_joint_states", 1000, &Callback::JointStatesCallback, &callback);

    ros::Subscriber FSR_L_sensor_subscriber_; ///< Gets FSR Sensor data from Arduino FSR_L
    FSR_L_sensor_subscriber_ = nh.subscribe("FSR_L", 1000, &Callback::FSRsensorCallback, &callback);

    ros::Subscriber FSR_R_sensor_subscriber_; ///< Gets FSR Sensor data from Arduino FSR_R
    FSR_R_sensor_subscriber_ = nh.subscribe("FSR_R", 1000, &Callback::FSRsensorCallback, &callback);

    ros::Subscriber IMU_sensor_subscriber_; ///< Gets IMU Sensor data from XSENSE mti_driver_node
    IMU_sensor_subscriber_ = nh.subscribe("/imu/data", 1000, &Callback::IMUsensorCallback, &callback);

    ros::Subscriber Motion_Selector_; ///< Gets FSR Sensor data from Arduino FSR
    Motion_Selector_ = nh.subscribe("Select_Motion", 1000, &Callback::SelectMotion, &callback);
    // ros::waitForShutdown(); // Multi-threaded spinning

    VectorXd A(NUMBER_OF_DYNAMIXELS);

    double walkfreq = 1.48114;
    double walktime = 2 / walkfreq;
    int freq = 500;
    int simt = walktime * freq;
    double sim_time = 5 * walktime;
    int sim_n = sim_time * freq;
    int indext = 0;
    

    callback.MotionMaker();

    while (ros::ok())
    {   
        // About motion
        // indext = 506;
        //  indext = 709;
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
        if (indext > simt * 1.74 && indext < simt * 1.75 && callback.R_value < 3)
        {
            indext = indext;
        }
        else if (indext > simt * 2.74 && indext < simt * 2.75 && callback.R_value < 3)
        {
            indext = indext;
        }
        else if (indext > simt * 3.74 && indext < simt * 3.75 && callback.R_value < 3)
        {
            indext = indext;
        }
        else if (indext > simt * 1.24 && indext < simt * 1.25 && callback.L_value < 3)
        {
            indext = indext;
        }
        else if (indext > simt * 2.24 && indext < simt * 2.25 && callback.L_value < 3)
        {
            indext = indext;
        }
        else if (indext > simt * 3.24 && indext < simt * 3.25 && callback.L_value < 3)
        {
            indext = indext;
        }
        else
        {
            indext += 1;
        }
        if (indext >= callback.RL_motion.rows() - 1)
        {
            if (callback.L_value > 1 && callback.R_value > 1)
                indext = 0;
            else
                indext = indext - 1;
        }
        // indext += 1;
        // if (indext >= callback.RL_motion.rows() - 1)
        // { indext = 0;}
        dxl.SetThetaRef(A);
        dxl.syncWriteTheta();

        sensor.Publish_Accel_Origin();
        sensor.Publish_Gyro_Origin();
        sensor.Publish_Accel_HPF();
        sensor.Publish_Gyro_LPF();
        sensor.Publish_Velocity_HPF_Integral();
        sensor.Publish_Velocity_Integral();
        sensor.Publish_Velocity_Complementary();

        ros::spinOnce();
        loop_rate.sleep();
}

// ROS_INFO("daynmixel_current_2port!");
dxl.~Dxl();
return 0;
}

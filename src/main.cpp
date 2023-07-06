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

    // ros::Publisher joint_state_publisher_; ///< Publishes joint states from reads
    // joint_state_publisher_ = nh.advertise<sensor_msgs::JointState>("KWJ_joint_states", 100);

    // ros::Subscriber joint_state_subscriber_; ///< Gets joint states for writes
    // joint_state_subscriber_ = nh.subscribe("KWJ_desired_joint_states", 1000, &Callback::JointStatesCallback, &callback);

    // ros::Subscriber FSR_L_sensor_subscriber_; ///< Gets FSR Sensor data from Arduino FSR_L
    // FSR_L_sensor_subscriber_ = nh.subscribe("FSR_L", 1000, &Callback::FSRsensorCallback, &callback);

    // ros::Subscriber FSR_R_sensor_subscriber_; ///< Gets FSR Sensor data from Arduino FSR_R
    // FSR_R_sensor_subscriber_ = nh.subscribe("FSR_R", 1000, &Callback::FSRsensorCallback, &callback);

    // ros::Subscriber IMU_sensor_subscriber_; ///< Gets IMU Sensor data from XSENSE mti_driver_node
    // IMU_sensor_subscriber_ = nh.subscribe("/imu/data", 1000, &Callback::IMUsensorCallback, &callback);

    ros::Subscriber Motion_Selector_; ///< Gets FSR Sensor data from Arduino FSR
    Motion_Selector_ = nh.subscribe("Select_Motion", 1000, &Callback::SelectMotion, &callback);
    // ros::waitForShutdown(); // Multi-threaded spinning
    FILE *file1 = fopen("/home/jaemin/matlab_codes/dynamixel_theta_data1.dat", "w");
    FILE *file2 = fopen("/home/jaemin/matlab_codes/dynamixel_theta_data2.dat", "w");
    callback.Write_Arm_Theta();
    callback.MotionMaker();
    for (int i = 0; i<callback.RL_motion3.rows();i++){
        fprintf(file1,"%d ",i);
        for (int j = 0; j<6;j++){
            fprintf(file1,"%lf ",callback.RL_motion3(i,j));
        }
        fprintf(file1, "\n");
    }
    for (int i = 0; i<callback.LL_motion7.rows();i++){
        fprintf(file2,"%d ",i);
        for (int j = 0; j<6;j++){
            fprintf(file2,"%lf ",callback.RL_motion7(i,j));
        }
        fprintf(file2, "\n");
    }
    fclose(file1);
    fclose(file2);
    while (ros::ok())
    {   
       callback.Write_Leg_Theta();

        dxl.SetThetaRef(callback.All_Theta);
        dxl.syncWriteTheta();

        // sensor.Publish_Accel_Origin();
        // sensor.Publish_Gyro_Origin();
        // sensor.Publish_Accel_HPF();
        // sensor.Publish_Gyro_LPF();
        // sensor.Publish_Velocity_HPF_Integral();
        // sensor.Publish_Velocity_Integral();
        // sensor.Publish_Velocity_Complementary();

        ros::spinOnce();
        loop_rate.sleep();
}

// ROS_INFO("daynmixel_current_2port!");
dxl.~Dxl();
return 0;
}
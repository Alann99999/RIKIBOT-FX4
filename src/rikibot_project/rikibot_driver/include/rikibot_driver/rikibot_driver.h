#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <tf/tf.h>
#include <geometry_msgs/Vector3.h>
#include <std_msgs/UInt8MultiArray.h>
#include <sensor_msgs/Imu.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Int16.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/Float32.h>
#include <sensor_msgs/Range.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TransformStamped.h>
#include <rikibot_driver/Servo.h>

#include <string>
#include <vector>
#include <math.h>
#include <serial/serial.h>
#include <iostream>
using namespace std;

#define PI 3.1415926

#define BATTERY_RATE 1

//机器人数据处理周期,单位S
#define DATA_PERIOD   0.02f

#define head1 0xAB
#define head2 0xCD
#define sendType_velocity    0x11
#define servoType_angle      0x12

//IMU加速度计量程±2g，对应数据范围±32768
//加速度计原始数据转换位m/s^2单位，32768/2g=32768/19.6=1671.84
#define ACC_RATIO 	  (2*9.8/32768)

//IMU陀螺仪量程±500°，对应数据范围±32768
//陀螺仪原始数据转换位弧度(rad)单位
#define GYRO_RATIO   ((500*PI/180)/32768)


//IMU数据结构体
typedef struct{
    float acc_x;
    float acc_y;
    float acc_z;

    float gyro_x;
    float gyro_y;
    float gyro_z;
}Imu_Data;

//IMU四元数结构体
typedef struct{
    float w;
    float x;
    float y;
    float z;
}Imu_Orientation;

//机器人速度数据结构体
typedef struct{
    float linear_x;
    float linear_y;
    float angular_z;
}Velocity;

//机器人位置数据结构体
typedef struct
{
    float pos_x;
    float pos_y;
    float angular_z;
}Odom_Pose;

class RikibotDriver
{
    public:
        RikibotDriver();
        ~RikibotDriver();
        void loop();

    private:
        bool ReadFormSerial();
        void Check_sum(uint8_t* data, size_t len, uint8_t& dest);


        void cmd_vel_callback(const geometry_msgs::Twist &twist_aux);
        void servo_callback(const rikibot_driver::Servo &servo_msg);
        void SetVelocity(double x, double y, double yaw);
        void SetServoAngle(uint8_t id, uint16_t angle, uint16_t time);

        void PublisherOdom();
        void publisherImu();
        void publisherBattery();
        void publisherSonar();
        void publisherInfrared();


        serial::Serial Robot_Serial;

        //Frame定义
        std::string odom_frame_;    //里程计
        std::string base_frame_;  //机器人
        std::string imu_frame_;     //IMU

        //话题定义
        std::string odom_topic_;    //里程计
        std::string imu_topic_;     //IMU话题
        std::string bat_topic_;     //电池话题
        std::string cmd_vel_topic_;     //电池话题	
        std::string sonar_topic_;     //电池话题	
        std::string infrared_topic_;     //电池话题	

        //发布器定义
        ros::Publisher bat_pub_;
        ros::Publisher odom_pub_;
        ros::Publisher imu_pub_;
        ros::Publisher sonar_pub_;
        ros::Publisher infrared_pub_;


        ros::Subscriber cmd_sub_;
        ros::Subscriber servo_sub_;

        nav_msgs::Odometry odom_msgs_;  //里程计发布消息
        sensor_msgs::Imu imu_msgs_;     //IMU发布消息
        std_msgs::Float32 bat_msgs_;    //电池电压发布消息
        std_msgs::Float32 sonar_msgs_;    //电池电压发布消息
        std_msgs::UInt8MultiArray infrared_msgs_;    //电池电压发布消息

        std::string port_name_;
        std::string rxdata;

        uint8_t *rxbuf;

        int baud_rate_;
        int control_rate_;

        //数据定义
        Imu_Data imu_data_;            //IMU数据
        Velocity vel_data_;          //机器人的速度
        Odom_Pose pos_data_;           //机器人的位置

        ros::Time now_;


};



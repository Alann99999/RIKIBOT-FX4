#include "../include/rikibot_driver/rikibot_driver.h"

RikibotDriver::RikibotDriver()
{
    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_("~");

    now_ = ros::Time::now();

     //frame初始化
    private_nh_.param<std::string>("odom_frame", odom_frame_, "odom");
    private_nh_.param<std::string>("base_frame", base_frame_, "base_footprint");
    private_nh_.param<std::string>("imu_frame", imu_frame_, "imu_link");

    //话题消息初始化
    private_nh_.param<std::string>("odom_topic", odom_topic_, "raw_odom");
    private_nh_.param<std::string>("imu_topic", imu_topic_, "imu/data_raw");
    private_nh_.param<std::string>("battery_topic", bat_topic_, "battery");
    private_nh_.param<std::string>("sonar_topic", sonar_topic_, "sonar");
    private_nh_.param<std::string>("cmd_vel_topic", cmd_vel_topic_, "cmd_vel");
    private_nh_.param<std::string>("infrared_topic", infrared_topic_, "infrared");


    /*从配置文件中获取机器人参数*/
    private_nh_.param<std::string>("port_name",port_name_,std::string("/dev/ttyUSB0"));
    private_nh_.param<int>("baud_rate",baud_rate_, 115200);
    
    //实例化发布者对象
    sonar_pub_ = nh_.advertise<std_msgs::Float32>(sonar_topic_, 10);
    odom_pub_ = nh_.advertise<nav_msgs::Odometry>(odom_topic_, 50);
    bat_pub_  = nh_.advertise<std_msgs::Float32>(bat_topic_, 10);
    imu_pub_  = nh_.advertise<sensor_msgs::Imu>(imu_topic_, 50);
    infrared_pub_  = nh_.advertise<std_msgs::UInt8MultiArray>(infrared_topic_, 10);

    cmd_sub_     = nh_.subscribe(cmd_vel_topic_, 10, &RikibotDriver::cmd_vel_callback, this);
    servo_sub_     = nh_.subscribe("servo", 10, &RikibotDriver::servo_callback, this);

	infrared_msgs_.data.resize(4);


    /**open serial device**/
    try{
         Robot_Serial.setPort(port_name_);
         Robot_Serial.setBaudrate(baud_rate_);
         serial::Timeout to = serial::Timeout::simpleTimeout(2000);
         Robot_Serial.setTimeout(to);
         Robot_Serial.setBytesize((serial::bytesize_t)8);
         Robot_Serial.setParity((serial::parity_t)0);
         Robot_Serial.setStopbits((serial::stopbits_t)1);
         Robot_Serial.open();
    }catch (serial::IOException& e){
		 ROS_ERROR_STREAM("Rikibot Serial Unable to open port ");
    }

	if(Robot_Serial.isOpen()){
	 	ROS_INFO_STREAM("Rikibot Serial Port opened");
	}else{
	}        

    //ROS_INFO("Rikibot Serial Running!");
}

RikibotDriver::~RikibotDriver()
{
    Robot_Serial.close();
}

/*主循环函数*/
void RikibotDriver::loop()
{
    while(ros::ok()){
        if (true == ReadFormSerial()){
            PublisherOdom();
            publisherImu();
            publisherBattery();
            publisherSonar();
            publisherInfrared();
        }
    	ros::spinOnce();
    }
}

/*cmd_vel Subscriber的回调函数*/
void RikibotDriver::cmd_vel_callback(const geometry_msgs::Twist &twist_aux)
{
    SetVelocity(twist_aux.linear.x, twist_aux.linear.y, twist_aux.angular.z);
}


/*servo Subscriber的回调函数*/
void RikibotDriver::servo_callback(const rikibot_driver::Servo &servo_msg)
{
    SetServoAngle(servo_msg.id, servo_msg.angle, servo_msg.time);
}


void RikibotDriver::Check_sum(uint8_t* data, size_t len, uint8_t& dest)
{
    dest = 0x00;
    for(std::size_t i=0; i < len; i++)
    {
        dest += *(data + i);
    }
}


bool RikibotDriver::ReadFormSerial()
{
    uint8_t check_sum;
    if(Robot_Serial.available()){
        rxdata = Robot_Serial.read(Robot_Serial.available());
        rxbuf = (uint8_t *)rxdata.c_str();
        if((rxbuf[0]==head1)&&(rxbuf[1]==head2)){
            //for(size_t i=0; i < rxdata.size(); i++){
            //   printf("%x ", rxbuf[i]);
            //}
            //printf("\r\n");
            Check_sum(rxbuf, rxdata.size() - 1, check_sum);
            if(check_sum == rxbuf[rxdata.size()-1]){
                return true;
            }else
                return false;
        }else{
            return false;
        }
    }else{
        return false;
    }
}

/*Servo Angle发送函数*/
void RikibotDriver::SetServoAngle(uint8_t id, uint16_t angle, uint16_t time)
{
    static uint8_t angle_data[10];
    angle_data[0] = head1;
    angle_data[1] = head2;
    angle_data[2] = 0x0a;
    angle_data[3] = servoType_angle;
    angle_data[4] = (id)&0xff;
    angle_data[5] = (angle>>8) & 0xff;
    angle_data[6] = (angle) & 0xff;
    angle_data[7] = (time>>8) & 0xff;
    angle_data[8] = (time) & 0xff;
    Check_sum(angle_data, 9, angle_data[9]);

    Robot_Serial.write(angle_data, sizeof(angle_data));
}

/*底盘速度发送函数*/
void RikibotDriver::SetVelocity(double x, double y, double yaw)
{
    static uint8_t vel_data[11];
    vel_data[0] = head1;
    vel_data[1] = head2;
    vel_data[2] = 0x0b;
    vel_data[3] = sendType_velocity;
    vel_data[4] = ((int16_t)(x*1000)>>8) & 0xff;
    vel_data[5] = ((int16_t)(x*1000)) & 0xff;
    vel_data[6] = ((int16_t)(y*1000)>>8) & 0xff;
    vel_data[7] = ((int16_t)(y*1000)) & 0xff;
    vel_data[8] = ((int16_t)(yaw*1000)>>8) & 0xff;
    vel_data[9] = ((int16_t)(yaw*1000)) & 0xff;
    Check_sum(vel_data,10,vel_data[10]);

    Robot_Serial.write(vel_data, sizeof(vel_data));
}

/*收到串口数据包解析函数*/
void RikibotDriver::PublisherOdom()
{
    tf2::Quaternion odom_quat;

    vel_data_.linear_x =  ((double)((int16_t)(rxbuf[16]*256+rxbuf[17]))/1000);
    vel_data_.linear_y  = ((double)((int16_t)(rxbuf[18]*256+rxbuf[19]))/1000);
    vel_data_.angular_z =  ((double)((int16_t)(rxbuf[20]*256+rxbuf[21]))/1000);

    //计算里程计数据
    pos_data_.pos_x += (vel_data_.linear_x*cos(pos_data_.angular_z) - vel_data_.linear_y*sin(pos_data_.angular_z)) * DATA_PERIOD;
    pos_data_.pos_y += (vel_data_.linear_x*sin(pos_data_.angular_z) + vel_data_.linear_y*cos(pos_data_.angular_z)) * DATA_PERIOD;
    pos_data_.angular_z += vel_data_.angular_z * DATA_PERIOD;   //绕Z轴的角位移，单位：rad

    //计算里程计四元数
    odom_quat.setRPY(0,0,pos_data_.angular_z);

    //获取数据
    odom_msgs_.header.stamp    = ros::Time::now();
    odom_msgs_.header.frame_id = odom_frame_;
    odom_msgs_.child_frame_id  = base_frame_;
    odom_msgs_.pose.pose.position.x = pos_data_.pos_x;
    odom_msgs_.pose.pose.position.y = pos_data_.pos_y;
    odom_msgs_.pose.pose.position.z = 0;  //高度为0

    odom_msgs_.pose.pose.orientation.x = odom_quat.getX();
    odom_msgs_.pose.pose.orientation.y = odom_quat.getY();
    odom_msgs_.pose.pose.orientation.z = odom_quat.getZ();
    odom_msgs_.pose.pose.orientation.w = odom_quat.getW();

    odom_msgs_.twist.twist.linear.x = vel_data_.linear_x;
    odom_msgs_.twist.twist.linear.y = vel_data_.linear_y;
    odom_msgs_.twist.twist.angular.z = vel_data_.angular_z;

    //里程计协防差矩阵，用于robt_pose_ekf功能包，静止和运动使用不同的参数
    if(vel_data_.linear_x==0 && vel_data_.linear_y==0 && vel_data_.angular_z==0){
        //机器人静止时，IMU水平陀螺仪会存在零飘，编码器没有误差，编码器数据权重增加
        odom_msgs_.pose.covariance = {   1e-9,    0,    0,    0,    0,    0,
                                            0, 1e-3, 1e-9,    0,    0,    0,
                                            0,    0,  1e6,    0,    0,    0,
                                            0,    0,    0,  1e6,    0,    0,
                                            0,    0,    0,    0,  1e6,    0,
                                            0,    0,    0,    0,    0, 1e-9 };

        odom_msgs_.twist.covariance = {  1e-9,    0,    0,    0,    0,    0,
                                            0, 1e-3, 1e-9,    0,    0,    0,
                                            0,    0,  1e6,    0,    0,    0,
                                            0,    0,    0,  1e6,    0,    0,
                                            0,    0,    0,    0,  1e6,    0,
                                            0,    0,    0,    0,    0, 1e-9 };
    }else{
        //机器人运动时，轮子滑动编码器误差增加，IMU陀螺仪数据更加准确，IMU数据权重增加
        odom_msgs_.pose.covariance = {   1e-3,    0,    0,    0,    0,    0,
                                            0, 1e-3,    0,    0,    0,    0,
                                            0,    0,  1e6,    0,    0,    0,
                                            0,    0,    0,  1e6,    0,    0,
                                            0,    0,    0,    0,  1e6,    0,
                                            0,    0,    0,    0,    0,  1e3 };

        odom_msgs_.twist.covariance = {  1e-3,    0,    0,    0,    0,    0,
                                            0, 1e-3,    0,    0,    0,    0,
                                            0,    0,  1e6,    0,    0,    0,
                                            0,    0,    0,  1e6,    0,    0,
                                            0,    0,    0,    0,  1e6,    0,
                                            0,    0,    0,    0,    0,  1e3 };
    }

    //发布
    odom_pub_.publish(odom_msgs_);
}

void RikibotDriver::publisherImu()
{
	//gyro
	imu_data_.gyro_x = ((double)((int16_t)(rxbuf[4]*256+rxbuf[5]))*GYRO_RATIO);
	imu_data_.gyro_y = ((double)((int16_t)(rxbuf[6]*256+rxbuf[7]))*GYRO_RATIO);
	imu_data_.gyro_z = ((double)((int16_t)(rxbuf[8]*256+rxbuf[9]))*GYRO_RATIO);
	//Acc
	imu_data_.acc_x = ((double)((int16_t)(rxbuf[10]*256+rxbuf[11]))*ACC_RATIO);
	imu_data_.acc_y = ((double)((int16_t)(rxbuf[12]*256+rxbuf[13]))*ACC_RATIO);
	imu_data_.acc_z = ((double)((int16_t)(rxbuf[14]*256+rxbuf[15]))*ACC_RATIO);


	imu_msgs_.header.stamp = ros::Time::now();
	imu_msgs_.header.frame_id = "imu_link";

    imu_msgs_.angular_velocity.x = imu_data_.gyro_x;
	imu_msgs_.angular_velocity.y = imu_data_.gyro_y;
	imu_msgs_.angular_velocity.z = imu_data_.gyro_z;

	if (imu_msgs_.angular_velocity.x > -0.01 && imu_msgs_.angular_velocity.x < 0.01 ){
        imu_msgs_.angular_velocity.x = 0;
	}

    if (imu_msgs_.angular_velocity.y > -0.01 && imu_msgs_.angular_velocity.y < 0.01 ){
        imu_msgs_.angular_velocity.y = 0;
	}

    if (imu_msgs_.angular_velocity.z > -0.01 && imu_msgs_.angular_velocity.z < 0.01 ){
        imu_msgs_.angular_velocity.z = 0;
	}

	imu_msgs_.linear_acceleration.x = imu_data_.acc_x;
	imu_msgs_.linear_acceleration.y = imu_data_.acc_y;
	imu_msgs_.linear_acceleration.z = imu_data_.acc_z;

	imu_msgs_.angular_velocity_covariance[0] = 0.00001;
    imu_msgs_.angular_velocity_covariance[4] = 0.00001;
    imu_msgs_.angular_velocity_covariance[8] = 0.00001;

	imu_msgs_.linear_acceleration_covariance[0] = 0.00001;
    imu_msgs_.linear_acceleration_covariance[4] = 0.00001;
    imu_msgs_.linear_acceleration_covariance[8] = 0.00001;

	imu_pub_.publish(imu_msgs_);
}

void RikibotDriver::publisherBattery()
{
    if((ros::Time::now() - now_).toSec() > 1/BATTERY_RATE){
        bat_msgs_.data = ((double)((int16_t)(rxbuf[22]*256+rxbuf[23]))/100);
        if (bat_msgs_.data != 0){
            bat_pub_.publish(bat_msgs_);
            now_ = ros::Time::now();
        }
    }
}

void RikibotDriver::publisherSonar()
{
	sonar_msgs_.data = ((double)((int16_t)(rxbuf[24]*256+rxbuf[25]))/10);
    sonar_pub_.publish(sonar_msgs_);
}

void RikibotDriver::publisherInfrared()
{
	infrared_msgs_.data[0] = rxbuf[26];
	infrared_msgs_.data[1] = rxbuf[27];
	infrared_msgs_.data[2] = rxbuf[28];
	infrared_msgs_.data[3] = rxbuf[29];
	infrared_pub_.publish(infrared_msgs_);
}


int main(int argc,char** argv)
{
    ros::init(argc,argv,"Rikibot_driver_node");
    RikibotDriver driver;
    driver.loop();
    return 0;
}

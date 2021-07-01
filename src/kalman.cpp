//
// Created by huangyujun on 4/16/21.
//

#include "/home/huangyujun/graduation_ws/src/kalman_location_fusion/include/zzw.h"
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/PoseStamped.h>
#include "../include/kalman.h"
#include "../include/zzw.h"
#include <sensor_msgs/NavSatFix.h>
#include <std_msgs/UInt32.h>
#include "kalman_location_fusion/zzw.h"


//#include "TimeConverter.h"

//打中文测试
ros::Publisher kalman_pose_pub;
ros::Publisher final_pose_pub;


geometry_msgs::PoseStamped g_latest_pose;
geometry_msgs::PoseStamped final_pose;


sensor_msgs::NavSatFix GPSposition;
int32_t sat_stars;

Kalman kalman;


//camera_frame_rate 不准
double camera_frame_rate = 20;
double imu_rate = 50;//rostopic hz name 读出来的
double pose_pub_rate = 30;


class ConstPtr;



//imu回调函数
void imu_callback(const sensor_msgs::Imu::ConstPtr& msg)
{
//    //四元数，其实用不上
//    geometry_msgs::Quaternion imu_orientation;
//    imu_orientation.x=msg->orientation.x;
//    imu_orientation.y=msg->orientation.y;
//    imu_orientation.z=msg->orientation.z;
//    imu_orientation.w=msg->orientation.w;
//    //float[9] imu_orientation_covariance;//=msg.orientation_covariance;

//    //角速度，其实用不上
//    geometry_msgs::Vector3 imu_angular_velocity;
//    imu_angular_velocity.x=msg->angular_velocity.x;
//    imu_angular_velocity.y=msg->angular_velocity.y;
//    imu_angular_velocity.z=msg->angular_velocity.z;

    //线加速度，只用到这个了
    //geometry_msgs::Vector3 imu_linear_acceleration;
    Eigen::Vector2d imu_linear_acceleration;
    imu_linear_acceleration(0)=-msg->linear_acceleration.y;
    imu_linear_acceleration(1)=msg->linear_acceleration.x;

    //z方向的线加速度，用不上
    //imu_linear_acceleration.z=msg->linear_acceleration.z;

    //std::cout<<"imu"<<std::endl;

    kalman.predict(imu_linear_acceleration,msg->header);
    //std::cout<<"imu,predict"<<std::endl;
}

//二维码回调函数,放在一个2x1的矩阵里
void aruco_callback(const kalman_location_fusion::zzw::ConstPtr& msg)
{
//    geometry_msgs::Pose aruco_pose;
//    aruco_pose.position.x=msg->pose.position.x;
//    aruco_pose.position.y=msg->pose.position.y;
//    aruco_pose.position.z=msg->pose.position.z;
    Eigen::Vector2d aruco_pose;
    aruco_pose(0)=msg->pose.position.x+8.2087932845;
    aruco_pose(1)=-msg->pose.position.z-4.5674296343;


    //std::cout<<"aruco"<<std::endl;

    kalman.update(aruco_pose,msg->header);

    //std::cout<<"aruco,update"<<std::endl;

}

//GPS数据回调函数
void gpsraw_callback(const sensor_msgs::NavSatFix::ConstPtr& msg)
{
GPSposition.header.stamp=ros::Time::now();
GPSposition.latitude=msg->latitude;
GPSposition.altitude=msg->altitude;
GPSposition.longitude=msg->longitude;
GPSposition.position_covariance=msg->position_covariance;
GPSposition.position_covariance_type=msg->position_covariance_type;
GPSposition.status=msg->status;

}

//GPS信号回调函数
void gpssat_callback(const std_msgs::UInt32::ConstPtr& msg)
{
sat_stars=msg->data;

}

//定时器回调函数,放在一个2x1的矩阵里
void pose_pub_timer_callback(const ros::TimerEvent&)
{
    //实时跑的时候改这里
    g_latest_pose.header=kalman.getheader();
    g_latest_pose.pose=kalman.getpose();

    std::cout<<"time getheader()"<<std::endl;
    //实时跑的时候改这里
    final_pose.header=kalman.getheader();

    kalman_pose_pub.publish(g_latest_pose);

    if (sat_stars>6)
    {
        final_pose.pose.position.x=GPSposition.altitude;
        final_pose.pose.position.y=GPSposition.latitude;
        final_pose.pose.position.z=GPSposition.longitude;
        final_pose_pub.publish(final_pose);
    } else
    {
        final_pose.pose=g_latest_pose.pose;
        final_pose_pub.publish(final_pose);
    }
}

int main(int argc, char **argv)
{
    // 初始化ROS节点
    ros::init(argc, argv, "kalman_node");

    // 创建节点句柄
    ros::NodeHandle nh;

    kalman_pose_pub = nh.advertise<geometry_msgs::PoseStamped>("/kalman_location_fusion/pose", 10);
    final_pose_pub = nh.advertise<geometry_msgs::PoseStamped>("/kalman_location_fusion/final/pose", 10);

    //imu的Subscriber
    ros::Subscriber imu_sub = nh.subscribe<sensor_msgs::Imu>("/mavros/imu/data",1,imu_callback);

    //二维码的Subscriber
    ros::Subscriber aruco_sub = nh.subscribe<kalman_location_fusion::zzw>("/aruco_detect/send_data_small",1,aruco_callback);

    //GPS位置的Subscriber
    ros::Subscriber GPSpositopn_sub = nh.subscribe<sensor_msgs::NavSatFix>("/marvous/global_position/raw/fix",1,gpsraw_callback);

    //GPS信号强弱的Subscriber
    ros::Subscriber GPSsat_sub = nh.subscribe<std_msgs::UInt32>("/marvous/global_position/raw/satellites",1,gpssat_callback);

    ros::spinOnce();
    ros::Timer pose_pub_timer = nh.createTimer(ros::Duration(1/pose_pub_rate), pose_pub_timer_callback);

    // 循环等待回调函数
    ros::spin();

    return 0;
}
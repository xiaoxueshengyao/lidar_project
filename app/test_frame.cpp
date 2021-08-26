/***
 * Descriptions:利用GPS和IMU数据显示点运数据，相当于用GPS和imu做里程计
 * 用了GPS的位置信息和IMU的旋转信息，相对结构化的程序
 * Author: Capta1nY
 * Data: 210329
 * 0402调试，在 RVIZ中Odometry/Keep射大一点可以保存所有的odom信息
 * Odometry/Head Length和Head Radius 设置里程计显示大小
 * 0403原文中还有imu到lidiar的坐标转换，使用tf发布转换矩阵然后相乘
 * ****/

#include <iostream>
#include <deque>

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>         //imu话题
#include <sensor_msgs/NavSatFix.h>   //gps话题
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>

#include "kitti_data/imu_data.hpp"
#include "kitti_data/gnss_data.hpp"
#include "kitti_data/cloud_data.hpp"
#include "kitti_data/velocity_data.hpp"

#include "publisher/cloud_publisher.hpp"
#include "publisher/odometry_publisher.hpp"

#include "tf_listener.hpp"

#include <pcl_conversions/pcl_conversions.h>//ros2PCL
#include <pcl/common/transforms.h>

#include "glog/logging.h"

using namespace lidar_project;

std::deque<CloudData> cloud_data_all;
std::deque<IMUData> imu_data_all;
std::deque<GNSSData> gnss_data_all;


void PointcloudCallback(const sensor_msgs::PointCloud2ConstPtr& kitti_laser){
    //需要把数据进行存储，可以使用全局变量，或者用sub类
    CloudData cloud_buff;
    cloud_buff.time = kitti_laser->header.stamp.toSec();
    pcl::fromROSMsg(*kitti_laser,*(cloud_buff.cloud_ptr));

    std::deque<CloudData> new_cloud;
    new_cloud.push_back(cloud_buff);
    if(new_cloud.size()>0){
        cloud_data_all.insert(cloud_data_all.end(),new_cloud.begin(),new_cloud.end());
    }
    new_cloud.clear();
    
}

void ImuCallback(const sensor_msgs::ImuConstPtr& kitti_imu){
    IMUData imu_data;
    imu_data.time = kitti_imu->header.stamp.toSec();
    imu_data.angular_velocity.x = kitti_imu->angular_velocity.x;//对角速度积分得到角度
    imu_data.angular_velocity.y = kitti_imu->angular_velocity.y;
    imu_data.angular_velocity.z = kitti_imu->angular_velocity.z;

    imu_data.linear_acceleration.x = kitti_imu->linear_acceleration.x;//对加速度二次积分得到位移
    imu_data.linear_acceleration.y = kitti_imu->linear_acceleration.y;
    imu_data.linear_acceleration.z = kitti_imu->linear_acceleration.z;

    imu_data.orientation.w = kitti_imu->orientation.w;
    imu_data.orientation.x = kitti_imu->orientation.x;
    imu_data.orientation.y = kitti_imu->orientation.y;
    imu_data.orientation.z = kitti_imu->orientation.z;

    std::deque<IMUData> new_imu;
    new_imu.push_back(imu_data);
    if(new_imu.size()>0){
        imu_data_all.insert(imu_data_all.end(),new_imu.begin(),new_imu.end());
    }
    new_imu.clear();


}

void GnssCallback(const sensor_msgs::NavSatFixConstPtr& kitti_gnss){
    GNSSData gnss_data;
    gnss_data.time = kitti_gnss->header.stamp.toSec();
    gnss_data.altitude = kitti_gnss->altitude;
    gnss_data.longitude = kitti_gnss->longitude;
    gnss_data.latitude = kitti_gnss->latitude;

    gnss_data.service = kitti_gnss->status.service;     //which global navigation satellite system
    gnss_data.status = kitti_gnss->status.status;       //fix type

    std::deque<GNSSData> new_gnss;
    new_gnss.push_back(gnss_data);
    if(new_gnss.size()>0){
        gnss_data_all.insert(gnss_data_all.end(),new_gnss.begin(),new_gnss.end());
    }

}



int main(int argc,char** argv){

  google::InitGoogleLogging(argv[0]);  
  
  ros::init(argc,argv,"test_frame");
  ros::NodeHandle nh;
  
  //话题订阅
  ros::Subscriber pointcloud_sub = nh.subscribe("/kitti/velo/pointcloud",100000,PointcloudCallback);
  ros::Subscriber imu_sub = nh.subscribe("/kitti/oxts/imu",1000000,ImuCallback);
  ros::Subscriber gnss_sub = nh.subscribe("/kitti/oxts/gps/fix",1000000,GnssCallback);
  TFListener lidar_to_imu_tf(nh,"velo_link","imu_link");//最终的转换就是冲lidar->IMU->map

  //话题发布
  ros::Publisher cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("current_scan",100);
  ros::Publisher odometry_pub = nh.advertise<nav_msgs::Odometry>("lidar_odom",100);
  

  bool gnss_pose_inited = false;
  bool transform_received = false;
  Eigen::Matrix4f lidar_to_imu_mat = Eigen::Matrix4f::Identity();


  ros::Rate rate(100);
  while(ros::ok()){
      ros::spinOnce();
      if(!transform_received){
          if(lidar_to_imu_tf.LookupData(lidar_to_imu_mat)){
              transform_received = true;
              continue;
          }
      }

      while(cloud_data_all.size()>0 && imu_data_all.size()>0 && gnss_data_all.size()>0){
          //使用队列的好处就是不会因为处理时间过长的问题导致数据不统一
        CloudData cloud_data = cloud_data_all.front();
        IMUData imu_data = imu_data_all.front();
        GNSSData gnss_data = gnss_data_all.front();

        double diff_time = cloud_data.time - imu_data.time;
        if(diff_time < -0.05){
            cloud_data_all.pop_front();//点云数据早了，imu.gps留下和下一帧点云一起处理
        }else if(diff_time > 0.05){
            imu_data_all.pop_front();
            gnss_data_all.pop_front();//imu,gps数据早了
        }else{
            cloud_data_all.pop_front();
            imu_data_all.pop_front();
            gnss_data_all.pop_front();//都弹出，下次处理下一帧
        }

        Eigen::Matrix4f odom_mat;
        if(!gnss_pose_inited){
            gnss_data.InitOriginPose();
            gnss_pose_inited = true;
        }
        
        gnss_data.UpdateXYZ();//经纬度转笛卡尔

        //用GPS的位置信息，用IMU的旋转信息
        odom_mat.topRightCorner(3,1) << gnss_data.local_x,gnss_data.local_y,gnss_data.local_z;
        odom_mat.block(0,0,3,3) = imu_data.GetRotateMat();
        odom_mat *= lidar_to_imu_mat;
        // std::cout<<odom_mat<<std::endl;

        pcl::transformPointCloud(*cloud_data.cloud_ptr,*cloud_data.cloud_ptr,odom_mat);

        sensor_msgs::PointCloud2Ptr cloud_out(new sensor_msgs::PointCloud2());
        pcl::toROSMsg(*cloud_data.cloud_ptr,*cloud_out);
        cloud_out->header.stamp = ros::Time::now();
        cloud_out->header.frame_id = "map";
        
        nav_msgs::Odometry odomtry_mat;
        odomtry_mat.header.frame_id = "map";
        odomtry_mat.child_frame_id = "lidar";
        odomtry_mat.header.stamp = ros::Time::now();
        odomtry_mat.pose.pose.position.x = odom_mat(0,3);
        odomtry_mat.pose.pose.position.y = odom_mat(1,3);
        odomtry_mat.pose.pose.position.z = odom_mat(2,3);
        Eigen::Matrix3f R = odom_mat.block(0,0,3,3).matrix();//注意这里给四元数赋值的方式
        Eigen::Quaternionf q(R);
        odomtry_mat.pose.pose.orientation.w = q.w();
        odomtry_mat.pose.pose.orientation.x = q.x();
        odomtry_mat.pose.pose.orientation.y = q.y();
        odomtry_mat.pose.pose.orientation.z = q.z();

        cloud_pub.publish(*cloud_out);
        odometry_pub.publish(odomtry_mat);      
      }

      rate.sleep();
  }

  return 0;
}
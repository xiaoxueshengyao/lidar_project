/****
 * Des: 写个前段的流程，相当于任务管理，这样所有功能都放在类里，main中就创建个对象
 * Author: Capta1nY
 * Data:0415
 * 0601 继续添加速度信息
 * 0604 添加路径保存，便于评价里程计精度
 * ***/

#ifndef FRONT_END_FLOW_HPP_
#define FRONT_END_FLOW_HPP_


#include <ros/ros.h>
#include "subscriber/cloud_subscriber.hpp" //数据订阅
#include "subscriber/imu_subscriber.hpp"
#include "subscriber/gnss_subscriber.hpp"
#include "subscriber/velocity_subscriber.hpp"
#include "tf_listener.hpp"     //lidar-imu的tf
#include "publisher/odometry_publisher.hpp"  //数据发布
#include "publisher/cloud_publisher.hpp"
#include "front_end.hpp"       //前端内容
#include "general_models/undistorted/undistorted.hpp"

namespace lidar_project{
class FrontEndFlow{
    public:
      FrontEndFlow(ros::NodeHandle& nh);

      bool Run();//所有流程在这
      bool SaveMap();
      bool PublisheGlobalMap();

    private:
      bool ReadData();
      bool InitClibration();//得到lidar到imu的转换
      bool InitGNSS();
      bool HasData();
      bool ValidData();
      bool UpdateGNSSOdometry();
      bool UpdateLaserOdometry();
      bool PublishData();
      bool SaveTrajectory();


    private:
      //数据订阅
      std::shared_ptr<CloudSubscriber> cloud_sub_ptr_;
      std::shared_ptr<IMUSubscriber> imu_sub_ptr_;
      std::shared_ptr<GNSSSubscriber> gnss_sub_ptr_;
      std::shared_ptr<VelocitySubscriber> velocity_sub_ptr_;
      std::shared_ptr<TFListener> lidar_to_imu_ptr_;
      //点云发布
      std::shared_ptr<CloudPublisher> cloud_pub_ptr_;
      std::shared_ptr<CloudPublisher> local_map_pub_ptr_;
      std::shared_ptr<CloudPublisher> global_map_pub_ptr_;
      //里程计发布
      std::shared_ptr<OdometryPublisher> laser_odom_pub_ptr_;
      std::shared_ptr<OdometryPublisher> gnss_pub_ptr_;
      //前端
      std::shared_ptr<FrontEnd> front_end_ptr_;

      //数据存储
      std::deque<CloudData> cloud_data_buff_;
      std::deque<IMUData> imu_data_buff_;
      std::deque<GNSSData> gnss_data_buff_;
      std::deque<VelocityData> velocity_data_buff_;

      Eigen::Matrix4f lidar_to_imu_ = Eigen::Matrix4f::Identity();
      CloudData current_cloud_data_;
      IMUData current_imu_data_;
      GNSSData current_gnss_data_;
      VelocityData current_velocity_data_;

      CloudData::CloudPtr local_map_ptr_;
      CloudData::CloudPtr global_map_ptr_;
      CloudData::CloudPtr current_scan_ptr_;

      Eigen::Matrix4f gnss_odometry_ = Eigen::Matrix4f::Identity();
      Eigen::Matrix4f laser_odometry_ = Eigen::Matrix4f::Identity();

      //畸变矫正指针
      std::shared_ptr<DistortionAdjust> distortion_adjust_ptr_;

};


}






#endif
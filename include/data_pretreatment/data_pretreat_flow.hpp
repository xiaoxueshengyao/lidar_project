/***
 * Description:把整个系统分为五大模块，分别是
 * 数据预处理、前端、后端、回环以及显示
 * 当前是数据预处理模块，主要包括原始数据订阅、时间同步、点云畸变补偿
 * Data:0609
 * ****/

#ifndef DATA_PRETREAT_FLOW_HPP_
#define DATA_PRETREAT_FLOW_HPP_

//订阅话题
#include <ros/ros.h>
#include "subscriber/cloud_subscriber.hpp"          //点云
#include "subscriber/imu_subscriber.hpp"            //IMU
#include "subscriber/velocity_subscriber.hpp"       //velocity
#include "subscriber/gnss_subscriber.hpp"           //GNSS
#include "tf_listener.hpp"                          //IMU和雷达的坐标转换

//发布话题
#include "publisher/cloud_publisher.hpp"
#include "publisher/odometry_publisher.hpp"

//畸变矫正
#include "general_models/undistorted/undistorted.hpp"

namespace lidar_project{
class DataPretreatFlow{
    public:
        DataPretreatFlow(ros::NodeHandle& nh);
        bool Run();

    private:
        bool ReadData();
        bool InitCalibration();
        bool InitGNSS();
        bool HasData();
        bool ValidData();
        bool TransformsData();
        bool PublishData();

    private:
        //数据订阅
        std::shared_ptr<CloudSubscriber> cloud_sub_ptr_;
        std::shared_ptr<IMUSubscriber> imu_sub_ptr_;
        std::shared_ptr<GNSSSubscriber> gnss_sub_ptr_;
        std::shared_ptr<VelocitySubscriber> velocity_sub_ptr_;
        std::shared_ptr<TFListener> lidar_to_imu_ptr_;
        //点云发布
        std::shared_ptr<CloudPublisher> cloud_pub_ptr_;
        //gnss里程计发布
        std::shared_ptr<OdometryPublisher> gnss_pub_ptr_;
        //畸变矫正
        std::shared_ptr<DistortionAdjust> distortion_adjust_ptr_; 

        Eigen::Matrix4f lidar_to_imu_ = Eigen::Matrix4f::Identity();

        //数据存储
        std::deque<CloudData> cloud_data_buff_;
        std::deque<IMUData> imu_data_buff_;
        std::deque<GNSSData> gnss_data_buff_;
        std::deque<VelocityData> velocity_data_buff_;

        //当前帧处理
        CloudData current_cloud_data_;
        IMUData current_imu_data_;
        GNSSData current_gnss_data_;
        VelocityData current_velocity_data_;

        //gnss数据的位姿
        Eigen::Matrix4f gnss_pose_ = Eigen::Matrix4f::Identity();




};



}



#endif
/***
 * 0604参考kitti_data把数据订阅也分开
 * ***/


#ifndef IMU_SUBSCRIBER_HPP_
#define IMU_SUBSCRIBER_HPP_

#include <deque>
#include <ros/ros.h>
#include "sensor_msgs/Imu.h"
#include "kitti_data/imu_data.hpp"


namespace lidar_project{

class IMUSubscriber{
    public:
      IMUSubscriber(ros::NodeHandle& nh,std::string topic_name, size_t buff_size);
      IMUSubscriber() = default;//为显示默认函数生成默认实现
      void ParasData(std::deque<IMUData>& deque_imu_data);

    private:
      void msg_callback(const sensor_msgs::ImuConstPtr& imu_msg_ptr);

    private:
      ros::NodeHandle nh_;
      ros::Subscriber subscriber_;

      std::deque<IMUData> new_imu_data_;
    public:
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};


}


#endif
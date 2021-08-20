/***
 * Description: 使用类来描述数据的订阅，全都放在main上不符合c++思维
 * 可以吧几个传感器的订阅放一起
 * Author:Capta1nY(copy from renqian)
 * Data:0413
 * ***/
#ifndef DATA_SUBSCRIBER_HPP_
#define DATA_SUBSCRIBER_HPP_

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/TwistStamped.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>//ros-pcl
#include <deque>
#include "kitti_data/imu_data.hpp"
#include "kitti_data/gnss_data.hpp"
#include "kitti_data/cloud_data.hpp"
#include "kitti_data/velocity_data.hpp"

namespace lidar_project{
//IMU订阅类
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

};


//GNSS订阅类
class GNSSSubscriber{
  public:
    GNSSSubscriber(ros::NodeHandle& nh_, std::string topic_name, size_t buff_size);
    GNSSSubscriber() = default;
    void ParasData(std::deque<GNSSData>& deque_gnss_data);

  private:
    void msg_callback(const sensor_msgs::NavSatFixConstPtr& gnss_msg_ptr);

  private:
    ros::NodeHandle nh_;
    ros::Subscriber subscriber_;
    std::deque<GNSSData> new_gnss_data_;

};

//点云订阅类
class CloudSubscriber{
  public:
    CloudSubscriber(ros::NodeHandle& nh_,std::string topic_name,size_t buff_size);
    CloudSubscriber() = default;
    void ParaData(std::deque<CloudData>& deque_cloud_data);

  private:
    void msg_callback(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg_ptr);

  private:
    ros::NodeHandle nh_;
    ros::Subscriber subscriber_;
    std::deque<CloudData> new_cloud_data_;


};

//速度信息订阅类
class VelocitySubscriber{
  public:
    VelocitySubscriber(ros::NodeHandle& nh, std::string topic_name,size_t buff_size);
    VelocitySubscriber() = default;
    void ParaData(std::deque<VelocityData>& deque_velocity_data);

  private:
    void msg_callback(const geometry_msgs::TwistStampedConstPtr& twist_msg_ptr);

  private:
    ros::NodeHandle nh_;
    ros::Subscriber subscriber_;
    std::deque<VelocityData> new_velocity_data_;

};


}












#endif
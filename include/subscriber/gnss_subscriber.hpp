/***
 * 0604参考kitti_data把数据订阅也分开
 * ***/

#ifndef GNSS_SUBSCRIBER_HPP_
#define GNSS_SUBSCRIBER_HPP_

#include <deque>
#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>
#include "kitti_data/gnss_data.hpp"

namespace lidar_project
{

//GNSS订阅类
class GNSSSubscriber{
  public:
    GNSSSubscriber(ros::NodeHandle& nh_, std::string topic_name, size_t buff_size);
    GNSSSubscriber() = default;
    void ParasData(std::deque<GNSSData>& deque_gnss_data);

  private:
    void msg_callback(const sensor_msgs::NavSatFixConstPtr& gnss_msg_ptrs);

  private:
    ros::NodeHandle nh_;
    ros::Subscriber subscriber_;
    std::deque<GNSSData> new_gnss_data_;
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

} // namespace lidar_project

#endif


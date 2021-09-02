/***
 * 0604参考kitti_data把数据订阅也分开
 * ***/

#include <deque>
#include <ros/ros.h>
#include "geometry_msgs/TwistStamped.h"
#include "kitti_data/velocity_data.hpp"

namespace lidar_project{

class VelocitySubscriber{
  public:
    VelocitySubscriber(ros::NodeHandle& nh, std::string topic_name,size_t buff_size);//数据订阅
    VelocitySubscriber() = default;
    void ParaData(std::deque<VelocityData>& deque_velocity_data);

  private:
    void msg_callback(const geometry_msgs::TwistStampedConstPtr& twist_msg_ptr);

  private:
    ros::NodeHandle nh_;
    ros::Subscriber subscriber_;
    std::deque<VelocityData> new_velocity_data_;

  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

};



}
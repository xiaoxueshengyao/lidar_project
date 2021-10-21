/***
 * 0604参考kitti_data把数据订阅也分开
 * ***/

#ifndef CLOUD_SUBSCRIBER_HPP_
#define CLOUD_SUBSCRIBER_HPP_


#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include "kitti_data/cloud_data.hpp"
#include <deque>


namespace lidar_project{
class CloudSubscriber{
  public:
    CloudSubscriber(ros::NodeHandle& nh_,std::string topic_name,size_t buff_size);
    CloudSubscriber() = default;
    void ParaData( std::deque<CloudData>& deque_cloud_data);

  private:
    /***点云订阅回调函数***/
    void msg_callback(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg_ptr);

  private:
    ros::NodeHandle nh_;
    ros::Subscriber subscriber_;
    std::deque<CloudData> new_cloud_data_;



};


}


#endif
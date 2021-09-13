/***
 * Des:定于odometry里程计数据
 * Data:0610
 * ***/


#ifndef ODOMETRY_SUBSCRIBER_HPP_
#define ODOMETRY_SUBSCRIBER_HPP_

#include <deque>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>

#include "kitti_data/pose_data.hpp"     //位姿 + 时间

namespace lidar_project{
class OdometrySubscriber{
    public:
        OdometrySubscriber(ros::NodeHandle& nh, std::string topic_name, size_t buff_size);
        OdometrySubscriber() = default;
        void ParseData(std::deque<PoseData,Eigen::aligned_allocator<PoseData>>& deque_pose_data);

    private:
        void msg_callback(const nav_msgs::OdometryConstPtr& odom_msg_ptr);

        
    private:
        ros::NodeHandle nh_;
        ros::Subscriber subscriber_;
        
        std::deque<PoseData> new_pose_data_;
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};



}





#endif
/***
 * Description:回环位姿的订阅
 * Data:1012
 * ***/

#ifndef LOOP_POSE_SUBSCRIBER_HPP_
#define LOOP_POSE_SUBSCRIBER_HPP_

#include <deque>
#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include "kitti_data/loop_pose.hpp"


namespace lidar_project{
class LoopPoseSubscriber{
    public:
        LoopPoseSubscriber(ros::NodeHandle& nh, std::string topic_name,size_t buff_size);
        LoopPoseSubscriber() = default;
        void ParseData(std::deque<LoopPose>& loop_pose_buff);

    private:
        void msg_callback(const geometry_msgs::PoseWithCovarianceStampedConstPtr& loop_pose_msg_ptr);
    private:
        ros::NodeHandle nh_;
        ros::Subscriber subscriber_;
        std::deque<LoopPose> new_loop_pose_;//存放当前订阅的数据

};
}

#endif
/***
 * Description:发送闭环检测的相对位姿
 * Data:1012
 * ****/

#ifndef LOOP_POSE_PUBLISHER_HPP_
#define LOOP_POSE_PUBLISHER_HPP_

#include <string>
#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>//发布消息的类型,带时间戳的位姿估计

#include "kitti_data/loop_pose.hpp"

namespace lidar_project{
class LoopPosePublisher{
    public:
        LoopPosePublisher(ros::NodeHandle& nh,
                          std::string topic_name,
                          std::string frame_id,
                          int buff_size);//构造函数，确定话题、坐标系等
        LoopPosePublisher() = default;

        void Publish(LoopPose& loop_pose);//id + 位姿
        bool HasSubscriberss();

    private:
        ros::NodeHandle nh_;
        ros::Publisher publisher_;
        std::string frame_id_ = "";
};





}




#endif
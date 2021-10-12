/***
 * Description:回环位姿的订阅
 * Data:1012
 * ***/

#include "subscriber/loop_pose_subscriber.hpp"
#include "glog/logging.h"

namespace lidar_project{
LoopPoseSubscriber::LoopPoseSubscriber(ros::NodeHandle& nh, std::string topic_name, size_t buff_size)
    :nh_(nh){
        subscriber_ = nh.subscribe(topic_name,buff_size,&LoopPoseSubscriber::msg_callback, this);
}

//回调函数，主要把数据放到deque中
void LoopPoseSubscriber::msg_callback(const geometry_msgs::PoseWithCovarianceStampedConstPtr& loop_pose_msg_ptr){
    LoopPose loop_pose;
    loop_pose.time = loop_pose_msg_ptr->header.stamp.toSec();
    loop_pose.index0 = (unsigned int)loop_pose_msg_ptr->pose.covariance[0];
    loop_pose.index1 = (unsigned int)loop_pose_msg_ptr->pose.covariance[1];

    loop_pose.pose(0,3) = loop_pose_msg_ptr->pose.pose.position.x;
    loop_pose.pose(1,3) = loop_pose_msg_ptr->pose.pose.position.y;
    loop_pose.pose(2,3) = loop_pose_msg_ptr->pose.pose.position.z;

    //旋转
    Eigen::Quaternionf q;
    q.x() = loop_pose_msg_ptr->pose.pose.orientation.x;
    q.y() = loop_pose_msg_ptr->pose.pose.orientation.y;
    q.z() = loop_pose_msg_ptr->pose.pose.orientation.z;
    q.w() = loop_pose_msg_ptr->pose.pose.orientation.w;
    loop_pose.pose.block<3,3>(0,0) = q.matrix();

    new_loop_pose_.push_back(loop_pose);
}

//载入数据
//OUTPUT:loop_pose_buff--把所有的数据insert进去
void LoopPoseSubscriber::ParseData(std::deque<LoopPose>& loop_pose_buff){
    if(new_loop_pose_.size()>0){
        //新读进来的数据放到一起
        loop_pose_buff.insert(loop_pose_buff.end(),new_loop_pose_.begin(),new_loop_pose_.end());
        new_loop_pose_.clear();
    }
}


}
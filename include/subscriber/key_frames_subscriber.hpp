/***
 * Des:订阅关键帧数据
 * Data:0610
 * 干嘛用两个相似的文件
 * ***/

#ifndef KEY_FRAMES_SUBSCRIBER_HPP_
#define KEY_FRAMES_SUBSCRIBER_HPP_

#include <deque>
#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>

#include "kitti_data/key_frame.hpp" //关键帧信息，包括时间、序号、姿态

namespace lidar_project{
class KeyFramesSubscriber{
    public:
        KeyFramesSubscriber(ros::NodeHandle& nh, std::string topic_name, size_t buff_size);
        KeyFramesSubscriber() = default;
        void ParseData(std::deque<KeyFrame>& deque_key_frames);

    private:
        void msg_callback(const nav_msgs::Path::ConstPtr& key_frames_msg_ptr);

    private:
        ros::NodeHandle nh_;
        ros::Subscriber subscriber_;
        std::deque<KeyFrame> new_key_frames_;
};


}




#endif
/***
 * Des:订阅关键帧数据
 * Data:0610
 * ***/


#ifndef KEY_FRAME_SUBSCRIBER_HPP_
#define KEY_FRAME_SUBSCRIBER_HPP_

#include <deque>
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>

#include "kitti_data/key_frame.hpp"


namespace lidar_project{
class KeyFrameSubscriber{
    public:
        KeyFrameSubscriber(ros::NodeHandle& nh, std::string topic_name, size_t buff_size);
        KeyFrameSubscriber() = default;
        void ParseData(std::deque<KeyFrame>& key_frame_buff);

    private:
        void msg_callback(const geometry_msgs::PoseStampedConstPtr& key_frame_msg_ptr);//注意这里的回调函数与frams_sub.hpp里面的参数不同

    private:
        ros::NodeHandle nh_;
        ros::Subscriber subscriber_;
        std::deque<KeyFrame> new_key_frame_;
};


}
#endif
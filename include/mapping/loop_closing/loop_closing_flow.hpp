/***
 * Description:回环流程管理
 * Data:1012
 * ***/
#ifndef LOOP_CLOSING_FLOW_HPP_
#define LOOP_CLOSING_FLOW_HPP_

#include <deque>
#include <ros/ros.h>

//subscriber
#include "subscriber/key_frame_subscriber.hpp"
//pub
#include "publisher/loop_pose_publisher.hpp"
#include "mapping/loop_closing/loop_closing.hpp"


namespace lidar_project{
class LoopClosingFlow{
    public:
        LoopClosingFlow(ros::NodeHandle& nh);

        bool Run();

    private:
        bool ReadData();
        bool HasData();
        bool ValidData();
        bool PublishData();

    private:
        //订阅指针
        std::shared_ptr<KeyFrameSubscriber> key_frame_sub_ptr_;
        std::shared_ptr<KeyFrameSubscriber> key_gnss_sub_ptr_;
        //发布指针
        std::shared_ptr<LoopPosePublisher> loop_pose_pub_ptr_;
        //回环检测
        std::shared_ptr<LoopClosing> loop_closing_ptr_;

        //s数据存储
        std::deque<KeyFrame> key_frame_buff_;
        std::deque<KeyFrame> key_gnss_buff_;

        KeyFrame current_key_frame_;
        KeyFrame current_key_gnss_;
};



}




#endif
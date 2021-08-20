/****
 * Description:单个keyframe信息发布
 * Data:0610
 * ***/

#ifndef KEY_FRAME_PUBLISHER_HPP_
#define KEY_FRAME_PUBLISHER_HPP_

#include <ros/ros.h>
#include <string>
#include <geometry_msgs/PoseStamped.h>

#include "kitti_data/key_frame.hpp"

namespace lidar_project{
class KeyFramePublisher{
    public:
        KeyFramePublisher(ros::NodeHandle& nh,
                          std::string topic_name,
                          std::string frame_id,
                          int buff_size);
        KeyFramePublisher() = default;


        void Publish(KeyFrame& key_frame);
        bool HasSubscribers();

    private:
        ros::NodeHandle nh_;
        ros::Publisher publisher_;
        std::string frame_id_ = "";

};

}






#endif
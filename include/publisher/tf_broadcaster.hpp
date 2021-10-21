/*****
 * Description: 发布TF转换
 * Data: 1019
 * ******/

#ifndef TF_BROADCASTER_HPP_
#define TF_BROADCASTER_HPP_

#include <string>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <Eigen/Dense>

namespace lidar_project{
class TFBroadCaster{
    public:
        TFBroadCaster(std::string frame_id, std::string child_frame_id);
        TFBroadCaster() = default;
        void SendTransform(Eigen::Matrix4f pose, double time);

    protected:
        tf::StampedTransform transform_;
        tf::TransformBroadcaster broadcaster_;
};

}

#endif
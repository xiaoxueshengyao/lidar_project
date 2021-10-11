/***
 * Description:回环位姿，历史帧与当前帧
 * Data:1011
 * ***/

#include "kitti_data/loop_pose.hpp"
namespace lidar_project{
Eigen::Quaternionf LoopPose::GetQuaternion(){
    Eigen::Quaternionf q;
    q = pose.block<3,3>(0,0);

    return q;
}
}
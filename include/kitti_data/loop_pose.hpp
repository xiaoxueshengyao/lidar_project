/***
 * Description:关键帧之间的相对位姿
 * Data:1011
 * ***/

#ifndef LOOP_POSE_HPP_
#define LOOP_POSE_HPP_

#include <Eigen/Dense>

namespace lidar_project{
class LoopPose{
    public:
        double time = 0.0;
        unsigned int index0 = 0;//历史帧的index
        unsigned int index1 = 1;//当前帧的index
        Eigen::Matrix4f pose = Eigen::Matrix4f::Identity();

    public:
        Eigen::Quaternionf GetQuaternion();
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
};

}



#endif
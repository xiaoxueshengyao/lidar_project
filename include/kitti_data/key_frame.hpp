/****
 * Description：关键帧数据类，用于在各个模块间的数据传递
 * Data：0609
 * ****/


#ifndef KEY_FRAME_HPP_
#define KEY_FRAME_HPP_

#include <Eigen/Dense>

namespace lidar_project{
class KeyFrame{
    public:
        double time = 0.0;
        unsigned int index = 0;                                 //关键帧序号
        Eigen::Matrix4f pose = Eigen::Matrix4f::Identity();     //关键帧位姿

        Eigen::Quaternionf GetQuaternion();

};


}

#endif
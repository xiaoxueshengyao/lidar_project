/***
 * Description:存放处理后的IMU姿态 he GNSS位置
 * 代码结构改了以后新建的数据
 * Data：0609
 * ***/


#ifndef POSE_DATA_HPP_
#define POSE_DATA_HPP_

#include <Eigen/Dense>
#include "Eigen/StdDeque"

namespace lidar_project{
class PoseData{

    public:
        Eigen::Matrix4f pose = Eigen::Matrix4f::Identity();
        double time = 0.0;

        Eigen::Quaternionf GetQuaternion();
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
}





#endif
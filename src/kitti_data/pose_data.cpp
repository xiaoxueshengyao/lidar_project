/***
 * 0609  pose数据
 * ***/


#include "kitti_data/pose_data.hpp"

namespace lidar_project{
Eigen::Quaternionf PoseData::GetQuaternion(){
    Eigen::Quaternionf q;
    q = pose.block(0,0,3,3);

    return q;
}



}
/***
 * 0609  pose数据
 * ***/


#include "kitti_data/pose_data.hpp"

namespace lidar_project{
Eigen::Quaternionf PoseData::GetQuaternion(){
    Eigen::Quaternionf q;
    //q = pose.block(0,0,3,3);
    q = pose.block<3,3>(0,0);

    return q;
}

PoseData & PoseData::operator=(const PoseData& p){
    pose = p.pose;
    time = p.time;
    return *this;
}

}
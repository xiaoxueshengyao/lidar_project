/***
 * 0609:关键帧实现
 * ***/


#include "kitti_data/key_frame.hpp"



namespace lidar_project{
Eigen::Quaternionf KeyFrame::GetQuaternion(){
    Eigen::Quaternionf q(pose.block<3,3>(0,0));
    //q = pose.block(0,0,3,3);

    return q;
}


}
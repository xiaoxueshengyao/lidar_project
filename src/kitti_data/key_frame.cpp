/***
 * 0609:关键帧实现
 * ***/


#include "kitti_data/key_frame.hpp"



namespace lidar_project{
Eigen::Quaternionf KeyFrame::GetQuaternion(){
    Eigen::Quaternionf q;
    q = pose.block(0,0,3,3);

    return q;
}


}
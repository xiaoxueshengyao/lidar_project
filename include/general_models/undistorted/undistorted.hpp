/****
 * 机械式雷达由于旋转时的运动造成了运动畸变，需要对其进行去畸变
 * 分两步，1、计算相对坐标，获取载体运动信息和时间
 *        2、进行转换计算
 * Capta1nY
 * 0607
 * ***/

#ifndef UNDISTORTED_HPP_
#define UNDISTORTED_HPP_

#include <pcl/common/transforms.h>
#include <Eigen/Dense>
#include "glog/logging.h"

#include "kitti_data/velocity_data.hpp"
#include "kitti_data/cloud_data.hpp"

namespace lidar_project{
class DistortionAdjust{
    public:
        void SetMotionInfo(float scan_period,VelocityData velocity_data);
        bool AdjustCloud(CloudData::CloudPtr& input_cloud_ptr, CloudData::CloudPtr& output_cloud_ptr);

    private:
        inline Eigen::Matrix3f UpdateMatrix(float real_time);

    private:
        float scan_period_;
        Eigen::Vector3f velocity_;
        Eigen::Vector3f angular_rate_;
};



}//namespacee lidar_project



#endif
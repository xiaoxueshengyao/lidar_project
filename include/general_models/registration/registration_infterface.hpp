/****
 * Des:配准接口的基类
 * Author: Capta1nY
 * Data: 0414
 * ***/
#ifndef REGISTRATION_INTERFACE_HPP_
#define REGISTRATION_INTERFACE_HPP_


#include <yaml-cpp/yaml.h>
#include <Eigen/Dense>
#include "kitti_data/cloud_data.hpp"

namespace lidar_project{
class RegistrationInterface{
    public:
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    public:
      virtual ~RegistrationInterface() = default;

      virtual bool SetInputTarget(const CloudData::CloudPtr& input_target) = 0;
      
      virtual bool ScanMatch(const CloudData::CloudPtr& input_source,
                             const Eigen::Matrix4f& predict_pose,
                             CloudData::CloudPtr& result_cloud_ptr,
                             Eigen::Matrix4f& result_pose) = 0;
};



}





#endif 
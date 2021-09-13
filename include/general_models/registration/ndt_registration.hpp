/****
 * Des: NDT配准
 * Author: Capta1nY
 * Data:0414
 * ***/
#ifndef NDT_REGISTRATION_HPP_
#define NDT_REGISTRATION_HPP_



#include "registration_infterface.hpp"
#include <pcl/registration/ndt.h>
#include "glog/logging.h"

namespace lidar_project{
class NDTRegistration:public RegistrationInterface
{
    public:
        NDTRegistration(const YAML::Node& node);//不同的构造函数使用不同场合，给了yaml就读，没有的话也可以自己设定
        NDTRegistration(float res,float step_size,float trans_eps, int max_iteration);

        bool SetInputTarget(const CloudData::CloudPtr& input_tatget) override;
        bool ScanMatch(const CloudData::CloudPtr& input_source,const Eigen::Matrix4f& predict_pose,
                    CloudData::CloudPtr& result_cloud_ptr,Eigen::Matrix4f& result_pose) override;
        
    private:
        pcl::NormalDistributionsTransform<CloudData::PointT,CloudData::PointT>::Ptr ndt_ptr_;
        bool SetRegistrationParam(float res,float step_size,float trans_eps,int max_iteration);

};
}

#endif
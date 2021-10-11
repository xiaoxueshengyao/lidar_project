#include "general_models/registration/ndt_registration.hpp"

namespace lidar_project{

NDTRegistration::NDTRegistration(const YAML::Node& node)
    :ndt_ptr_(new pcl::NormalDistributionsTransform<CloudData::PointT,CloudData::PointT>()){
    float res = node["res"].as<float>();
    float step_size = node["step_size"].as<float>();
    float trans_eps = node["trans_eps"].as<float>();
    int max_iteration = node["max_iteration"].as<int>();
    SetRegistrationParam(res,step_size,trans_eps,max_iteration);
}



NDTRegistration::NDTRegistration(float res,float step_size,float trans_eps, int max_iteration)
    :ndt_ptr_(new pcl::NormalDistributionsTransform<CloudData::PointT,CloudData::PointT>()){
    SetRegistrationParam(res,step_size,trans_eps,max_iteration);

}

bool NDTRegistration::SetRegistrationParam(float res,float step_size,float trans_eps,int max_iteration){
    ndt_ptr_->setResolution(res);
    ndt_ptr_->setTransformationEpsilon(trans_eps);
    ndt_ptr_->setStepSize(step_size);
    ndt_ptr_->setMaximumIterations(max_iteration);

    LOG(INFO) << "NDT parameters: "<<std::endl
              <<"resolution: "<<res<<", "
              <<"step size: "<<step_size<<", "
              <<"transformation epsilon"<<trans_eps<<", "
              <<std::endl;

    return true;
}

bool NDTRegistration::SetInputTarget(const CloudData::CloudPtr& input_tatget){
    ndt_ptr_->setInputTarget(input_tatget);
    return true;
}

/***
 * 点云配准，设定输入源点云，预测位姿（初始位姿）
***/
bool NDTRegistration::ScanMatch(const CloudData::CloudPtr& input_source, 
                                const Eigen::Matrix4f& predict_pose,
                                CloudData::CloudPtr& result_cloud_ptr,
                                Eigen::Matrix4f& result_pose){
    ndt_ptr_->setInputSource(input_source);
    ndt_ptr_->align(*result_cloud_ptr,predict_pose);
    result_pose = ndt_ptr_->getFinalTransformation();

    return true;

}


}
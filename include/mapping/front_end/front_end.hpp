/****
 * Descroption: Lidar odometry defination(copy from renqian)
 * Author: Capta1n
 * Data: 0414
 * 优化，读参数，滤波模块，配准模块，把流程都写在类里
 * ***/
#ifndef FRONT_END_HPP_
#define FRONT_END_HPP_


#include <pcl/filters/voxel_grid.h>
#include <pcl/registration/ndt.h>

#include <Eigen/Dense>
#include <deque>
#include <string>
#include <yaml-cpp/yaml.h>//修改++

#include "kitti_data/cloud_data.hpp"
#include "general_models/cloud_filter/voxel_filter.hpp"
#include "general_models/registration/ndt_registration.hpp"

namespace lidar_project{

//const std::string WORK_SPACE_PATH = "@WORK_SPACE_PATH@";

class FrontEnd{
  public:
    //每一帧包含点云数据和对应位姿，类中声明帧结构体
    struct Frame{
        Eigen::Matrix4f pose = Eigen::Matrix4f::Identity();
        CloudData cloud_data;
    };

  public:
    FrontEnd();

    //添加
    bool Update(const CloudData& cloud_data, Eigen::Matrix4f& cloud_pose);//点云数据处理
    bool SetInitPose(const Eigen::Matrix4f& init_pose);

  private:
    //添加
    bool InitWithConfig();
    bool InitParam(const YAML::Node& config_node);
    bool InitRegistration(std::shared_ptr<RegistrationInterface>& registration_ptr,const YAML::Node& config_node);
    bool InitFilter(std::string filter_user,std::shared_ptr<CloudFilterInterface>& filter_ptr,const YAML::Node& config_node);
    bool UpdateWithNewFrame(const Frame& new_key_frame);


  private:
    std::string data_path_ = "";
    std::shared_ptr<CloudFilterInterface> frame_filter_ptr_;
    std::shared_ptr<CloudFilterInterface> local_map_filter_ptr_;
    std::shared_ptr<RegistrationInterface> registration_ptr_;

    std::deque<Frame> local_map_frames_;

    CloudData::CloudPtr local_map_ptr_;

    Frame current_frame_;

    Eigen::Matrix4f init_pose_ = Eigen::Matrix4f::Identity();

    float key_frame_dis_ = 2.0;
    int local_frame_num_ = 20;
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  
};

}


#endif
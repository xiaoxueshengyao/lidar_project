/****
 * Descroption: Lidar odometry defination(copy from renqian)
 * Author: Capta1n
 * Data: 0407
 * 0414:优化，读参数，滤波模块，配准模块，把流程都写在类里
 * ***/
#ifndef LIDAR_PROJECT_FRONT_END_BACK_HPP_
#define LIDAR_PROJECT_FRONT_END_BACK_HPP_

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/registration/ndt.h>

#include <Eigen/Dense>
#include <deque>

#include "kitti_data.hpp"

#include <yaml-cpp/yaml.h>//修改++
#include "general_models/cloud_filter/voxel_filter.hpp"
#include "general_models/registration/ndt_registration.hpp"

namespace lidar_project{
class FrontEnd{
  public:
    //每一帧包含点云数据和对应位姿
    class Frame{
      public:
        Eigen::Matrix4f pose = Eigen::Matrix4f::Identity();
        CloudData cloud_data;
    };

  public:
    FrontEnd();
    Eigen::Matrix4f Update(const CloudData& cloud_data);//点云数据处理
    bool SetInitPose(const Eigen::Matrix4f& init_pose);
    bool SetPredictPose(const Eigen::Matrix4f& predict_pose);

    bool GetNewLocalMap(CloudData::CloudPtr& local_map_ptr);
    bool GetNewGlobalMap(CloudData::CloudPtr& global_map_ptr);
    bool GetCurrentScan(CloudData::CloudPtr& current_cloud_ptr);



  private:
    
    bool InitParam(const YAML::Node& config_node);
    bool InitDataPath(const YAML::Node& config_node);
    bool InitRegistration(std::shared_ptr<RegistrationInterface>& registration_ptr,const YAML::Node& config_node);
    bool InitFilter(std::string filter_user,std::shared_ptr<CloudFilterInterface>& filter_ptr,const YAML::Node& config_node);
    void UpdateNewFrame(const Frame& new_key_frame);


  private:
    pcl::VoxelGrid<CloudData::PointT> cloud_filter_;
    pcl::VoxelGrid<CloudData::PointT> local_map_filter_;
    pcl::VoxelGrid<CloudData::PointT> display_filter_;
    pcl::NormalDistributionsTransform<CloudData::PointT,CloudData::PointT>::Ptr ndt_ptr_;

    std::deque<Frame> local_map_frames_;
    std::deque<Frame> global_map_frames_;

    bool has_new_local_map_ = false;
    bool has_new_global_map_ = false;
    CloudData::CloudPtr local_map_ptr_;
    CloudData::CloudPtr global_map_ptr_;
    CloudData::CloudPtr result_cloud_ptr_;
    Frame current_frame_;

    Eigen::Matrix4f init_pose_ = Eigen::Matrix4f::Identity();
    Eigen::Matrix4f predict_pose_ = Eigen::Matrix4f::Identity();

  
};

}


#endif
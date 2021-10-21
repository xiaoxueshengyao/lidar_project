/*****
 * Description: 定位的核心函数实现
 * Data: 1019
 * ****/

#ifndef MATCHING_HPP_
#define MATCHING_HPP_

#include <deque>
#include <Eigen/Dense>
#include <yaml-cpp/yaml.h>

#include "kitti_data/cloud_data.hpp"
#include "kitti_data/pose_data.hpp"

#include "general_models/registration/registration_infterface.hpp"
#include "general_models/cloud_filter/filter_interface.hpp"
#include "general_models/cloud_filter/box_filter.hpp"

namespace lidar_project{

class Matching{
    public:
        Matching();
        bool Update(const CloudData& cloud_data, Eigen::Matrix4f& cloud_pose);
        bool SetGNSSPose(const Eigen::Matrix4f& init_pose);

        void GetGlobalMap(CloudData::CloudPtr& global_map);
        CloudData::CloudPtr& GetLoaclMap();
        CloudData::CloudPtr& GetCurrentScan();
        bool HasInited();
        bool HasNewGlobalMap();
        bool HasNewLocalMap();

    private:
        bool InitWithConfig();
        bool InitDataPath(const YAML::Node& config_node);
        bool InitRegistration(std::shared_ptr<RegistrationInterface>& registration_ptr, const YAML::Node& config_node);
        bool InitFilter(std::string filter_user, std::shared_ptr<CloudFilterInterface>& filter_ptr,const YAML::Node& config_node);
        bool InitBoxFilter(const YAML::Node& config_node);

        bool SetInitPose(const Eigen::Matrix4f& init_pose);
        bool InitGlobalMap();
        bool ResetLocalMap(float x, float y, float z);

    private:
        std::string map_path_ = "";
        

        std::shared_ptr<BoxFilter> box_filter_ptr_;
        std::shared_ptr<CloudFilterInterface> frame_filter_ptr_;
        std::shared_ptr<CloudFilterInterface> local_map_filter_ptr_;
        std::shared_ptr<CloudFilterInterface> global_map_filter_ptr_;
        std::shared_ptr<RegistrationInterface> registration_ptr_;

        CloudData::CloudPtr local_map_ptr_;
        CloudData::CloudPtr global_map_ptr_;
        CloudData::CloudPtr current_scan_ptr_;

        Eigen::Matrix4f current_pose_ = Eigen::Matrix4f::Identity();
        Eigen::Matrix4f init_pose_ = Eigen::Matrix4f::Identity();
        Eigen::Matrix4f current_gnss_pose_ = Eigen::Matrix4f::Identity();

        bool has_inited_ = false;
        bool has_new_global_map_ = false;
        bool has_new_local_map_ = false;

};




}



#endif
/***
 * Description:闭环检测算法实现
 * Data:1011
 * ***/

#ifndef LOOP_CLOSING_HPP_
#define LOOP_CLOSING_HPP_

#include <deque>
#include <Eigen/Dense>
#include <pcl/registration/ndt.h> //scan_to_map的匹配
#include <yaml-cpp/yaml.h>

#include "kitti_data/key_frame.hpp"
#include "kitti_data/loop_pose.hpp"
#include "general_models/registration/registration_infterface.hpp"
#include "general_models/cloud_filter/filter_interface.hpp"

namespace lidar_project{
class LoopClosing{
    public:
        LoopClosing();
        bool Update(const KeyFrame key_frame, const KeyFrame key_gnss);
        bool HasNewLoopPose();
        LoopPose& GetCurrentLoopPose();

    private:
        bool InitWithConfig();
        bool InitParam(const YAML::Node& config_node);
        bool InitDataPath(const YAML::Node& config_node);//数据路径
        bool InitRegistration(std::shared_ptr<RegistrationInterface>& registration_ptr,const YAML::Node& config_node);//配准参数
        bool InitFilter(std::string filter_user,std::shared_ptr<CloudFilterInterface>& filter_ptr, const YAML::Node& config_node);//滤波参数

        bool DetectNearestKeyFrame(int& key_frame_index);//核心，寻找最近的历史帧
        bool CloudRegistration(int key_frame_index);//采用scan_to_map的策略进行匹配，减小误差
        bool JointMap(int key_frame_index, CloudData::CloudPtr& map_cloud_ptr,Eigen::Matrix4f& map_pose);//构建个匹配到的历史帧局部地图
        bool JointScan(CloudData::CloudPtr& scan_cloud_ptr, Eigen::Matrix4f& scan_pose);
        bool Registration(CloudData::CloudPtr& map_cloud_ptr,
                          CloudData::CloudPtr& scan_cloud_ptr,
                          Eigen::Matrix4f& scan_pose,
                          Eigen::Matrix4f& result_pose);//scan_to_map
    
    private:
        std::string key_frames_path_ = "";
        int extend_frame_num_ = 3;//历史帧周围三帧作为待配准地图
        int loop_step_ = 10;//每有10个关键帧检测一次回环
        int diff_num_ = 100;//检测100帧之前的关键帧
        float detect_area_ = 10.0;
        float fitness_score_limit_ = 2.0;

        std::shared_ptr<CloudFilterInterface> scan_filter_ptr_;
        std::shared_ptr<CloudFilterInterface> map_filter_ptr_;
        std::shared_ptr<RegistrationInterface> registration_ptr_;

        std::deque<KeyFrame> all_key_frames_;
        std::deque<KeyFrame> all_key_gnss_;

        LoopPose current_loop_pose_;
        bool has_new_loop_pose_ = false;
};



}








#endif
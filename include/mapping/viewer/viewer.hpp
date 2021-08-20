/****
 * Description: 点云的实时显示
 * Data:0611
 * ***/

#ifndef VIEWER_HPP_
#define VIEWER_HPP_

#include <string>
#include <Eigen/Dense>
#include <yaml-cpp/yaml.h>

#include "kitti_data/cloud_data.hpp"
#include "kitti_data/key_frame.hpp"     //关键帧是带序号的
#include "kitti_data/pose_data.hpp"     //posedata只是pose
#include "general_models/cloud_filter/voxel_filter.hpp"

namespace lidar_project{
class Viewer{
    public:
        Viewer();
        bool Update(std::deque<KeyFrame>& new_key_frames,
                    std::deque<KeyFrame>& optimized_key_frames,
                    PoseData transformed_data,
                    CloudData cloud_data);
        bool SaveMap();
        Eigen::Matrix4f& GetCurrentPose();
        CloudData::CloudPtr& GetCurrentScan();

        bool GetLocalMap(CloudData::CloudPtr& local_map_ptr);
        bool GetGlobalMap(CloudData::CloudPtr& global_map_ptr);
        bool HasNewLocalMap();
        bool HasNewGlobalMap();

    private:
        bool InitWithConfig();//根据参数文件初始化参数
        bool InitParam(const YAML::Node& config_node);
        bool InitDataPath(const YAML::Node& config_node);
        bool InitFilter(std::string filter_user,
                        std::shared_ptr<CloudFilterInterface>& filter_ptr,
                        const YAML::Node& config_node);
        void ResetParam();
        bool OptimizeKeyFrames();
        bool JointGlobalMap(CloudData::CloudPtr& global_map_ptr);
        bool JointLocalMap(CloudData::CloudPtr& local_map_ptr);
        bool JointCloudMap(const std::deque<KeyFrame>& key_frames,
                           CloudData::CloudPtr& map_cloud_ptr);

    private:
        std::string data_path_ = "";
        int local_frame_num_ = 20;

        std::string key_frames_path_ = "";
        std::string map_path_ = "";

        //显示前进行滤波，包括当前帧、局部地图、全局地图        
        std::shared_ptr<CloudFilterInterface> frame_filter_ptr_;
        std::shared_ptr<CloudFilterInterface> local_map_filter_ptr_;
        std::shared_ptr<CloudFilterInterface> global_map_filter_ptr_;

        Eigen::Matrix4f pose_to_optimize_ = Eigen::Matrix4f::Identity();
        PoseData optimized_odom_;//当前数据
        CloudData optimized_cloud_;//当前点云012
        
        std::deque<KeyFrame> optimized_key_frames_;
        std::deque<KeyFrame> all_key_frames_;

        bool has_new_global_map_ = false;
        bool has_new_local_map_ = false;
};




}








#endif
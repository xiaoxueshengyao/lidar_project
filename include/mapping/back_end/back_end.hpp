/***
 * Description:后端模块
 * 功能———从里程计位姿中提取关键帧
 *       根据关键帧位姿、GNss约束和闭环检测约束来优化位姿
 * Input : 前段里程计位姿 + GNSS组合导航位姿 +　闭环检测相对位姿
 * Output: 优化后的位姿
 * Data: 0608
 * ***/

#ifndef BACK_END_HPP_
#define BACK_END_HPP_

#include "kitti_data/cloud_data.hpp"
#include "kitti_data/pose_data.hpp"
#include "kitti_data/key_frame.hpp"

#include <string>
#include <deque>
#include <yaml-cpp/yaml.h>
#include <fstream>


namespace lidar_project{
class BackEnd{
    public:
        BackEnd();

        bool Updata(const CloudData& cloud_data, const PoseData& laser_odom, const PoseData& gnss_pose);
        void GetOptimizedKeyFrames(std::deque<KeyFrame>& key_frames_deque);
        bool HasNewKeyFrame();
        bool HasNewOptimized();
        void GetLatestKeyFrame(KeyFrame& key_frame);

    private:
        bool InitWithConfig();                              //关键帧、优化等参数选择
        bool InitParam(const YAML::Node& config_node);      //关键帧+优化条件
        bool InitDataPath(const YAML::Node& config_node);   //数据路径,轨迹等

        void ResetParam();
        bool SaveTrajectory(const PoseData& laser_odom, const PoseData& gnss_pose);
        bool MaybeNewKeyFrame(const CloudData& cloud_data, const PoseData& laser_odom);
        bool MaybeOptimized();

    private:
        std::string key_frame_path_ = "" ;
        std::string trajectory_path_ = "";
        
        std::ofstream ground_truth_ofs_;
        std::ofstream laser_odom_ofs_;

        float key_frame_distance_ = 2.0;
        int optimize_step_with_none_ = 100;             
        int optimize_step_with_gnss_ = 100;             //带gnss数据的优化
        int optimize_step_with_loop_ = 10;              //带回环的优化，优化阈值

        bool has_new_key_frame_ = false;
        bool has_new_optimized_ = false;
        
        Eigen::Matrix4f last_key_pose_ = Eigen::Matrix4f::Identity();
        KeyFrame latest_key_frame_;
        std::deque<KeyFrame> key_frames_deque_;

};





}




#endif
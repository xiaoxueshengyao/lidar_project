/****
 * Des：后端具体实现
 * 功能： 从里程计位姿中提取关键帧
 *       根据关键帧位姿、GNss约束和闭环检测约束来优化位姿
 * Input : 前段里程计位姿 + GNSS组合导航位姿 +　闭环检测相对位姿
 * Output: 优化后的位姿
 * Data:0610
 * ****/

#include "mapping/back_end/back_end.hpp"
#include "general_models/file_manager/file_manager.hpp"

#include <Eigen/Dense>
#include <pcl/io/pcd_io.h>
#include "glog/logging.h"

namespace lidar_project{
BackEnd::BackEnd(){
    InitWithConfig();
}


//参数文件读取，包括关键帧和优化的执行条件以及轨迹文件路径
bool BackEnd::InitWithConfig(){
    std::string config_file_path = "/home/jerry/yjj_ws/src/lidar_project/config/BackEndConfig.yaml";
    YAML::Node config_node = YAML::LoadFile(config_file_path); 

    InitParam(config_node);
    InitDataPath(config_node);

    return true;
}

bool BackEnd::InitParam(const YAML::Node& config_node){
    key_frame_distance_ = config_node["key_frame_distance"].as<float>();
    optimize_step_with_none_ = config_node["optimize_step_with_none"].as<int>();
    optimize_step_with_gnss_ = config_node["optimize_step_with_gnss"].as<int>();
    optimize_step_with_loop_ = config_node["optimize_step_with_loop"].as<int>();

    return true;

}

//各个文件路径初始化
bool BackEnd::InitDataPath(const YAML::Node& config_node){
    std::string data_path = config_node["data_path"].as<std::string>();
    if(data_path == "./"){
        data_path = "/home/jerry/yjj_ws/src/lidar_project";
    }

    if(!FileManager::CreateDirectory(data_path + "/slam_data")){
        return false;
    }

    key_frames_path_ = data_path + "/slam_data/key_frames";
    trajectory_path_ = data_path + "/slam_data/trajectory";

    if(!FileManager::InitDirectory(key_frames_path_,"关键帧点云"))
        return false;
    if(!FileManager::InitDirectory(trajectory_path_,"轨迹文件"))
        return false;
    if(!FileManager::CreateFile(ground_truth_ofs_,trajectory_path_+"/ground_truth.txt"))
        return false;
    if(!FileManager::CreateFile(laser_odom_ofs_,trajectory_path_+"/laser_odom.txt"))
        return false;

    return true;


}

//更新里程计数据
bool BackEnd::Updata(const CloudData& cloud_data, const PoseData& laser_odom,const PoseData& gnss_pose){
    ResetParam();
    SaveTrajectory(laser_odom,gnss_pose);

    if(MaybeNewKeyFrame(cloud_data,laser_odom)){
        MaybeOptimized();
    }
    return true;
}

void BackEnd::ResetParam(){
    has_new_key_frame_ = false;
    has_new_optimized_ = false;
}


//把里程计和真值进行保存
bool BackEnd::SaveTrajectory(const PoseData& laser_odom, const PoseData& gnss_pose){
    for(int i=0; i<3; i++){
        for(int j=0; j<4; j++){
            ground_truth_ofs_ << gnss_pose.pose(i,j);//按行进行存储
            laser_odom_ofs_ << laser_odom.pose(i,j);
            
            if(i==2 && j==3){
                ground_truth_ofs_ << std::endl;
                laser_odom_ofs_ << std::endl;
            }else{
                ground_truth_ofs_ << " ";
                laser_odom_ofs_ << " ";
            }

        }
    }

    return true;

}



//加入关键帧判断
bool BackEnd::MaybeNewKeyFrame(const CloudData& cloud_data,const PoseData& laser_odom){
    if(key_frames_deque_.size() ==0 ){
        has_new_key_frame_ = true;//第一帧设为关键帧
        last_key_pose_ = laser_odom.pose;
    }

    if(fabs(laser_odom.pose(0,3) - last_key_pose_(0,3)) + 
       fabs(laser_odom.pose(1,3) - last_key_pose_(1,3)) + 
       fabs(laser_odom.pose(2,3) - last_key_pose_(2,3)) > key_frame_distance_){
           has_new_key_frame_ = true;
           last_key_pose_ = laser_odom.pose;
    }

    if(has_new_key_frame_){
        //有关键帧后把对应点云存到硬盘
        std::string file_path = key_frames_path_ + "/key_frame_"+std::to_string(key_frames_deque_.size()) +".pcd";
        pcl::io::savePCDFileBinary(file_path,*cloud_data.cloud_ptr);

        KeyFrame key_frame;
        key_frame.time = laser_odom.time;
        key_frame.index = (unsigned int)key_frames_deque_.size();
        key_frame.pose = laser_odom.pose;
        key_frames_deque_.push_back(key_frame);//把关键帧放到存储的队列中

        latest_key_frame_ = key_frame;
    }

    return has_new_key_frame_;

}


//是否优化
bool BackEnd::MaybeOptimized(){
    static int unoptimized_cnt = 0;
    //没关键帧，没回环，100次优化一次
    if(++unoptimized_cnt > optimize_step_with_none_){
        unoptimized_cnt = 0;
        has_new_optimized_ = true;
    }

    return true;
}

void BackEnd::GetOptimizedKeyFrames(std::deque<KeyFrame>& key_frames_deque){

    key_frames_deque = key_frames_deque_;
}

bool BackEnd::HasNewKeyFrame(){
    return has_new_key_frame_;
}


bool BackEnd::HasNewOptimized(){

    return has_new_optimized_;
}

void BackEnd::GetLatestKeyFrame(KeyFrame& key_frame){

    key_frame = latest_key_frame_;
}

}
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
    std::string config_file_path = "/home/jingwan/lslidar_ws/src/lidar_project/config/BackEndConfig.yaml";
    YAML::Node config_node = YAML::LoadFile(config_file_path); 

    InitParam(config_node);
    InitDataPath(config_node);
    InitGraphOptimizer(config_node);//添加后端优化参数初始化，优化库、信息矩阵确定

    return true;
}

bool BackEnd::InitParam(const YAML::Node& config_node){
    key_frame_distance_ = config_node["key_frame_distance"].as<float>();
    // optimize_step_with_none_ = config_node["optimize_step_with_none"].as<int>();
    // optimize_step_with_gnss_ = config_node["optimize_step_with_gnss"].as<int>();
    // optimize_step_with_loop_ = config_node["optimize_step_with_loop"].as<int>();

    return true;

}

//优化参数初始化
bool BackEnd::InitGraphOptimizer(const YAML::Node& config_node){
    std::string graph_optimizer_type = config_node["graph_optimizer_type"].as<std::string>();//目前使用g2o
    if(graph_optimizer_type == "g2o"){
        graph_optimizer_ptr_ = std::make_shared<G2oGraphOptimizer>("lm_var");//接口类优化指针确定
    }else{
        LOG(ERROR)<<"没有找到匹配优化库 "<<graph_optimizer_type << " ， 请检查配置文件";
        return false;
    }
    LOG(INFO) << "后端优化悬则的优化器为：　"<< graph_optimizer_type << std::endl;

    graph_optimizer_config_.use_gnss = config_node["use_gnss"].as<bool>();
    graph_optimizer_config_.use_loop_close = config_node["use_loop_close"].as<bool>();

    graph_optimizer_config_.optimize_step_with_key_frame = config_node["optimize_step_with_key_frame"].as<int>();
    graph_optimizer_config_.optimize_step_with_gnss = config_node["optimize_step_with_gnss"].as<int>();
    graph_optimizer_config_.optimize_step_with_loop = config_node["optimize_step_with_loop"].as<int>();

    for(int i=0; i<6; i++){
        graph_optimizer_config_.odom_edge_noise(i) = 
            config_node[graph_optimizer_type + "_param"]["odom_edge_noise"][i].as<double>();//二级参数使用
        graph_optimizer_config_.close_loop_noise(i) = 
            config_node[graph_optimizer_type + "_param"]["close_loop_noise"][i].as<double>();
    }

    for(int i=0; i<3; i++){
        graph_optimizer_config_.gnss_noise(i) = 
            config_node[graph_optimizer_type + "_param"]["gnss_noise"][i].as<double>();
    }

    return true;
}
//各个文件路径初始化
bool BackEnd::InitDataPath(const YAML::Node& config_node){
    std::string data_path = config_node["data_path"].as<std::string>();
    if(data_path == "./"){
        data_path = "/home/jingwan/lslidar_ws/src/lidar_project";
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
        AddNodeAndEdge(gnss_pose);//插入关键帧时添加对应节点和边
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
    static Eigen::Matrix4f last_key_pose = laser_odom.pose;//通过计算激光里程计变化设定关键帧
    if(key_frames_deque_.size() ==0 ){
        has_new_key_frame_ = true;//第一帧设为关键帧
        last_key_pose = laser_odom.pose;
    }

    if(fabs(laser_odom.pose(0,3) - last_key_pose(0,3)) + 
       fabs(laser_odom.pose(1,3) - last_key_pose(1,3)) + 
       fabs(laser_odom.pose(2,3) - last_key_pose(2,3)) > key_frame_distance_){
           has_new_key_frame_ = true;
           last_key_pose = laser_odom.pose;
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

        // latest_key_frame_ = key_frame;
        current_key_frame_ = key_frame;//这里更换为当前帧
    }

    return has_new_key_frame_;

}


//添加观测节点和边
bool BackEnd::AddNodeAndEdge(const PoseData& gnss_data){
    Eigen::Isometry3d isometry;
    //添加关键帧对应节点
    isometry.matrix() = current_key_frame_.pose.cast<double>();
    graph_optimizer_ptr_->AddSe3Node(isometry,false);
    new_key_frame_cnt_++;
    std::cout<<"The graph has "<<new_key_frame_cnt_<<" key frames"<<std::endl;

    //添加激光里程计对应的边
    //添加了先验边，所以第一个节点fix不需要设置为true,否则会冲突
    static KeyFrame last_key_frame = current_key_frame_;//静态局部变量，使值数据持久
    int node_num = graph_optimizer_ptr_->GetNodeNum();
    if(node_num > 1){
        Eigen::Matrix4f relative_pose = last_key_frame.pose.inverse() * current_key_frame_.pose;//相对位姿计算
        isometry.matrix() = relative_pose.cast<double>();
        graph_optimizer_ptr_->AddSe3Edge(node_num-2, node_num-1, isometry, graph_optimizer_config_.odom_edge_noise);//添加边
        
    }
    last_key_frame = current_key_frame_;

    //添加gnss对应的先验边
    if(graph_optimizer_config_.use_gnss){
        Eigen::Vector3d xyz(static_cast<double>(gnss_data.pose(0,3)),
                            static_cast<double>(gnss_data.pose(1,3)),
                            static_cast<double>(gnss_data.pose(2,3))
        );
        graph_optimizer_ptr_->AddSe3PriorXYZEdge(node_num-1,xyz,graph_optimizer_config_.gnss_noise);//节点先验边，就是位姿
        new_gnss_cnt_++;
    }

    return true;
    
}

//是否优化
bool BackEnd::MaybeOptimized(){
    // static int unoptimized_cnt = 0;
    // //没关键帧，没回环，100次优化一次
    // if(++unoptimized_cnt > optimize_step_with_none_){
    //     unoptimized_cnt = 0;
    //     has_new_optimized_ = true;
    // }

    bool need_optimize = false;
    if(new_gnss_cnt_ >= graph_optimizer_config_.optimize_step_with_gnss)
        need_optimize = true;
    if(new_loop_cnt_ >= graph_optimizer_config_.optimize_step_with_loop)
        need_optimize = true;
    if(new_key_frame_cnt_ >= graph_optimizer_config_.optimize_step_with_key_frame)
        need_optimize = true;

    if(!need_optimize)
        return false;//如果不需要优化直接返回false

    //优化一次后，重新计数
    new_gnss_cnt_ = 0;
    new_key_frame_cnt_ = 0;
    new_loop_cnt_ = 0;
    
    if(graph_optimizer_ptr_->Optimize())
        has_new_optimized_ = true;

    return true;
}

bool BackEnd::ForceOptimize(){
    if(graph_optimizer_ptr_->Optimize())
        has_new_optimized_ = true;
    return has_new_optimized_;
}

void BackEnd::GetOptimizedKeyFrames(std::deque<KeyFrame>& key_frames_deque){

    // key_frames_deque = key_frames_deque_;
    key_frames_deque.clear();
    if(graph_optimizer_ptr_->GetNodeNum() > 0){
        std::deque<Eigen::Matrix4f> optimized_pose;
        graph_optimizer_ptr_->GetOptimizedPose(optimized_pose);
        KeyFrame key_frame;
        for(size_t i=0; i<optimized_pose.size(); i++){
            key_frame.pose = optimized_pose.at(i);
            key_frame.index = (unsigned int)i;
            key_frames_deque.push_back(key_frame);
        }
    }
}

bool BackEnd::HasNewKeyFrame(){
    return has_new_key_frame_;
}


bool BackEnd::HasNewOptimized(){

    return has_new_optimized_;
}

void BackEnd::GetLatestKeyFrame(KeyFrame& key_frame){

    // key_frame = latest_key_frame_;
    key_frame = current_key_frame_;
}

}
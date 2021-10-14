/***
 * Description:闭环检测算法实现
 * Data:1011
 * ***/

#include "mapping/loop_closing/loop_closing.hpp"

#include <cmath>
#include <algorithm>
#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
#include "glog/logging.h"

#include "general_models/registration/ndt_registration.hpp"
#include "general_models/cloud_filter/voxel_filter.hpp"
#include "general_models/tools/print_info.hpp"
#include "general_models/cloud_filter/no_filter.hpp"

#include "general_models/tools/global_path.h"

namespace lidar_project{

//构造函数初始化参数
LoopClosing::LoopClosing(){
    InitWithConfig();
}


//使用yaml初始化各类参数
bool LoopClosing::InitWithConfig(){
    std::string config_file_path = WORK_SPACE_PATH + "/config/LoopClosingConfig.yaml";
    std::cout<<"回环读取的路径:"<<config_file_path<<std::endl;
    YAML::Node config_node = YAML::LoadFile(config_file_path);

    std::cout<<">>>>>>>>>>>闭环检测初始化<<<<<<<<<<<<<"<<std::endl;
    InitParam(config_node);
    InitDataPath(config_node);
    InitRegistration(registration_ptr_,config_node);
    InitFilter("map",map_filter_ptr_,config_node);
    InitFilter("scan",scan_filter_ptr_,config_node);

    return true;
}

//初始化策略相关参数
bool LoopClosing::InitParam(const YAML::Node& config_node){
    extend_frame_num_ = config_node["extend_frame_num"].as<int>();
    loop_step_ = config_node["loop_step"].as<int>();
    diff_num_ = config_node["diff_num"].as<int>();
    detect_area_ = config_node["detect_area"].as<float>();
    fitness_score_limit_ = config_node["fitness_score_limit"].as<float>();

    return true;
}


//初始化路径
bool LoopClosing::InitDataPath(const YAML::Node& config_node){
    std::string data_path = config_node["data_path"].as<std::string>();

    if(data_path == "./"){
        data_path = WORK_SPACE_PATH;
    }

    key_frames_path_ = data_path + "/slam_data/key_frames";

    return true;
}


//初始化配准方法
bool LoopClosing::InitRegistration(std::shared_ptr<RegistrationInterface>& registration_ptr, const YAML::Node& config_node){
    std::string registration_method = config_node["registration_method"].as<std::string>();
    std::cout<<"闭环点云匹配方式为 "<<registration_method<<std::endl;

    if(registration_method == "NDT"){
        registration_ptr = std::make_shared<NDTRegistration>(config_node[registration_method]);
    }else{
        LOG(ERROR)<<"未找到匹配的滤波方法"<< registration_method<<std::endl;
        return false;
    }

    return true;
}


//初始化滤波器
bool LoopClosing::InitFilter(std::string filter_user, std::shared_ptr<CloudFilterInterface>& filter_ptr, const YAML::Node& config_node){
    std::string filter_method = config_node[filter_user + "_filter"].as<std::string>();
    std::cout<<"回环检测的 "<<filter_user<<" 选择的滤波方法为: "<<filter_method<<std::endl;

    if(filter_method == "voxel_filter"){
        filter_ptr = std::make_shared<VoxelFilter>(config_node[filter_method][filter_user]);
    }else if(filter_method == "no_filter"){
        filter_ptr = std::make_shared<NoFilter>();
    }else{
        LOG(ERROR) << "没有为 "<<filter_user<<" 找到与"<<filter_method<<" 相对应的滤波方法";
        return false;
    }

    return true;
}


/***
 * Input:关键帧，关键帧对应gnss位姿
 * Ouput:判断是否是回环
 * 检测两部分，1--检测  2--配准
 * ***/
bool LoopClosing::Update(const KeyFrame key_frame, const KeyFrame key_gnss){
    has_new_loop_pose_ = false;

    all_key_frames_.push_back(key_frame);
    all_key_gnss_.push_back(key_gnss);

    int key_frame_index = 0;
    if(!DetectNearestKeyFrame(key_frame_index)){
        // std::cout<<"未找到符合规则的回环"<<std::endl;
        return false;
    }else{
        std::cout<<"找到回环啦，我要更新位姿去了"<<std::endl;
    }
        
    if(!CloudRegistration(key_frame_index))
        return false;
    
    has_new_loop_pose_ = true;
    return true;
}



/* 寻找回环
 * OUTPUT:找到的回环历史帧的索引
 * ***/
bool LoopClosing::DetectNearestKeyFrame(int& key_frame_index){
    
    static int skip_cnt = 0;
    static int skip_num = loop_step_;

    if(++skip_cnt < skip_num){
        return false;//不到十个关键帧不进行检测，防止太频繁
    }
    if((int)all_key_gnss_.size() < diff_num_ + 1)
        return false;//关键帧数目太少时，不进行检测

    std::cout<<">>>>>>>>>>>>>>>寻找回环<<<<<<<<<<<<<<<<<"<<std::endl;
    int key_num = (int)all_key_gnss_.size();
    float min_distance = 1000000.0;
    float distance = 0.0;//判断计算

    KeyFrame history_key_frame;
    KeyFrame current_key_frame = all_key_gnss_.back();//从最近的关键帧拿出来去和历史匹配

    key_frame_index = -1;
    for(int i=0; i<key_num-1; i++){
        if(key_num - i < diff_num_)
            break;//对100帧之前的历史帧进行检测

        history_key_frame = all_key_gnss_.at(i);
        distance = fabs(current_key_frame.pose(0,3) - history_key_frame.pose(0,3)) + 
                   fabs(current_key_frame.pose(1,3) - history_key_frame.pose(1,3)) + 
                   fabs(current_key_frame.pose(2,3) - history_key_frame.pose(2,3));
        
        //寻找距离最小的作为回环待选
        if(distance < min_distance){
            min_distance = distance;
            key_frame_index = i;
        }
    }

    //小于3个关键帧不好拼个小地图，所以前面的索引就算了
    if(key_frame_index < extend_frame_num_)
        return false;
    
    skip_cnt =0;
    skip_num = (int)min_distance;//啥玩意，其实后面都改了，无所谓
    if(min_distance > detect_area_){//如果检测到的最小距离小于设定的值，不行，
        skip_num = std::max((int)(min_distance/2.0), loop_step_);//这算是个工程值么
        return false;
    }else{
        skip_num = loop_step_;
        return true;
    }
    
}



//当前帧点云与匹配到的历史帧进行配准（实际上是与历史帧周围一起的局部地图）
bool LoopClosing::CloudRegistration(int key_frame_index){
    //局部关键帧组成的地图，提高配准准确率
    CloudData::CloudPtr map_cloud_ptr(new CloudData::CloudPointT());
    Eigen::Matrix4f map_pose = Eigen::Matrix4f::Identity();
    JointMap(key_frame_index,map_cloud_ptr,map_pose);

    //生成当前scan
    CloudData::CloudPtr scan_cloud_ptr(new CloudData::CloudPointT());
    Eigen::Matrix4f scan_pose = Eigen::Matrix4f::Identity();
    JointScan(scan_cloud_ptr,scan_pose);

    //进行匹配
    Eigen::Matrix4f result_pose = Eigen::Matrix4f::Identity();
    Registration(map_cloud_ptr,scan_cloud_ptr,scan_pose,result_pose);
    std::cout<<"回环小地图与当前帧地图匹配完毕"<<std::endl;

    //计算相对位姿
    current_loop_pose_.pose = map_pose.inverse() *  result_pose;

}



/***
 * 构建匹配到的历史帧局部地图
 * INPUT:历史帧索引--key_frame_index
 *       历史帧位姿--map_pose
 * OUTPUT:map_cloud_ptr --以历史帧为中心融合而得的局部点云地图，用于做当前帧与历史帧的点云配准
 * ***/
bool LoopClosing::JointMap(int key_frame_index,                 //匹配到的历史帧索引
                           CloudData::CloudPtr& map_cloud_ptr,  //融合后的局部点云
                           Eigen::Matrix4f& map_pose){          //历史帧位姿
    
    map_pose = all_key_gnss_.at(key_frame_index).pose;
    current_loop_pose_.index0 = all_key_frames_.at(key_frame_index).index;

    //融合地图,这行类似于把关键帧位姿转换到gnss的坐标系中,得到这个转换
    Eigen::Matrix4f pose_to_gnss = map_pose * all_key_frames_.at(key_frame_index).pose.inverse();

    for(int i = key_frame_index - extend_frame_num_; i<key_frame_index + extend_frame_num_; i++){
        std::string file_path = key_frames_path_ + "/key_frame_"+std::to_string(all_key_frames_.at(i).index)+".pcd";

        CloudData::CloudPtr cloud_ptr(new CloudData::CloudPointT());
        pcl::io::loadPCDFile(file_path,*cloud_ptr);//载入数据

        Eigen::Matrix4f cloud_pose = pose_to_gnss * all_key_frames_.at(i).pose;//转换到匹配到的历史帧上
        pcl::transformPointCloud(*cloud_ptr,*cloud_ptr,cloud_pose);

        *map_cloud_ptr += *cloud_ptr;//点云融合 
    }

    map_filter_ptr_->Filter(map_cloud_ptr,map_cloud_ptr);
    return true;
}

/***
 * 当前帧获取
 * OUTPUT:当前帧点云--scan_cloud_ptr
 *        当前帧位姿--scan_pose
 * ***/
bool LoopClosing::JointScan(CloudData::CloudPtr& scan_cloud_ptr, Eigen::Matrix4f& scan_pose){
    scan_pose = all_key_gnss_.back().pose;
    current_loop_pose_.index1 = all_key_frames_.back().index;//当前帧
    current_loop_pose_.time = all_key_frames_.back().time;

    std::string file_path = key_frames_path_+"/key_frame_"+std::to_string(all_key_frames_.back().index)+".pcd";
    pcl::io::loadPCDFile(file_path,*scan_cloud_ptr);
    scan_filter_ptr_->Filter(scan_cloud_ptr,scan_cloud_ptr);

    return true;
}


/***
* Description:点云配准
* INPUT: map_cloud_ptr--历史帧局部点云
         scan_cloud_ptr -- 当前帧点云
         scan_pose -- 当前帧位姿
* OUTPUT:转换位姿
***/
bool LoopClosing::Registration(CloudData::CloudPtr& map_cloud_ptr,
                               CloudData::CloudPtr& scan_cloud_ptr,
                               Eigen::Matrix4f& scan_pose,
                               Eigen::Matrix4f& result_pose){
    CloudData::CloudPtr result_cloud_ptr(new CloudData::CloudPointT());
    registration_ptr_->SetInputTarget(map_cloud_ptr);//目标点云，不变的
    registration_ptr_->ScanMatch(scan_cloud_ptr,scan_pose,result_cloud_ptr,result_pose);

    return true;
}


bool LoopClosing::HasNewLoopPose(){
    return has_new_loop_pose_;
}

LoopPose& LoopClosing::GetCurrentLoopPose(){
    return current_loop_pose_;
}



}
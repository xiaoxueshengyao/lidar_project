/***
 * Description:点云各个函数实现
 * Data: 0611
 * ***/


#include "mapping/viewer/viewer.hpp"

#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
#include <glog/logging.h>

#include "general_models/file_manager/file_manager.hpp"


namespace lidar_project
{

//构造函数
Viewer::Viewer(){
    InitWithConfig();
}

bool Viewer::InitWithConfig(){
    std::string config_file_path = "/home/jingwan/lslidar_ws/src/lidar_project/config/viewer_config.yaml";
    YAML::Node config_node = YAML::LoadFile(config_file_path);

    InitParam(config_node);
    InitDataPath(config_node);
    InitFilter("frame",frame_filter_ptr_,config_node);
    InitFilter("local_map",local_map_filter_ptr_,config_node);
    InitFilter("global_map",global_map_filter_ptr_,config_node);;

    return true;
}


bool Viewer::InitParam(const YAML::Node& config_node){
    local_frame_num_ = config_node["local_frame_num"].as<int>();

    return true;
}

//初始化点云、关键帧保存路径
bool Viewer::InitDataPath(const YAML::Node& config_node){
    std::string data_path = config_node["data_path"].as<std::string>();
    if(data_path == "./"){
        data_path = "/home/jingwan/lslidar_ws/src/lidar_project";
    }
    key_frames_path_ = data_path + "/slam_data/key_frames";
    map_path_ = data_path + "/slam_data/map";

    if(!FileManager::InitDirectory(map_path_,"PointCloud Map"))
        return false;

    return true;
}

//初始化滤波器参数
bool Viewer::InitFilter(std::string filter_user,
                        std::shared_ptr<CloudFilterInterface>& filter_ptr,
                        const YAML::Node& config_node){
    std::string filter_method = config_node[filter_user+"filter"].as<std::string>();
    LOG(INFO) <<"Viewer_"+filter_user<<"The chosen filter method is: "<<filter_method;

    if(filter_method == "voxel_filter"){
        filter_ptr = std::make_shared<VoxelFilter>(config_node[filter_method][filter_user]);
    }else
    {
        LOG(ERROR) <<"No matching method for "<<filter_user;
        return false;
    }

    return true;
    
}


bool Viewer::Update(std::deque<KeyFrame>& new_key_frames,
                    std::deque<KeyFrame>& optimized_key_frames,
                    PoseData transformed_data,
                    CloudData cloud_data){
    ResetParam();

    if(optimized_key_frames.size() > 0){
        optimized_key_frames_ = optimized_key_frames;
        optimized_key_frames.clear();
        OptimizeKeyFrames();
        has_new_local_map_ = true;
    }

    //添加新关键帧
    if(new_key_frames.size()){
        all_key_frames_.insert(all_key_frames_.end(),new_key_frames.begin(),new_key_frames.end());
        new_key_frames.clear();
        has_new_local_map_ = true;
    }

    optimized_odom_ = transformed_data;
    optimized_odom_.pose = pose_to_optimize_ * optimized_odom_.pose;//???

    optimized_cloud_ = cloud_data;
    pcl::transformPointCloud(*cloud_data.cloud_ptr,*optimized_cloud_.cloud_ptr,optimized_odom_.pose);

    return true;

}



void Viewer::ResetParam(){
    has_new_global_map_ = false;
    has_new_local_map_ = false;
}

//关键帧位姿更新
bool Viewer::OptimizeKeyFrames(){
    size_t optimized_index = 0;
    size_t all_index = 0;
    while (optimized_index < optimized_key_frames_.size() && all_index < all_key_frames_.size())
    {
        //这里的两个条件都是小于，有问题,把后面的条件改为大于感觉比较合理
        if(optimized_key_frames_.at(optimized_index).index < all_key_frames_.at(all_index).index){
            optimized_index++;
        }else if(optimized_key_frames_.at(optimized_index).index > all_key_frames_.at(all_index).index){
            all_index++;
        }else{//如果优化位姿的index和全局中对应的位姿index对应上了
            //相当于求了同一个位置，优化前后的位姿差
            pose_to_optimize_ = optimized_key_frames_.at(optimized_index).pose * all_key_frames_.at(all_index).pose.inverse());
            all_key_frames_.at(all_index) = optimized_key_frames_.at(optimized_index);
            optimized_index++;
            all_index++;
        }
    }
    
    while (all_index < all_key_frames_.size())
    {
        //相当于更新全局位姿，把优化后的位姿给到全局，为什么不直接给呢，而且pose2op是一直被覆盖的
        all_key_frames_.at(all_index).pose = pose_to_optimize_ * all_key_frames_.at(all_index).pose;
        all_index++;
    }
    return true;
    

}



bool Viewer::JointGlobalMap(CloudData::CloudPtr& global_map_ptr){
    JointCloudMap(optimized_key_frames_,global_map_ptr);
    return true;
}

bool Viewer::JointLocalMap(CloudData::CloudPtr& local_map_ptr){
    size_t begin_index = 0;
    if(all_key_frames_.size() > (size_t)local_frame_num_){
        begin_index = all_key_frames_.size() - (size_t)local_frame_num_;
    }//局部地图只显示20帧关键帧

    std::deque<KeyFrame> local_key_frames;
    for(size_t i=begin_index; i<all_key_frames_.size();i++){
        local_key_frames.push_back(all_key_frames_.at(i));
    }

    JointCloudMap(local_key_frames,local_map_ptr);

    return true;
}

//点云地图读入+拼接
bool Viewer::JointCloudMap(const std::deque<KeyFrame>& key_frames,CloudData::CloudPtr& map_cloud_ptr){

    map_cloud_ptr.reset(new CloudData::CloudPointT);

    CloudData::CloudPtr cloud_ptr(new CloudData::CloudPointT);
    std::string file_path = "";
    for(size_t i=0; i<key_frames.size();i++){
        file_path = key_frames_path_+"/key_frame_"+std::to_string(key_frames.at(i).index) + ".pcd";
        pcl::io::loadPCDFile(file_path,*cloud_ptr);
        pcl::transformPointCloud(*cloud_ptr,*cloud_ptr,key_frames.at(i).pose);
        *map_cloud_ptr += *cloud_ptr;
    }

    return true;


}

bool Viewer::SaveMap(){
    if(optimized_key_frames_.size() == 0){
        return false;
    }

    CloudData::CloudPtr global_map_ptr(new CloudData::CloudPointT);
    JointCloudMap(optimized_key_frames_,global_map_ptr);
    std::string map_file_path = map_path_ + "/map.pcd";
    pcl::io::savePCDFileBinary(map_file_path,*global_map_ptr);

    LOG(INFO)<<"Map Saved in "<<std::endl<<map_file_path <<std::endl<<std::endl;
    return true;
}

Eigen::Matrix4f& Viewer::GetCurrentPose(){
    return optimized_odom_.pose;
}

CloudData::CloudPtr& Viewer::GetCurrentScan(){
    frame_filter_ptr_->Filter(optimized_cloud_.cloud_ptr,optimized_cloud_.cloud_ptr);
    return optimized_cloud_.cloud_ptr;
}

bool Viewer::GetLocalMap(CloudData::CloudPtr& local_map_ptr){
    JointLocalMap(local_map_ptr);
    local_map_filter_ptr_->Filter(local_map_ptr,local_map_ptr);
    return true;
}

bool Viewer::GetGlobalMap(CloudData::CloudPtr& global_map_ptr){
    JointGlobalMap(global_map_ptr);
    global_map_filter_ptr_->Filter(global_map_ptr,global_map_ptr);
    return true;
}

bool Viewer::HasNewGlobalMap(){
    return has_new_global_map_;
}

bool Viewer::HasNewLocalMap(){
    return has_new_local_map_;
}



} // namespace lidar_project

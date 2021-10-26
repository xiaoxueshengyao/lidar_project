/*****
 * Description: 定位的核心函数实现
 * Data: 1019
 * ****/
#include "matching/matching.hpp"
#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
#include "glog/logging.h"

#include "general_models/tools/global_path.h"
#include "general_models/registration/ndt_registration.hpp"
#include "general_models/cloud_filter/voxel_filter.hpp"
#include "general_models/cloud_filter/no_filter.hpp"

namespace lidar_project{
Matching::Matching()
    :local_map_ptr_(new CloudData::CloudPointT()),
    global_map_ptr_(new CloudData::CloudPointT()),
    current_scan_ptr_(new CloudData::CloudPointT()){
        InitWithConfig();
        InitGlobalMap();
        ResetLocalMap(0.0, 0.0, 0.0);
}

//初始化各种参数，从yaml文件中读取
bool Matching::InitWithConfig(){
    std::string config_file_path = WORK_SPACE_PATH + "/config/Matching.yaml";
    YAML::Node config_node = YAML::LoadFile(config_file_path);

    std::cout<<">>>>>>>>>>>>>>>>>>地图初始化<<<<<<<<<<<<<<<<<"<<std::endl<<std::endl;
    InitDataPath(config_node);
    InitRegistration(registration_ptr_,config_node);
    InitFilter("global_map",global_map_filter_ptr_,config_node);
    InitFilter("local_map",local_map_filter_ptr_,config_node);
    InitFilter("frame",frame_filter_ptr_,config_node);
    InitBoxFilter(config_node);

    return true;
}

//初始化地图保存路径，后续读取地图
bool Matching::InitDataPath(const YAML::Node& config_node){
    map_path_ = config_node["map_path"].as<std::string>();
    return true;
}

//选择点云配准方法
bool Matching::InitRegistration(std::shared_ptr<RegistrationInterface>& registration_ptr, const YAML::Node& config_node){
    std::string registration_method = config_node["registration_method"].as<std::string>();
    std::cout<<"地图匹配选择的点云匹配方式:"<<registration_method<<std::endl;

    if(registration_method == "NDT"){
        registration_ptr = std::make_shared<NDTRegistration>(config_node[registration_method]);//多了下划线，函数给的变量直接变成了类的成员变量，出现bug
    }else{
        LOG(ERROR)<<"未找到与 "<<registration_method<<" 相匹配的配准方法";
        return false;
    }

    return true;
}

/****
* 初始化滤波器类型
****/
bool Matching::InitFilter(std::string filter_user, std::shared_ptr<CloudFilterInterface>& filter_ptr,const YAML::Node& config_node){
    std::string filter_method = config_node[filter_user+"_filter"].as<std::string>();
    std::cout<<"地图匹配"<<filter_user<<"选择的滤波的方法为: "<<filter_method <<std::endl;

    if(filter_method == "voxel_filter"){
        filter_ptr = std::make_shared<VoxelFilter>(config_node[filter_method][filter_user]);
    }else if(filter_method == "no_filter"){
        filter_ptr = std::make_shared<NoFilter>();
    }else{
        LOG(ERROR)<<"No Matching Filter method";
        return false;
    }

    return true;
}



bool Matching::InitBoxFilter(const YAML::Node& config_node){
    box_filter_ptr_ = std::make_shared<BoxFilter>(config_node);
    return true;
}


bool Matching::InitGlobalMap(){
    pcl::io::loadPCDFile(map_path_,*global_map_ptr_);
    LOG(INFO) << "load global map size: "<<global_map_ptr_->points.size();

    local_map_filter_ptr_->Filter(global_map_ptr_,global_map_ptr_);
    LOG(INFO) << "filtered global map size: "<<global_map_ptr_->points.size();

    has_new_global_map_ = true;
    return true;
}

bool Matching::ResetLocalMap(float x, float y, float z){
    std::vector<float> origin = {x, y, z};
    box_filter_ptr_->SetOrigin(origin);
    box_filter_ptr_->Filter(global_map_ptr_, local_map_ptr_);//取全局地图的一部分

    registration_ptr_->SetInputTarget(local_map_ptr_);

    has_new_local_map_ = true;
    std::vector<float> edge = box_filter_ptr_->GetEdge();
    LOG(INFO) << "new local map: "<< edge.at(0)<<","<<edge.at(1)<<","<<edge.at(2)<<","
                                  << edge.at(3)<<","<<edge.at(4)<<","<<edge.at(5)<<std::endl<<std::endl;
    return true;
}


/**更新定位位姿数据*/
bool Matching::Update(const CloudData& cloud_data, Eigen::Matrix4f& cloud_pose){
    std::vector<int> indices;
    pcl::removeNaNFromPointCloud(*cloud_data.cloud_ptr,*cloud_data.cloud_ptr,indices);//去除NAN的点

    CloudData::CloudPtr filtered_cloud_ptr(new CloudData::CloudPointT());
    frame_filter_ptr_->Filter(cloud_data.cloud_ptr,filtered_cloud_ptr);//滤波

    static Eigen::Matrix4f step_pose = Eigen::Matrix4f::Identity();
    static Eigen::Matrix4f last_pose = init_pose_;//修饰局部变量时，表明该变量的值不会因为函数终止而丢失
    static Eigen::Matrix4f predict_pose = init_pose_;

    if(!has_inited_){
        predict_pose = current_gnss_pose_;//第一帧使用gnss的数据初始化
    }

    //与地图匹配
    CloudData::CloudPtr result_cloud_ptr(new CloudData::CloudPointT());
    registration_ptr_->ScanMatch(filtered_cloud_ptr,predict_pose,result_cloud_ptr,cloud_pose);
    pcl::transformPointCloud(*cloud_data.cloud_ptr,*current_scan_ptr_,cloud_pose);//为current_scan_ptr_(当前点云)赋值

    //更新相邻两帧的相对运动
    step_pose = last_pose.inverse() * cloud_pose;//相邻帧相对位姿
    predict_pose = cloud_pose * step_pose;//位姿预测，和刚开始做这个的时候类似，匀速运动
    last_pose = cloud_pose;

    //匹配之后判断是否需要更新局部地图
    //当前位置和立方体的边缘距离小于50m时，则以当前位置为原点，重新分割一个局部地图
    std::vector<float> edge = box_filter_ptr_->GetEdge();
    for(int i=0; i<3; i++){
        if(fabs(cloud_pose(i,3) - edge.at(2*i)) > 50.0 && 
           fabs(cloud_pose(i,3) - edge.at(2*i + 1))>50.0)
           continue;
        ResetLocalMap(cloud_pose(0,3),cloud_pose(1,3),cloud_pose(2,3));
        break;
    }

    return true;
}


bool Matching::SetInitPose(const Eigen::Matrix4f& init_pose){
    init_pose_ = init_pose;
    ResetLocalMap(init_pose(0,3), init_pose(1,3), init_pose(2,3));

    return true;
}

//如果没有初始化，就用gnss的位姿进行初始化
bool Matching::SetGNSSPose(const Eigen::Matrix4f& gnss_pose){
    current_gnss_pose_ = gnss_pose;

    static int gnss_cnt=0;
    if(gnss_cnt==0){
        SetInitPose(gnss_pose);
    }else if(gnss_cnt > 3){
        has_inited_ =true;
    }
    gnss_cnt++;

    return true;
}

//输出滤波后的全局地图global_map
void Matching::GetGlobalMap(CloudData::CloudPtr& global_map){
    global_map_filter_ptr_->Filter(global_map_ptr_,global_map);
    has_new_global_map_ = false;
}

CloudData::CloudPtr& Matching::GetLoaclMap(){
    return local_map_ptr_;
}

CloudData::CloudPtr& Matching::GetCurrentScan(){
    return current_scan_ptr_;
}

bool Matching::HasInited(){
    return has_inited_;
}

bool Matching::HasNewGlobalMap(){
    return has_new_global_map_;
}

bool Matching::HasNewLocalMap(){
    return has_new_local_map_;
}

}
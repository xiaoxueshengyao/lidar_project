/****
 * Description: The realization of front end(copy from renqian)
 * 第一个版本，很多功能是混在一起的
 * Author: Capta1nY
 * Data: 0408
 * 0415 降耦合，把滤波和配准分开，方便后续改接口
 * ***/
#include "mapping/front_end/front_end.hpp"
#include <pcl/common/transforms.h>
#include <cmath>

#include <fstream>
#include <boost/filesystem.hpp>
#include <pcl/io/pcd_io.h>
#include <glog/logging.h>


namespace lidar_project{

FrontEnd::FrontEnd()
    :local_map_ptr_(new CloudData::CloudPointT)
    // global_map_ptr_(new CloudData::CloudPointT),
    // result_cloud_ptr_(new CloudData::CloudPointT)//当前帧转换后
{
    InitWithConfig();
}

bool FrontEnd::InitWithConfig(){
    std::string config_file_path = "/home/jingwan/lslidar_ws/src/lidar_project/config/FrontEndConfig.yaml";
    YAML::Node config_node = YAML::LoadFile(config_file_path);

    InitParam(config_node);
    InitRegistration(registration_ptr_,config_node);
    InitFilter("local_map",local_map_filter_ptr_,config_node);
    InitFilter("frame",frame_filter_ptr_,config_node);
    // InitFilter("display",display_map_filter_ptr_,config_node);

    return true;
}

//关键帧提取阈值，局部地图关键帧数量
bool FrontEnd::InitParam(const YAML::Node& config_node){
    key_frame_dis_ = config_node["key_frame_distance"].as<float>();
    local_frame_num_ = config_node["local_frame_num"].as<int>();
    return true;
}

//把地图关珍珍存放在硬盘，放在内存里不利于大范围建图
/***
bool FrontEnd::InitDataPath(const YAML::Node& config_node){
    data_path_ = config_node["data_path"].as<std::string>();
    if(data_path_ == "./"){
        data_path_  = "/home/jingwan/lslidar_ws/src/lidar_project";
    }
    data_path_ += "/slam_data";

    if(boost::filesystem::is_directory(data_path_)){
        boost::filesystem::remove_all(data_path_);//是目录就删除
    }

    boost::filesystem::create_directory(data_path_);
    if(!boost::filesystem::is_directory(data_path_)){
        LOG(WARNING)<<"File "<<data_path_<<" created failed";
        return false;
    }else{
        LOG(INFO)<<"Point cloud map will saved in "<<data_path_;
    }

    std::string key_frame_path = data_path_+"/key_frame";
    boost::filesystem::create_directory(key_frame_path);
    if(!boost::filesystem::is_directory(key_frame_path)){
        LOG(WARNING)<<"File "<<key_frame_path<<" created failed";
        return false;
    }else{
        LOG(INFO)<<"Key Frame point cloud map will saved in "<<key_frame_path;
    }

    return true;

}
***/

bool FrontEnd::InitRegistration(std::shared_ptr<RegistrationInterface>& registration_ptr,
                                const YAML::Node& config_node){
    std::string registratioin_method = config_node["registration_method"].as<std::string>();
    LOG(INFO)<<"The registration method is "<<registratioin_method;

    if(registratioin_method == "NDT"){
        //创建ndt配准指针，参数为yaml——node,读取对应的参数
        registration_ptr = std::make_shared<NDTRegistration>(config_node[registratioin_method]);
    }else{
        LOG(ERROR)<< "No matched registration for "<<registratioin_method;
        return false;
    }
    return true;
}

//初始化滤波器类型
bool FrontEnd::InitFilter(std::string filter_user, std::shared_ptr<CloudFilterInterface>& filter_ptr,
                          const YAML::Node& config_node){   
    std::string filter_method = config_node[filter_user+"_filter"].as<std::string>();
    LOG(INFO) << "front_" + filter_user <<"The choosen filter method is "<<filter_method;
    if(filter_method == "voxel_filter"){
        filter_ptr = std::make_shared<VoxelFilter>(config_node[filter_method][filter_user]);
        
    }else{
        LOG(ERROR)<<"No matched filter for "<<filter_method;
        return false;
    }
    
    return true;
    
}




//新的帧处理程序
bool FrontEnd::Update(const CloudData& cloud_data, Eigen::Matrix4f& cloud_pose){
    current_frame_.cloud_data.time = cloud_data.time;
    std::vector<int> indices;//去除NAN点，否则会编译错误
    pcl::removeNaNFromPointCloud(*cloud_data.cloud_ptr,*current_frame_.cloud_data.cloud_ptr,indices);

    //点云滤波
    CloudData::CloudPtr filtered_cloud(new CloudData::CloudPointT);
    frame_filter_ptr_->Filter(current_frame_.cloud_data.cloud_ptr,filtered_cloud);//输入，输出

    //初始化预测用矩阵
    static Eigen::Matrix4f step_pose = Eigen::Matrix4f::Identity();//relative pose from t-2 to t-1 for prediction 
    static Eigen::Matrix4f last_pose = init_pose_;
    static Eigen::Matrix4f predict_pose = init_pose_;
    static Eigen::Matrix4f last_key_frame_pose = init_pose_;

    //配准处理
    //开始时容器没有关键帧，代表是第一帧，直接保存,设为关键帧更新在local_map和globa_map中
    if(local_map_frames_.size() == 0){
        current_frame_.pose = init_pose_;
        UpdateWithNewFrame(current_frame_);//设置关键帧 + local_map为target_cloud
        cloud_pose = current_frame_.pose; 
        return true;       
    }

    //不是第一帧就进行配准，与局部地图进行配准
    CloudData::CloudPtr result_cloud_ptr(new CloudData::CloudPointT());
    registration_ptr_->ScanMatch(filtered_cloud,predict_pose,result_cloud_ptr,current_frame_.pose);
    cloud_pose = current_frame_.pose;
    

    //更新相邻两帧的相对云从
    //用k-2到k-1帧的相对位姿作为k-1到k帧的预测位姿，这个可以配准用，align时作为初始预测位姿
    step_pose = last_pose.inverse() * current_frame_.pose;
    predict_pose = current_frame_.pose * step_pose;
    last_pose = current_frame_.pose;

    //关键帧判断，任然使用原文中的曼哈顿距离，后续根据情况设定
    if(fabs(last_key_frame_pose(0,3) - current_frame_.pose(0,3)) + 
       fabs(last_key_frame_pose(1,3) - current_frame_.pose(1,3)) + 
       fabs(last_key_frame_pose(2,3) - current_frame_.pose(2,3)) > key_frame_dis_){
           UpdateWithNewFrame(current_frame_);
           last_key_frame_pose = current_frame_.pose;
       }

    return true;

}

bool FrontEnd::SetInitPose(const Eigen::Matrix4f& init_pose){
    init_pose_ = init_pose;
    return true;
}

//关键帧处理，
//包括局部地图更新、配准的target更新（就是local)、全局地图的更新
bool FrontEnd::UpdateWithNewFrame(const Frame& new_key_frame){
    //把关键帧点云存储到硬盘里，节省内存
    // std::string file_path = data_path_+"/key_frame/key_frame_"+std::to_string(global_map_frames_.size())+".pcd";
    // pcl::io::savePCDFileBinary(file_path,*new_key_frame.cloud_data.cloud_ptr);
    // std::cout<<"Got the "<<global_map_frames_.size()<<" key frame"<<std::endl;

    Frame key_frame = new_key_frame;
    //保存关键帧，使用的是共享指针，这里直接复制的是指针
    key_frame.cloud_data.cloud_ptr.reset(new CloudData::CloudPointT(*new_key_frame.cloud_data.cloud_ptr));
    CloudData::CloudPtr transformed_cloud_ptr(new CloudData::CloudPointT);//不带括号和只带个空括号一样，都是无参构造

    //局部地图更新
    //无论放多少关键帧，这些关键帧点云指针都是指向同一个点云
    local_map_frames_.push_back(key_frame);
    while(local_map_frames_.size() > static_cast<size_t>(local_frame_num_)){
        local_map_frames_.pop_front();
    }
    local_map_ptr_.reset(new CloudData::CloudPointT);
    for(size_t i=0; i<local_map_frames_.size(); i++){
        pcl::transformPointCloud(*local_map_frames_.at(i).cloud_data.cloud_ptr,
                                 *transformed_cloud_ptr,
                                 local_map_frames_.at(i).pose);
        *local_map_ptr_ += *transformed_cloud_ptr;
    }
    // has_new_local_map_ = true;

    //更新ndt匹配的目标点云
    //局部地图关键帧数目较少时，直接用作配准，较大时，滤波后使用
    if(local_map_frames_.size()<10){
        registration_ptr_->SetInputTarget(local_map_ptr_);
    }else{
        CloudData::CloudPtr filtered_local_map_ptr(new CloudData::CloudPointT);
        local_map_filter_ptr_->Filter(local_map_ptr_,filtered_local_map_ptr);
        registration_ptr_->SetInputTarget(filtered_local_map_ptr);//local较大时滤波后配准
    }

    //全局地图更新
    //存储之前，把关键帧点云释放，已经放在硬盘
    // key_frame.cloud_data.cloud_ptr.reset(new CloudData::CloudPointT);
    // global_map_frames_.push_back(key_frame);
    return true;

}

//保存全局地图
/****
bool FrontEnd::SaveMap(){
    global_map_ptr_.reset(new CloudData::CloudPointT);

    std::string key_frame_path = "";
    CloudData::CloudPtr key_frame_cloud_ptr(new CloudData::CloudPointT);
    CloudData::CloudPtr transformed_cloud_ptr(new CloudData::CloudPointT);

    for(size_t i=0; i<global_map_frames_.size();i++){
        key_frame_path = data_path_ + "/key_frame/key_frame_"+std::to_string(i)+".pcd";
        pcl::io::loadPCDFile(key_frame_path,*key_frame_cloud_ptr);
        pcl::transformPointCloud(*key_frame_cloud_ptr,
                                *transformed_cloud_ptr,
                                global_map_frames_.at(i).pose);
        *global_map_ptr_ += *transformed_cloud_ptr;
    }


    std::string map_file_path = data_path_ + "/global_map.pcd";
    pcl::io::savePCDFileBinary(map_file_path,*global_map_ptr_);
    has_new_global_map_ = true;
    std::cout<<"Global Map Saved"<<std::endl;

    return true;
}





//对局部地图点云滤波后返回
bool FrontEnd::GetNewLocalMap(CloudData::CloudPtr& local_map_ptr){
    if(has_new_local_map_){
        display_map_filter_ptr_->Filter(local_map_ptr_,local_map_ptr);
        return true;
    }
    return false;
}


bool FrontEnd::GetNewGlobalMap(CloudData::CloudPtr& global_map_ptr){
    if(has_new_global_map_){
        has_new_global_map_ = false;//只在需要的时候显示，每次显示后都false
        display_map_filter_ptr_->Filter(global_map_ptr_,global_map_ptr);
        global_map_ptr_.reset(new CloudData::CloudPointT);
        return true;
    }
    return false;
}

bool FrontEnd::GetCurrentScan(CloudData::CloudPtr& current_cloud_ptr){
    display_map_filter_ptr_->Filter(result_cloud_ptr_,current_cloud_ptr);
    return true;
}
***/

}
/****
 * Description: The realization of front end(copy from renqian)
 * 第一个版本，很多功能是混在一起的
 * Author: Capta1nY
 * Data: 0408
 * ***/
#include "front_end/front_end_back.hpp"
#include <pcl/common/transforms.h>
#include <cmath>

namespace lidar_project{

FrontEnd::FrontEnd()
    :ndt_ptr_(new pcl::NormalDistributionsTransform<CloudData::PointT,CloudData::PointT>),
     local_map_ptr_(new CloudData::CloudPointT),
     global_map_ptr_(new CloudData::CloudPointT),
     result_cloud_ptr_(new CloudData::CloudPointT){}


Eigen::Matrix4f FrontEnd::Update(const CloudData& cloud_data){
    current_frame_.cloud_data.time = cloud_data.time;
    std::vector<int> indices;//去除NAN点，否则会编译错误
    pcl::removeNaNFromPointCloud(*cloud_data.cloud_ptr,*current_frame_.cloud_data.cloud_ptr,indices);

    //点云滤波
    CloudData::CloudPtr filtered_cloud(new CloudData::CloudPointT);
    cloud_filter_.setLeafSize(1.2,1.2,1.2);
    cloud_filter_.setInputCloud(current_frame_.cloud_data.cloud_ptr);
    cloud_filter_.filter(*filtered_cloud);

    //初始化预测用矩阵
    static Eigen::Matrix4f step_pose = Eigen::Matrix4f::Identity();//relative pose from t-2 to t-1 for prediction 
    static Eigen::Matrix4f last_pose = init_pose_;
    static Eigen::Matrix4f predict_pose = init_pose_;
    static Eigen::Matrix4f last_key_frame_pose = init_pose_;

    //配准处理
    //容器没有关键帧就是第一帧，直接保存,设为关键帧
    if(local_map_frames_.size() == 0){
        current_frame_.pose = init_pose_;
        UpdateNewFrame(current_frame_);//设置关键帧 + local_map为target_cloud
        return current_frame_.pose;        
    }

    //不是第一帧就进行配准，与局部地图进行配准
    ndt_ptr_->setResolution(1.0);
    ndt_ptr_->setStepSize(0.1);
    ndt_ptr_->setTransformationEpsilon(0.01);
    ndt_ptr_->setMaximumIterations(30);
    ndt_ptr_->setInputSource(filtered_cloud);
    ndt_ptr_->align(*result_cloud_ptr_,predict_pose);//使用运动模型作为位姿预测，而没用gnss+imu的信息
    //改用gnss试试,使用gps+imu作为先验位姿后，发现建图和定位效果可好太多了，这个先验位姿对于ndt，icp这样的配准算法还是挺重要的
    //之前一直用单位阵（默认的），也用过前一帧的相对位姿，没啥效果，不过要是使用gps+imu，直接点云拼接就出地图了，还是要从算法上
    //提高激光里程计的精度，包括使用后端优化
    current_frame_.pose = ndt_ptr_->getFinalTransformation();

    //更新相邻两帧的相对云顿
    //用k-2到k-1帧的相对位姿作为k-1到k帧的预测位姿，这个可以配准用，align时作为初始预测位姿
    step_pose = last_pose.inverse() * current_frame_.pose;
    predict_pose = current_frame_.pose * step_pose;
    last_pose = current_frame_.pose;

    //关键帧判断，任然使用原文中的曼哈顿距离，后续根据情况设定
    if(fabs(last_key_frame_pose(0,3) - current_frame_.pose(0,3)) + 
       fabs(last_key_frame_pose(1,3) - current_frame_.pose(1,3)) + 
       fabs(last_key_frame_pose(2,3) - current_frame_.pose(2,3)) > 2.0){
           UpdateNewFrame(current_frame_);
           last_key_frame_pose = current_frame_.pose;
       }

    return current_frame_.pose;

}

//关键帧处理，
//包括局部地图更新、配准的target更新（就是local)、全局地图的更新
void FrontEnd::UpdateNewFrame(const Frame& new_key_frame){
    Frame key_frame = new_key_frame;
    key_frame.cloud_data.cloud_ptr.reset(new CloudData::CloudPointT(*new_key_frame.cloud_data.cloud_ptr));
    CloudData::CloudPtr transformed_cloud_ptr(new CloudData::CloudPointT);//不带括号和只带个空括号一样，都是无参构造

    //局部地图更新
    //无论放多少关键帧，这些关键帧点云指针都是指向同一个点云
    local_map_frames_.push_back(key_frame);
    while(local_map_frames_.size()>20){
        local_map_frames_.pop_front();
    }
    local_map_ptr_.reset(new CloudData::CloudPointT);
    for(size_t i=0; i<local_map_frames_.size(); i++){
        pcl::transformPointCloud(*local_map_frames_.at(i).cloud_data.cloud_ptr,
                                 *transformed_cloud_ptr,
                                 local_map_frames_.at(i).pose);
        *local_map_ptr_ += *transformed_cloud_ptr;
    }
    has_new_local_map_ = true;

    //更新ndt匹配的目标点云
    //局部地图关键帧数目较少时，直接用作配准，较大时，滤波后使用
    if(local_map_frames_.size()<10){
        ndt_ptr_->setInputTarget(local_map_ptr_);
    }else{
        CloudData::CloudPtr filtered_local_map_ptr(new CloudData::CloudPointT);
        local_map_filter_.setLeafSize(0.6,0.6,0.6);
        local_map_filter_.setInputCloud(local_map_ptr_);
        local_map_filter_.filter(*filtered_local_map_ptr);
        ndt_ptr_->setInputTarget(filtered_local_map_ptr);
    }

    //全局地图更新
    global_map_frames_.push_back(key_frame);
    if(global_map_frames_.size() % 100 !=0){
        return;
    }else{
        global_map_ptr_.reset(new CloudData::CloudPointT);//new后面跟的不是指针
        for(size_t i=0; i<global_map_frames_.size();i++){
            pcl::transformPointCloud(*global_map_frames_.at(i).cloud_data.cloud_ptr,
                                     *transformed_cloud_ptr,
                                     global_map_frames_.at(i).pose);
            *global_map_ptr_ += *transformed_cloud_ptr;
        }
        has_new_global_map_ = true;
    }

}


bool FrontEnd::SetInitPose(const Eigen::Matrix4f& init_pose){
    init_pose_ = init_pose;
    return true;
}

bool FrontEnd::SetPredictPose(const Eigen::Matrix4f& predict_pose){
    predict_pose_ = predict_pose;
    return true;
}

//对局部地图点云滤波后返回
bool FrontEnd::GetNewLocalMap(CloudData::CloudPtr& local_map_ptr){
    if(has_new_local_map_){
        display_filter_.setLeafSize(0.5,0.5,0.5);
        display_filter_.setInputCloud(local_map_ptr_);
        display_filter_.filter(*local_map_ptr);
        return true;
    }
    return false;
}


bool FrontEnd::GetNewGlobalMap(CloudData::CloudPtr& global_map_ptr){
    if(has_new_global_map_){
        display_filter_.setLeafSize(0.5,0.5,0.5);
        display_filter_.setInputCloud(global_map_ptr_);
        display_filter_.filter(*global_map_ptr);
        return true;
    }
    return false;
}

bool FrontEnd::GetCurrentScan(CloudData::CloudPtr& current_cloud_ptr){
    display_filter_.setLeafSize(0.5,0.5,0.5);
    display_filter_.setInputCloud(result_cloud_ptr_);//当前帧转换到global的点云指针
    display_filter_.filter(*current_cloud_ptr);
    return true;
}


}
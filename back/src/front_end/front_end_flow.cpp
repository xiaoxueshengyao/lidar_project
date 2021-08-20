/***
 * Des:Realization of front end flow
 * Author: Capta1nY
 * 0416
 * 0506 时间同步，以雷达数据为核心做差值
 * ***/

#include "front_end/front_end_flow.hpp"
#include "glog/logging.h"
#include "general_models/file_manager/file_manager.hpp"


namespace lidar_project{

//构造函数初始化数据订阅、发布以及前端指针
FrontEndFlow::FrontEndFlow(ros::NodeHandle& nh){
    cloud_sub_ptr_ = std::make_shared<CloudSubscriber>(nh,"/kitti/velo/pointcloud",100000);
    imu_sub_ptr_ = std::make_shared<IMUSubscriber>(nh,"/kitti/oxts/imu",100000);
    gnss_sub_ptr_ = std::make_shared<GNSSSubscriber>(nh,"/kitti/oxts/gps/fix",1000000);
    velocity_sub_ptr_ = std::make_shared<VelocitySubscriber>(nh,"/kitti/oxts/gps/vel",1000000);
    lidar_to_imu_ptr_ = std::make_shared<TFListener>(nh,"velo_link","imu_link");

    cloud_pub_ptr_ = std::make_shared<CloudPublisher>(nh,"current_scan",100,"map");
    local_map_pub_ptr_ = std::make_shared<CloudPublisher>(nh,"local_map",100,"map");
    global_map_pub_ptr_ = std::make_shared<CloudPublisher>(nh,"global_map",100,"map");
    laser_odom_pub_ptr_ = std::make_shared<OdometryPublisher>(nh,"laser_odom","map","lidar",100);
    gnss_pub_ptr_ = std::make_shared<OdometryPublisher>(nh,"gnss_odom","map","lidar",100);

    front_end_ptr_ = std::make_shared<FrontEnd>();

    //畸变矫正
    distortion_adjust_ptr_ = std::make_shared<DistortionAdjust>();

    local_map_ptr_.reset(new CloudData::CloudPointT);//reset(new pcl::PointCloud<pcl::PointXYZ>)
    global_map_ptr_.reset(new CloudData::CloudPointT);//而不是只有xyz
    current_scan_ptr_.reset(new CloudData::CloudPointT);
    std::cout<<" Initilized all data "<<std::endl;


}

bool FrontEndFlow::Run(){

    if(!ReadData())   
      return false;
    if(!InitClibration())
      return false;
    if(!InitGNSS())
      return false;
    
    while (HasData())
    {
        if(!ValidData()){
            continue;
        }
        UpdateGNSSOdometry();
        if(UpdateLaserOdometry()){
            PublishData();
            SaveTrajectory();
        }
    }

    return true;
    
}

//载入订阅的数据
//0506 雷达数据直接读入，其他的需要做差值处理0602
bool FrontEndFlow::ReadData(){
    cloud_sub_ptr_->ParaData(cloud_data_buff_);

    static std::deque<IMUData> unsynced_imu_;   
    static std::deque<GNSSData> unsynced_gnss_;
    static std::deque<VelocityData> unsynced_velocity_;
    
    gnss_sub_ptr_->ParasData(unsynced_gnss_);
    imu_sub_ptr_->ParasData(unsynced_imu_);   
    velocity_sub_ptr_->ParaData(unsynced_velocity_);
    

    if(cloud_data_buff_.size() == 0)  return false;

    double cloud_time = cloud_data_buff_.front().time;
    //std::cout<<"Ready to Go"<<"\t"<<std::fixed<<std::endl;
    bool valid_gnss = GNSSData::SyncData(unsynced_gnss_,gnss_data_buff_,cloud_time); 
    bool valid_imu = IMUData::SyncData(unsynced_imu_,imu_data_buff_,cloud_time);
    bool valid_velocity = VelocityData::SyncData(unsynced_velocity_,velocity_data_buff_,cloud_time);
          
    
    std::cout<<"IMU  GNSS   Velocity "<<"\t"<<valid_imu<<"\t"<<valid_gnss<<"\t"<<valid_velocity<<std::endl;

    static bool sensor_inited = false;
    if(!sensor_inited){
        if(!valid_imu || !valid_velocity || !valid_gnss){
            cloud_data_buff_.pop_front();
            return false;
        }
        sensor_inited = true;
        std::cout<<"Sensor Initialized Succeed"<<std::endl;
    }
    return true;
}

//得到lidar到imu的转换
bool FrontEndFlow::InitClibration(){
    static bool calibration_received = false;
    if(!calibration_received){
        if(lidar_to_imu_ptr_->LookupData(lidar_to_imu_)){
            calibration_received = true;
        }
    }
    std::cout<<"Initialized Clibration"<<std::endl;
    return calibration_received;
}

//初始化gps信息经纬度
bool FrontEndFlow::InitGNSS(){
    static bool gnss_init = false;
    if(!gnss_init && gnss_data_buff_.size()>0){
        GNSSData gnss_data = gnss_data_buff_.front();
        gnss_data.InitOriginPose();//初始化经纬度
        gnss_init = true;
    }
    std::cout<<"GNSS Initialized"<<std::endl;
    return gnss_init;
}

//判断是否读入数据
bool FrontEndFlow::HasData(){
    if(cloud_data_buff_.size() == 0) return false;
    if(imu_data_buff_.size() == 0) return false;
    if(gnss_data_buff_.size() == 0) return false;
    if(velocity_data_buff_.size() == 0) return false;
    std::cout<<"Has data"<<std::endl;
    return true;
}


//判断是否是一帧数据(gps,imu,cloud)
bool FrontEndFlow::ValidData(){
    current_cloud_data_ = cloud_data_buff_.front();
    current_imu_data_ = imu_data_buff_.front();
    current_gnss_data_ = gnss_data_buff_.front();
    current_velocity_data_ = velocity_data_buff_.front();

    double diff_time = current_cloud_data_.time - current_imu_data_.time;
    if(diff_time < -0.05){
        cloud_data_buff_.pop_front();
        return false;
    }else if(diff_time > 0.05){
        imu_data_buff_.pop_front();
        gnss_data_buff_.pop_front();//imu,gps数据早了
        velocity_data_buff_.pop_front();
        return false;
    }
    //否则就正常弹出，处理下一帧
    cloud_data_buff_.pop_front();
    imu_data_buff_.pop_front();
    gnss_data_buff_.pop_front();//都弹出，下次处理下一帧
    velocity_data_buff_.pop_front();
    std::cout<<"Valid Data"<<std::endl;
    return true;

}

//更新gnss里程计信息
bool FrontEndFlow::UpdateGNSSOdometry(){

    gnss_odometry_ = Eigen::Matrix4f::Identity();
    current_gnss_data_.UpdateXYZ();//转换为xyz坐标系
    gnss_odometry_.topRightCorner(3,1) << current_gnss_data_.local_x,
                                           current_gnss_data_.local_y,
                                           current_gnss_data_.local_z;
    gnss_odometry_.block(0,0,3,3) = current_imu_data_.GetRotateMat();
    gnss_odometry_ *= lidar_to_imu_;
    std::cout<<"GNSS Update"<<std::endl;
    return true;

}

//更新激光里程计信息
bool FrontEndFlow::UpdateLaserOdometry(){
    //0608添加畸变矫正
    std::cout<<"矫正开始    "<<std::endl;
    current_velocity_data_.TransformCoordinate(lidar_to_imu_);
    std::cout<<"1"<<std::endl;
    distortion_adjust_ptr_->SetMotionInfo(0.1,current_velocity_data_);//速度信息
    std::cout<<"2"<<std::endl;
    distortion_adjust_ptr_->AdjustCloud(current_cloud_data_.cloud_ptr,current_cloud_data_.cloud_ptr);//转换
    std::cout<<"矫正完成"<<std::endl;

    static bool front_end_pose_inited = false;//使用静态变量，类的
    if(!front_end_pose_inited){
        front_end_pose_inited = true;
        front_end_ptr_->SetInitPose(gnss_odometry_);
        laser_odometry_ = gnss_odometry_;//初始化都设为关键帧
        return true;

    }
    laser_odometry_ = Eigen::Matrix4f::Identity();
    if(front_end_ptr_->Update(current_cloud_data_,laser_odometry_))  return true;
    else return false;
   
}

bool FrontEndFlow::PublishData(){
    gnss_pub_ptr_->Publish(gnss_odometry_);
    laser_odom_pub_ptr_->Publish(laser_odometry_);

    front_end_ptr_->GetCurrentScan(current_scan_ptr_);
    cloud_pub_ptr_->Publish(current_scan_ptr_);

    if(front_end_ptr_->GetNewLocalMap(local_map_ptr_)){
        local_map_pub_ptr_->Publish(local_map_ptr_);
    }
    return true;


}


bool FrontEndFlow::SaveMap(){
    return front_end_ptr_->SaveMap();
}

bool FrontEndFlow::PublisheGlobalMap(){
    if(front_end_ptr_->GetNewGlobalMap(global_map_ptr_)){
        global_map_pub_ptr_->Publish(global_map_ptr_);
        global_map_ptr_.reset(new CloudData::CloudPointT);
    }
    return true;
}


//保存轨迹
bool FrontEndFlow::SaveTrajectory(){
    std::string path = "/home/jingwan/lslidar_ws/src/lidar_project";
    static std::ofstream ground_truth,laser_odom;
    static bool is_file_created = false;
    if(!is_file_created){
        if(!FileManager::CreateDirectory(path + "/slam_data/trajectory"))
            return false;
        if(!FileManager::CreateFile(ground_truth, path + "/slam_data/trajectory/ground_truth.txt"))
            return false;
        if(!FileManager::CreateFile(laser_odom, path + "/slam_data/trajectory/laser_odometry.txt"))
            return false;
        is_file_created = true;
    }

    for(int i=0; i<3; i++){
        for(int j=0; j<4; j++){
            ground_truth << gnss_odometry_(i,j);
            laser_odom << laser_odometry_(i,j);
            if(i==2 && j==3){
                ground_truth << std::endl;
                laser_odom << std::endl;
            }else{
                ground_truth << " ";
                laser_odom << " ";
            }

        }
    }

    return true;

}




}
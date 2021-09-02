/***
 * Description:把整个系统分为五大模块，分别是
 * 数据预处理、前端、后端、回环以及显示
 * 当前是数据预处理模块，主要包括原始数据订阅、时间同步、点云畸变补偿、传感器坐标系统一
 * Data:0609
 * 
 * @Input: GNSS组合导航位置、姿态、线/角速度
 *         雷达点云，雷达和IMU的坐标变换
 * @Output:GNSS组合导航的位置和姿态
 *         畸变补偿后的点云
 * ****/

#include "data_pretreatment/data_pretreat_flow.hpp"
#include "glog/logging.h"


namespace lidar_project
{
DataPretreatFlow::DataPretreatFlow(ros::NodeHandle& nh){
    //数据订阅
    // cloud_sub_ptr_ = std::make_shared<CloudSubscriber>(nh,"/kitti/velo/pointcloud",100000);
    // imu_sub_ptr_ = std::make_shared<IMUSubscriber>(nh,"/kitti/oxts/imu",100000);
    // gnss_sub_ptr_ = std::make_shared<GNSSSubscriber>(nh,"/kitti/oxts/gps/fix",1000000);
    // velocity_sub_ptr_ = std::make_shared<VelocitySubscriber>(nh,"/kitti/oxts/gps/vel",1000000);
    // lidar_to_imu_ptr_ = std::make_shared<TFListener>(nh,"imu_link","velo_link");

    cloud_sub_ptr_ = std::shared_ptr<CloudSubscriber>(new CloudSubscriber(nh,"/kitti/velo/pointcloud",100000) );
    imu_sub_ptr_ = std::shared_ptr<IMUSubscriber>(new IMUSubscriber(nh,"/kitti/oxts/imu",100000));
    gnss_sub_ptr_ = std::shared_ptr<GNSSSubscriber>(new GNSSSubscriber(nh,"/kitti/oxts/gps/fix",1000000));
    velocity_sub_ptr_ = std::shared_ptr<VelocitySubscriber>(new VelocitySubscriber(nh,"/kitti/oxts/gps/vel",1000000));
    lidar_to_imu_ptr_ = std::shared_ptr<TFListener>(new TFListener(nh,"imu_link","velo_link"));

    //数据发布
    // cloud_pub_ptr_ = std::make_shared<CloudPublisher>(nh,"/synced_cloud","/velo_link",100);
    // gnss_pub_ptr_ = std::make_shared<OdometryPublisher>(nh,"/synced_gnss","map","/velo_link",100);
    cloud_pub_ptr_ = std::shared_ptr<CloudPublisher>(new CloudPublisher(nh,"/synced_cloud","/velo_link",100));
    gnss_pub_ptr_ = std::shared_ptr<OdometryPublisher>(new OdometryPublisher(nh,"/synced_gnss","map","/velo_link",100));
    //畸变矫正
    // distortion_adjust_ptr_ = std::make_shared<DistortionAdjust>();
    distortion_adjust_ptr_ = std::shared_ptr<DistortionAdjust>(new DistortionAdjust());

}

//数据预处理运行流程
bool DataPretreatFlow::Run(){
    if(!ReadData())  return false;                     //读取数据
    if(!InitCalibration())  return false;           //得到lidar到imu的转换
    if(!InitGNSS())  return false;                      //初始化GNSS数据

    while (HasData())
    {
        if(!ValidData())   continue;                    //验证数据有效性

        TransformsData();                                   //坐标系转换
        PublishData();                                           //数据发布
    }

    return true;
    
}

//订阅的数据进行处理，先进行同步
bool DataPretreatFlow::ReadData(){
    cloud_sub_ptr_->ParaData(cloud_data_buff_);

    static std::deque<IMUData> unsynced_imu_;   
    static std::deque<GNSSData> unsynced_gnss_;
    static std::deque<VelocityData> unsynced_velocity_;
    
    //数据存储到队列中
    gnss_sub_ptr_->ParasData(unsynced_gnss_);
    imu_sub_ptr_->ParasData(unsynced_imu_);   
    velocity_sub_ptr_->ParaData(unsynced_velocity_);
    

    if(cloud_data_buff_.size() == 0)  
        return false;

    //以点云数据的时间作为基准，对其他数据进行同步
    double cloud_time = cloud_data_buff_.front().time;
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
bool DataPretreatFlow::InitCalibration(){
    static bool calibration_received = false;
    if(!calibration_received){
        if(lidar_to_imu_ptr_->LookupData(lidar_to_imu_)){
            calibration_received = true;
        }
    }
    // std::cout<<"Initialized Clibration"<<std::endl;
    return calibration_received;

}

//初始化GNSS数据，做个转换
bool DataPretreatFlow::InitGNSS(){
    static bool gnss_init = false;
    if(!gnss_init && gnss_data_buff_.size()>0){
        GNSSData gnss_data = gnss_data_buff_.front();
        gnss_data.InitOriginPose();//初始化经纬度
        gnss_init = true;
    }
    // std::cout<<"GNSS Initialized"<<std::endl;
    return gnss_init;

}


//检验数据容器是否为空
bool DataPretreatFlow::HasData(){
    if(cloud_data_buff_.size() == 0) return false;
    if(imu_data_buff_.size() == 0) return false;
    if(gnss_data_buff_.size() == 0) return false;
    if(velocity_data_buff_.size() == 0) return false;
    // std::cout<<"Has data"<<std::endl;
    
    return true;
}


//验证数据有效性，判断是否属于统一帧数据，主要就是时间戳的差不要太大
bool DataPretreatFlow::ValidData(){

    current_cloud_data_ = cloud_data_buff_.front();
    current_imu_data_ = imu_data_buff_.front();
    current_gnss_data_ = gnss_data_buff_.front();
    current_velocity_data_ = velocity_data_buff_.front();

    double diff_imu_time = current_cloud_data_.time - current_imu_data_.time;
    double diff_velo_time = current_cloud_data_.time - current_velocity_data_.time;
    double diff_gnss_time = current_cloud_data_.time - current_gnss_data_.time;
    if(diff_imu_time < -0.05 || diff_gnss_time < -0.05 || diff_velo_time < -0.05){
        cloud_data_buff_.pop_front();
        return false;
    }
    if(diff_imu_time > 0.05){
        imu_data_buff_.pop_front();
        return false;
    }
    if(diff_gnss_time > 0.05){
        gnss_data_buff_.pop_front();//gps数据早了
        return false;
    }
    if(diff_velo_time > 0.05){
        velocity_data_buff_.pop_front();
        return false;
    }
    //否则就正常弹出，处理下一帧
    cloud_data_buff_.pop_front();
    imu_data_buff_.pop_front();
    gnss_data_buff_.pop_front();//都弹出，下次处理下一帧
    velocity_data_buff_.pop_front();
    // std::cout<<"Valid Data"<<std::endl;
    return true;
}



//GNSS数据转换为笛卡尔坐标系，速度信息转换到雷达，按理说要用imu的速度，
//但是这里用的gnss的数据，另外对点云进行畸变矫正
bool DataPretreatFlow::TransformsData(){
    gnss_pose_ = Eigen::Matrix4f::Identity();

    current_gnss_data_.UpdateXYZ();//转换为xyz
    gnss_pose_.topRightCorner(3,1) << current_gnss_data_.local_x,
                                           current_gnss_data_.local_y,
                                           current_gnss_data_.local_z;
    gnss_pose_.block(0,0,3,3) = current_imu_data_.GetRotateMat();
    gnss_pose_ *= lidar_to_imu_;

    current_velocity_data_.TransformCoordinate(lidar_to_imu_);
    distortion_adjust_ptr_->SetMotionInfo(0.1,current_velocity_data_);
    distortion_adjust_ptr_->AdjustCloud(current_cloud_data_.cloud_ptr,current_cloud_data_.cloud_ptr);

    return true;
}


//发布处理后的点云数据和GNSS位姿数据
bool DataPretreatFlow::PublishData(){
    cloud_pub_ptr_->Publish(current_cloud_data_.cloud_ptr,current_cloud_data_.time);
    gnss_pub_ptr_->Publish(gnss_pose_,current_gnss_data_.time);

    return true;



}



} // namespace lidar_project

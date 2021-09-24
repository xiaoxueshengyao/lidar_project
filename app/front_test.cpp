/***
 * Description: A test of front_end flow(copy from renqian)
 * Author: Capta1nY
 * Data: 0412
 * 0413 完成调试，可以看到只用ndt的雷达里程计和groundtruth有差距，
 * 使用的运动模型，根据前一帧的运动作为当前帧的运动预测，效果不理想，
 * 总体的关键帧 + local map + global map格局合理借鉴学习
 * ***/

#include <deque>
#include "kitti_data.hpp"//这里进行引用会出现“不明确”的提示
#include "front_end/front_end_back.hpp"
#include "tf_listener.hpp"

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <nav_msgs/Odometry.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>

using namespace lidar_project;

std::deque<CloudData> cloud_data_all;
std::deque<IMUData> imu_data_all;
std::deque<GNSSData> gnss_data_all;

void PointcloudCallback(const sensor_msgs::PointCloud2ConstPtr& kitti_laser){
    //需要把数据进行存储，可以使用全局变量，或者用sub类
    CloudData cloud_buff;
    cloud_buff.time = kitti_laser->header.stamp.toSec();
    pcl::fromROSMsg(*kitti_laser,*(cloud_buff.cloud_ptr));

    std::deque<CloudData> new_cloud;
    new_cloud.push_back(cloud_buff);
    if(new_cloud.size()>0){
        cloud_data_all.insert(cloud_data_all.end(),new_cloud.begin(),new_cloud.end());
    }
    new_cloud.clear();
    
}

void ImuCallback(const sensor_msgs::ImuConstPtr& kitti_imu){
    IMUData imu_data;
    imu_data.time = kitti_imu->header.stamp.toSec();
    imu_data.angular_velocity.x = kitti_imu->angular_velocity.x;//对角速度积分得到角度
    imu_data.angular_velocity.y = kitti_imu->angular_velocity.y;
    imu_data.angular_velocity.z = kitti_imu->angular_velocity.z;

    imu_data.linear_acceleration.x = kitti_imu->linear_acceleration.x;//对加速度二次积分得到位移
    imu_data.linear_acceleration.y = kitti_imu->linear_acceleration.y;
    imu_data.linear_acceleration.z = kitti_imu->linear_acceleration.z;

    imu_data.orientation.w = kitti_imu->orientation.w;
    imu_data.orientation.x = kitti_imu->orientation.x;
    imu_data.orientation.y = kitti_imu->orientation.y;
    imu_data.orientation.z = kitti_imu->orientation.z;

    std::deque<IMUData> new_imu;
    new_imu.push_back(imu_data);
    if(new_imu.size()>0){
        imu_data_all.insert(imu_data_all.end(),new_imu.begin(),new_imu.end());
    }
    new_imu.clear();


}

void GnssCallback(const sensor_msgs::NavSatFixConstPtr& kitti_gnss){
    GNSSData gnss_data;
    gnss_data.time = kitti_gnss->header.stamp.toSec();
    gnss_data.altitude = kitti_gnss->altitude;
    gnss_data.longitude = kitti_gnss->longitude;
    gnss_data.latitude = kitti_gnss->latitude;

    gnss_data.service = kitti_gnss->status.service;     //which global navigation satellite system
    gnss_data.status = kitti_gnss->status.status;       //fix type

    std::deque<GNSSData> new_gnss;
    new_gnss.push_back(gnss_data);
    if(new_gnss.size()>0){
        gnss_data_all.insert(gnss_data_all.end(),new_gnss.begin(),new_gnss.end());
    }
}



int main(int argc, char** argv){

    ros::init(argc,argv,"front_end_node");
    ros::NodeHandle nh;

    ros::Subscriber pointcloud_sub = nh.subscribe("/kitti/velo/pointcloud",100000,PointcloudCallback);
    ros::Subscriber imu_sub = nh.subscribe("/kitti/oxts/imu",1000000,ImuCallback);
    ros::Subscriber gnss_sub = nh.subscribe("/kitti/oxts/gps/fix",1000000,GnssCallback);
    TFListener lidar_to_imu_tf(nh,"velo_link","imu_link");//最终的转换就是冲lidar->IMU->map

    ros::Publisher cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("current_scan",100);
    ros::Publisher laser_odometry_pub = nh.advertise<nav_msgs::Odometry>("lidar_odom",100);
    ros::Publisher local_map_pub = nh.advertise<sensor_msgs::PointCloud2>("local_map",100);
    ros::Publisher global_map_pub = nh.advertise<sensor_msgs::PointCloud2>("global_map",100);
    ros::Publisher gnss_pub = nh.advertise<nav_msgs::Odometry>("gnss",100);

    //试下智能指针
    std::shared_ptr<FrontEnd> front_end_ptr = std::make_shared<FrontEnd>();

    bool gnss_pose_inited = false;
    bool transform_received = false;
    bool front_end_inited = false;
    Eigen::Matrix4f lidar_to_imu_mat = Eigen::Matrix4f::Identity();
    CloudData::CloudPtr local_map_ptr(new CloudData::CloudPointT);
    CloudData::CloudPtr global_map_ptr(new CloudData::CloudPointT);
    CloudData::CloudPtr current_scan_ptr(new CloudData::CloudPointT);


    double run_time = 0;//运行时间，保存全局地图用的标志位
    double init_time = 0;
    bool time_inited = false;
    bool has_global_map_published = false;

    ros::Rate rate(100);
    while(ros::ok()){
        ros::spinOnce();//调用callback,点云、gnss、imu数据进行存储

        if(!transform_received){
            if(lidar_to_imu_tf.LookupData(lidar_to_imu_mat)){
                transform_received = true;
            }
        }
        else{
            while(cloud_data_all.size()>0 && imu_data_all.size()>0 && gnss_data_all.size()>0){
                CloudData cloud_data = cloud_data_all.front();
                IMUData imu_data = imu_data_all.front();
                GNSSData gnss_data = gnss_data_all.front();

                if(!time_inited){
                    time_inited = true;
                    init_time = cloud_data.time;
                }else{
                    run_time = cloud_data.time - init_time;
                }

                double diff_time = cloud_data.time - imu_data.time;
                if(diff_time < -0.05){
                    cloud_data_all.pop_front();//点云数据早了，imu.gps留下和下一帧点云一起处理
                }else if(diff_time > 0.05){
                    imu_data_all.pop_front();
                    gnss_data_all.pop_front();//imu,gps数据早了
                }else{
                    cloud_data_all.pop_front();
                    imu_data_all.pop_front();
                    gnss_data_all.pop_front();//都弹出，下次处理下一帧

                    Eigen::Matrix4f odom_mat = Eigen::Matrix4f::Identity();
                    if(!gnss_pose_inited){
                        gnss_data.InitOriginPose();
                        gnss_pose_inited = true;
                    }
                    gnss_data.UpdateXYZ();//转换为笛卡尔坐标系
                    odom_mat.topRightCorner(3,1) << gnss_data.local_x,gnss_data.local_y,gnss_data.local_z;
                    odom_mat.block(0,0,3,3) = imu_data.GetRotateMat();
                    odom_mat *= lidar_to_imu_mat;

                    //gnss 里程计
                    nav_msgs::Odometry odomtry_mat;//用于里程计发布的格式
                    odomtry_mat.header.frame_id = "map";
                    odomtry_mat.child_frame_id = "lidar";
                    odomtry_mat.header.stamp = ros::Time::now();
                    odomtry_mat.pose.pose.position.x = odom_mat(0,3);
                    odomtry_mat.pose.pose.position.y = odom_mat(1,3);
                    odomtry_mat.pose.pose.position.z = odom_mat(2,3);
                    Eigen::Matrix3f R = odom_mat.block(0,0,3,3).matrix();//注意这里给四元数赋值的方式
                    Eigen::Quaternionf q(R);
                    odomtry_mat.pose.pose.orientation.w = q.w();
                    odomtry_mat.pose.pose.orientation.x = q.x();
                    odomtry_mat.pose.pose.orientation.y = q.y();
                    odomtry_mat.pose.pose.orientation.z = q.z();
                    gnss_pub.publish(odomtry_mat);//发布gps+imu得到的里程计信息，作为对比下面会有激光里程计的

                    //激光雷达里程计
                    if(!front_end_inited){
                        front_end_inited = true;
                        front_end_ptr->SetInitPose(odom_mat);
                    }
                    front_end_ptr->SetPredictPose(odom_mat);//这里虽然设置了gnss+imu的先验位姿，但是没有用到；自行测设后发现精度大幅提高
                    Eigen::Matrix4f laser_mat = front_end_ptr->Update(cloud_data);
                    nav_msgs::Odometry laser_odom;//这么转换太恶心了，还是写成类或者函数
                    laser_odom.header.frame_id = "map";
                    laser_odom.child_frame_id = "lidar";
                    laser_odom.header.stamp = ros::Time::now();
                    laser_odom.pose.pose.position.x = laser_mat(0,3);
                    laser_odom.pose.pose.position.y = laser_mat(1,3);
                    laser_odom.pose.pose.position.z = laser_mat(2,3);
                    Eigen::Matrix3f R_laser = laser_mat.block(0,0,3,3).matrix();//注意这里给四元数赋值的方式
                    Eigen::Quaternionf q_laser(R_laser);
                    laser_odom.pose.pose.orientation.w = q_laser.w();
                    laser_odom.pose.pose.orientation.x = q_laser.x();
                    laser_odom.pose.pose.orientation.y = q_laser.y();
                    laser_odom.pose.pose.orientation.z = q_laser.z();
                    laser_odometry_pub.publish(laser_odom);

                    //发布点云
                    //当前帧
                    front_end_ptr->GetCurrentScan(current_scan_ptr);//当前点云滤波
                    sensor_msgs::PointCloud2Ptr cloud_current(new sensor_msgs::PointCloud2());
                    pcl::toROSMsg(*current_scan_ptr,*cloud_current);
                    cloud_current->header.stamp = ros::Time::now();
                    cloud_current->header.frame_id = "map";
                    cloud_pub.publish(cloud_current);

                    //local map
                    if(front_end_ptr->GetNewLocalMap(local_map_ptr)){
                        sensor_msgs::PointCloud2Ptr cloud_local(new sensor_msgs::PointCloud2());
                        pcl::toROSMsg(*local_map_ptr,*cloud_local);
                        cloud_local->header.stamp = ros::Time::now();
                        cloud_local->header.frame_id = "map";
                        local_map_pub.publish(cloud_local);
                    }

                    //global map
                    if(run_time>460 && !has_global_map_published){
                        if(front_end_ptr->GetNewGlobalMap(global_map_ptr)){
                            sensor_msgs::PointCloud2Ptr cloud_global(new sensor_msgs::PointCloud2());
                            pcl::toROSMsg(*global_map_ptr,*cloud_global);
                            cloud_global->header.stamp = ros::Time::now();
                            cloud_global->header.frame_id = "map";
                            global_map_pub.publish(cloud_global);
                            pcl::io::savePCDFileASCII("/home/jingwan/data/global_map.pcd",*global_map_ptr);

                            has_global_map_published = true;
                        }
                    }

                }

            }
        }


        rate.sleep();

    }





    return 0;
}
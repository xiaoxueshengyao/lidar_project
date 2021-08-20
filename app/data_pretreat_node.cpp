/****
 * 数据处理，包括订阅、时间同步、点云畸变补偿、传感器坐标系统一
 * 0616
 * ****/


#include <ros/ros.h>
#include "glog/logging.h"

#include "data_pretreatment/data_pretreat_flow.hpp"

using namespace lidar_project;

int main(int argc, char** argv){
    google::InitGoogleLogging(argv[0]); 
    
    ros::init(argc,argv,"data_pretreat_node");
    ros::NodeHandle nh;

    std::shared_ptr<DataPretreatFlow> data_pretreat_flow_ptr = std::make_shared<DataPretreatFlow>(nh);

    ros::Rate rate(100);
    while(ros::ok()){
        ros::spinOnce();
        data_pretreat_flow_ptr->Run();
        rate.sleep();

    }

    return 0;

}
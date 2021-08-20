/***
 * Des: 更新后前端，主函数就一个对象创建了，简洁
 * Data: 0419
 * ***/

#include <ros/ros.h>
#include "glog/logging.h"
#include "front_end/front_end_flow.hpp"

#include <lidar_project/saveMap.h>

using namespace lidar_project;
std::shared_ptr<FrontEndFlow> _front_end_flow_ptr;

bool save_map_callback(saveMap::Request&  request, saveMap::Response& response){
    response.succeed = _front_end_flow_ptr->SaveMap();
    _front_end_flow_ptr->PublisheGlobalMap();
    return response.succeed;

}

int main(int argc, char** argv){
    google::InitGoogleLogging(argv[0]);
    FLAGS_log_dir = "/home/jingwan/lslidar_ws/src/lidar_project/Log";
    FLAGS_alsologtostderr = 1;

    ros::init(argc,argv,"front_end_flow");
    ros::NodeHandle nh;

    ros::ServiceServer service = nh.advertiseService("save_map",save_map_callback);
    _front_end_flow_ptr = std::make_shared<FrontEndFlow>(nh);

    ros::Rate rate(100);
    while (ros::ok())
    {
        ros::spinOnce();
        _front_end_flow_ptr->Run();
        rate.sleep();
    }

    return 0;
    
}
/***
 * Des:viewer对应Node
 * Data:0824
 * ****/

#include <ros/ros.h>
#include "glog/logging.h"
#include <lidar_project/saveMap.h>         //ros的服务头文件，不用写出实体，在cmakelists.txt中写出可在build自动生成

#include "mapping/viewer/viewer_flow.hpp"

using namespace lidar_project;

std::shared_ptr<ViewerFlow> _viewer_flow_ptr;
bool _need_save_map = false;

//保存地图的服务
bool save_map_callback(saveMap::Request& request, saveMap::Response& response){
    _need_save_map = true;
    response.succeed = true;
    return response.succeed;
}

int main(int argc, char** argv){
    google::InitGoogleLogging(argv[0]);
    FLAGS_log_dir = "/home/jerry/yjj_ws/src/lidar_project/Log";
    FLAGS_alsologtostderr = 1;//Set whether log messages go to stderr in addition to logfiles.
}


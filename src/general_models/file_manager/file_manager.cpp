/***
 * Des:创建文件
 * Aut:Capta1nY
 * Dat:0604
 * ***/

#include "general_models/file_manager/file_manager.hpp"

#include <boost/filesystem.hpp>
#include "glog/logging.h"

namespace lidar_project{
//打开文件
bool FileManager::CreateFile(std::ofstream& ofs, std::string file_path){
    ofs.close();
    boost::filesystem::remove(file_path.c_str());


    ofs.open(file_path.c_str(),std::ios::app);
    if(!ofs){
        LOG(WARNING)<<"No File "<<file_path<<std::endl;
        return false;
    }

    return true;
}

//是否存在该目录，如果有就删除之前保存的东西，没有就创建
bool FileManager::InitDirectory(std::string directory_path, std::string use_for){
    if(boost::filesystem::is_directory(directory_path))//根据获取的状态判断是否是目录，返回bool
    {
        boost::filesystem::remove_all(directory_path+"/tail");//递归删除整个目录结构
        LOG(INFO) << use_for <<" 存放地址: "<<std::endl<<directory_path<<std::endl<<std::endl;

        // return true;
    }

    return CreateDirectory(directory_path,use_for);
}


//创建目录，用于保存地图文件
bool FileManager::CreateDirectory(std::string directory_path){
    if(!boost::filesystem::is_directory(directory_path)){
        boost::filesystem::create_directories(directory_path);  //创建目录
    }
    if(!boost::filesystem::is_directory(directory_path)){
        LOG(WARNING)<<"无法创建文件  "<<directory_path;
        return false;
    }

    return true;
}

//多态，参数不同
bool FileManager::CreateDirectory(std::string directory_path,std::string use_for){
    if(!boost::filesystem::is_directory(directory_path)){
        boost::filesystem::create_directories(directory_path);
    }
    if(!boost::filesystem::is_directory(directory_path)){
        LOG(WARNING)<<"无法创建文件  "<<directory_path;
        return false;
    }

    LOG(INFO) <<"存放地址："<<std::endl<<directory_path << std::endl<<std::endl;
    

    return true;
}


}

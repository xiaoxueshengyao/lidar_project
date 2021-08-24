cmake_minimum_required(VERSION 2.8)
project(lidar_project)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} -O3 -Wall")
add_compile_options(-std=c++11)
add_definitions(-std=c++11)



find_package(catkin REQUIRED COMPONENTS
  roscpp  
  rospy  
  std_msgs 
  pcl_ros 
  geometry_msgs 
  tf 
  eigen_conversions
  message_generation
  std_srvs
)
set(ALL_TARGET_LIBRARIES "")#需要的库都放一起

#创建server
add_service_files(
  FILES
  saveMap.srv
)
generate_messages(
  DEPENDENCIES
  std_msgs
)


find_package(PCL REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})


find_package(Glog REQUIRED)
include_directories(${GFLAGS_INCLUDE_DIRS} ${GLOG_INCLUDE_DIRS})
if(GLOG_FOUND)
  set(GLOG_INCLUDE_DIRS ${GLOG_INCLUDE_DIR})
  set(GLOG_LIBRARIES ${GLOG_LIBRARY})
  message("Glog got =======")
endif()

#地理信息坐标
find_package(GeographicLib REQUIRED)#需要安装对应的库
include_directories(${Geographic_INCLUDE_DIRS})

find_package(PkgConfig REQUIRED)
pkg_check_modules(YAML_CPP REQUIRED yaml-cpp)
include_directories(${YAML_CPP_INCLUDEDIR})



include_directories( "/usr/include/eigen3" )
include_directories(include ${catkin_INCLUDE_DIRS})


catkin_package(
CATKIN_DEPENDS roscpp sensor_msgs std_msgs message_runtime
DEPENDS PCL
INCLUDE_DIRS include)


add_executable(test_data app/test_frame.cpp src/kitti_data/gnss_data.cpp src/kitti_data/imu_data.cpp src/tf_listener.cpp)
target_link_libraries(test_data ${catkin_LIBRARIES} 
${PCL_LIBRARIES} 
${Eigen_LIBRARIES}
${GeographicLib_LIBRARIES}
glog::glog
)

file(GLOB_RECURSE ALL_SRCS "src/*.cpp")
file(GLOB_RECURSE APP_SRCS "app/*.cpp")
list(REMOVE_ITEM ALL_SRCS ${APP_SRCS})


#add_executable(front_end app/front_test.cpp src/kitti_data.cpp src/tf_listener.cpp src/front_end_back.cpp)
#target_link_libraries(front_end ${catkin_LIBRARIES} 
#${PCL_LIBRARIES} ${Eigen_LIBRARIES} ${GeographicLib_LIBRARIES} glog::glog)

add_executable(front_end_node app/front_end_node.cpp ${ALL_SRCS} )#src/general_models/file_manager/file_manager.cpp) 
add_dependencies(front_end_node ${catkin_EXPORTED_TARGETS} saveMap_gencpp)
target_link_libraries(front_end_node ${catkin_LIBRARIES} ${YAML_CPP_LIBRARIES} ${PCL_LIBRARIES} ${Eigen_LIBRARIES} ${GeographicLib_LIBRARIES} glog::glog )
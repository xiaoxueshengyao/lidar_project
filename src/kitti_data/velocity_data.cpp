/***
 * 0603把几种数据分开不同文件，尝试解决同步时无法完成的问题
 * Capta1nY
 * ***/

#include "kitti_data/velocity_data.hpp"


namespace lidar_project{
//针对线速度和角速度进行同步，做差值运算
bool VelocityData::SyncData(std::deque<VelocityData>& UnsyncedData,std::deque<VelocityData>& SyncedData,double sync_time){
        //找到与同步时间相邻的两个数据
        //如果两个时间差距过大，说明中间有缺失的数据，不宜做差值，做了也不准
        //核心就是让第一个数据的时间早于同步时间，第二个数据晚于同步时间
        //std::cout<<"Ready to synce Velocity"<<std::endl;
        while (UnsyncedData.size() >= 2)
        {
            if(UnsyncedData.front().time > sync_time) //如果最前面的数据时间晚于参考时间
                return false;
            if(UnsyncedData.at(1).time < sync_time){    //如果第一个早于参考时间（正常情况），但是第二个也早于
                UnsyncedData.pop_front();
                continue;
            }
            if(sync_time - UnsyncedData.front().time > 0.2){ //如果前两个的数据的时间卡在两边，但是最前面的数据离同步时间过长
                UnsyncedData.pop_front();
                break;
            }
            if(UnsyncedData.at(1).time - sync_time > 0.2){
                UnsyncedData.pop_front();
                break;
            }
            break;
        
        }
        if(UnsyncedData.size() < 2) return false;//只有俩数据时没法做差值

        VelocityData front_data = UnsyncedData.at(0);
        VelocityData back_data = UnsyncedData.at(1);
        VelocityData synced_data;

        //差值两个数据的系数
        double front_scale = (back_data.time - sync_time) / (back_data.time - front_data.time);
        double back_scale = (sync_time - front_data.time) / (back_data.time - front_data.time);
        synced_data.time = sync_time;

        //差值计算公式a*(1-t) + b*t,为什么呢，这样当t=0的时候就还是0，符合实际情况
        synced_data.linear_velocity.x = front_scale * front_data.linear_velocity.x + back_scale * back_data.linear_velocity.x;
        synced_data.linear_velocity.y = front_scale * front_data.linear_velocity.y + back_scale * back_data.linear_velocity.y;
        synced_data.linear_velocity.z = front_scale * front_data.linear_velocity.z + back_scale * back_data.linear_velocity.z;
        synced_data.angular_velocity.x = front_scale * front_data.angular_velocity.x + back_scale * back_data.angular_velocity.x;
        synced_data.angular_velocity.y = front_scale * front_data.angular_velocity.y + back_scale * back_data.angular_velocity.y;
        synced_data.angular_velocity.z = front_scale * front_data.angular_velocity.z + back_scale * back_data.angular_velocity.z;   

        SyncedData.push_back(synced_data);
        //std::cout<<"Got Velocity Synced data"<<std::endl;
        return true;                                                 
                            
                            
}

/***
 * 把imu的线速度和角速度转换到雷达坐标系上
 * 由于IMU和雷达所处的位置并不重合，存在杆臂，所以在转弯时两者的速度并不一致，
 * 需要按照两者的相对坐标，把速度转换到雷达上
***/
void VelocityData::TransformCoordinate(Eigen::Matrix4f& transform_matrix){
    //先对旋转进行处理
    Eigen::Matrix4d matrix = transform_matrix.cast<double>();
    matrix = matrix.inverse();
    Eigen::Matrix3d t_R = matrix.block(0,0,3,3);
    Eigen::Vector3d w(angular_velocity.x, angular_velocity.y, angular_velocity.z);
    Eigen::Vector3d v(linear_velocity.x, linear_velocity.y, linear_velocity.z);
    w = t_R * w;
    v = t_R * v;
    //平移
    Eigen::Vector3d r(matrix(0,3),matrix(1,3),matrix(2,3));
    Eigen::Vector3d delta_v;
    delta_v(0) = w(1) * r(2) - w(2) * r(1);
    delta_v(1) = w(2) * r(0) - w(0) * r(2);
    delta_v(2) = w(0) * r(1) - w(1) * r(0);
    v = v + delta_v;

    angular_velocity.x = w(0);
    angular_velocity.y = w(1);
    angular_velocity.z = w(2);
    linear_velocity.x = v(0);
    linear_velocity.y = v(1);
    linear_velocity.z = v(2);
}


}
/***
 * Des:运行时间测试工具
 * 0927
 * ***/
#ifndef TIC_TOC_HPP_
#define TIC_TOC_HPP_

#include <ctime>
#include <cstdlib>
#include <chrono>

namespace lidar_project{
class TicToc{
    public:
        TicToc(){
            tic();
        }

        void tic(){
            start = std::chrono::system_clock::now();
        }

        double toc(){
            end = std::chrono::system_clock::now();
            std::chrono::duration<double> time_used = end - start;
            start = std::chrono::system_clock::now();
            return time_used.count();//返回耗时
        }
    private:
        std::chrono::time_point<std::chrono::system_clock> start,end;

};


}






#endif
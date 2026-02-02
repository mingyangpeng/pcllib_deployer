#ifndef UTIL_HPP_
#define UTIL_HPP_
#include <Eigen/Dense>
#include <iostream> 
#include <chrono>
#include <thread>
#include <string>
#include <filesystem>
 

namespace pcllibs{  
namespace util{


// 定义计时器类
class ScopedTimer{
    public:
        // 构造函数：传入任务名称（可选），并记录开始时间
        explicit ScopedTimer(std::string name = "Block") 
            : task_name(std::move(name)), start_time(std::chrono::high_resolution_clock::now()) {}
    
        // 析构函数：离开作用域时自动调用，计算并打印耗时
        ~ScopedTimer();  
        // 禁止拷贝，防止意外行为
        ScopedTimer(const ScopedTimer&) = delete;
        ScopedTimer& operator=(const ScopedTimer&) = delete; 
    private:
        std::string task_name;
        std::chrono::time_point<std::chrono::high_resolution_clock> start_time;
    };
    

// 检查变换矩阵是否为正交变换矩阵
bool isRotationMatrixOrthogonal(const Eigen::Matrix4f& T, float tolerance = 1e-5f);

bool isRotationMatrixOrthogonal(const Eigen::Matrix3f& R, float tolerance = 1e-5f);

// 计算T1*T2的复合变换矩阵, 并确保结果矩阵是正交变换矩阵
Eigen::Matrix4f mulT(const Eigen::Matrix4f& T1, const Eigen::Matrix4f& T2);

// 创建文件夹，返回文件夹路径。 
// 如果文件夹已存在，则不创建，isfolder=true给定的路径默认是文件夹路径，isfolder=flase给定的路径默认是文件路径
std::string createFolder(const std::string& path, bool isfolder=true);
 
    
} // namespace util

} // namespace pcllibs

#endif //UTIL_HPP_
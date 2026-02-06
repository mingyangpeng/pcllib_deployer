/**
 * @copyright  Copyright (c) 2026 pengmingyang All Rights Reserved.
 * @file register.hpp
 * @brief 点云功能
 * @author pengmingyang
 */
#ifndef UTIL_HPP_
#define UTIL_HPP_
#include <pcl/common/centroid.h>
#include <pcl/common/pca.h>
#include <pcl/features/moment_of_inertia_estimation.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>
#include <pcl/search/search.h>

#include <Eigen/Dense>
#include <chrono>
#include <filesystem>
#include <iostream>
#include <string>
#include <thread>

#include "datastruct.hpp"

namespace pcllibs {
namespace util {

// 定义计时器类
class ScopedTimer {
   public:
    // 构造函数：传入任务名称（可选），并记录开始时间
    explicit ScopedTimer(std::string name = "Block")
        : task_name(std::move(name)),
          start_time(std::chrono::high_resolution_clock::now()) {}

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
bool isRotationMatrixOrthogonal(const Eigen::Matrix4f& T,
                                float tolerance = 1e-5f);

bool isRotationMatrixOrthogonal(const Eigen::Matrix3f& R,
                                float tolerance = 1e-5f);

// 计算T1*T2的复合变换矩阵, 并确保结果矩阵是正交变换矩阵
Eigen::Matrix4f mulT(const Eigen::Matrix4f& T1, const Eigen::Matrix4f& T2);

// 创建文件夹，返回文件夹路径。
// 如果文件夹已存在，则不创建，isfolder=true给定的路径默认是文件夹路径，isfolder=flase给定的路径默认是文件路径
std::string createFolder(const std::string& path, bool isfolder = true);

/**
 * @brief 计算点云的法向量
 * @param cloud_in 输入点云
 * @param cloud_normal 输出法向量点云
 * @param k_search 搜索邻域的点的个数
 */
template <typename PointInT, typename PointOutT>
void computeCloudNormal(
    const typename pcl::PointCloud<PointInT>::Ptr& cloud_in,
    typename pcl::PointCloud<PointOutT>::Ptr& cloud_normal, float search_radius,
    int k_search,
    typename pcl::search::KdTree<PointInT>::Ptr search_tree = nullptr) {
    typename pcl::search::KdTree<PointInT>::Ptr tree;
    if (!search_tree) {
        tree.reset(new pcl::search::KdTree<PointInT>());
    }
    if (tree->getInputCloud() != cloud_in) {
        tree->setInputCloud(cloud_in);
    }
    cloud_normal->clear();
    cloud_normal->resize(cloud_in->size());
    cloud_normal->header = cloud_in->header;
    cloud_normal->sensor_origin_ = cloud_in->sensor_origin_;
    cloud_normal->sensor_orientation_ = cloud_in->sensor_orientation_;
    if constexpr (std::is_same_v<PointOutT, pcl::PointNormal>) {
#pragma omp parallel for
        for (size_t i = 0; i < cloud_in->size(); ++i) {
            cloud_normal->points[i].x = cloud_in->points[i].x;
            cloud_normal->points[i].y = cloud_in->points[i].y;
            cloud_normal->points[i].z = cloud_in->points[i].z;
        }
    }

    pcl::NormalEstimation<PointInT, PointOutT> ne;
    ne.setInputCloud(cloud_in);
    ne.setSearchMethod(tree);

    if (search_radius > 0.0f) {
        ne.setRadiusSearch(search_radius);
    } else {
        ne.setKSearch(k_search);
    }
    ne.setViewPoint(0.0f, 0.0f, 1.0f);
    ne.compute(*cloud_normal);
}

void getOBBCorners(const OBBDate& obb, std::vector<Eigen::Vector3f>& vertices);

template <typename PointT>
bool calcOBB(const typename pcl::PointCloud<PointT>::Ptr& cloud, OBBDate& obb) {
    try {
        // 使用PCL的MomentOfInertiaEstimation计算OBB
        pcl::MomentOfInertiaEstimation<pcl::PointXYZ> moe;
        moe.setInputCloud(cloud);
        moe.compute();

        pcl::PointXYZ min_point, max_point, position;
        Eigen::Matrix3f rotational_matrix;

        moe.getOBB(min_point, max_point, position, rotational_matrix);

        // 填充OBB结构体
        obb.center = Eigen::Vector3f(position.x, position.y, position.z);
        obb.min_point = Eigen::Vector3f(min_point.x, min_point.y, min_point.z);
        obb.max_point = Eigen::Vector3f(max_point.x, max_point.y, max_point.z);
        obb.axes = rotational_matrix;

        // 计算半尺寸
        obb.extents = (obb.max_point - obb.min_point) / 2.0f;

        // 计算OBB的8个顶点
        std::vector<Eigen::Vector3f> vertices;
        getOBBCorners(obb, vertices);
        obb.vertices = vertices;

        // 计算原始坐标系下的最小和最大点
        obb.min_point = vertices[0];
        obb.max_point = vertices[0];
        for (size_t i = 1; i < vertices.size(); ++i) {
            obb.min_point = obb.min_point.cwiseMin(vertices[i]);
            obb.max_point = obb.max_point.cwiseMax(vertices[i]);
        }

        return true;
    } catch (const std::exception& e) {
        std::cerr << "Error computing OBB: " << e.what() << std::endl;
        return false;
    }
}

}  // namespace util

}  // namespace pcllibs

#endif  // UTIL_HPP_
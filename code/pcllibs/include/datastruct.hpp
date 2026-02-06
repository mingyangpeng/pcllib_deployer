/**
 * @copyright  Copyright (c) 2026 pengmingyang All Rights Reserved.
 * @file datastruct.hpp
 * @brief 数据结构定义
 * @author pengmingyang
 */
#ifndef DATASTRUCT_HPP
#define DATASTRUCT_HPP
#include <pcl/point_types.h>

#include <Eigen/Dense>
namespace pcllibs {

// OBB 数据结构
struct OBBDate {
    Eigen::Vector3f center;     // 包围盒中心点
    Eigen::Vector3f extents;    // 包围盒半尺寸 (长、宽、高的一半)
    Eigen::Matrix3f axes;       // 包围盒的三个主轴方向
    Eigen::Vector3f min_point;  // 原始坐标系下的最小点
    Eigen::Vector3f max_point;  // 原始坐标系下的最大点
    std::vector<Eigen::Vector3f> vertices;  // 包围盒顶点
};
}  // namespace pcllibs

#endif  // DATASTRUCT_HPP
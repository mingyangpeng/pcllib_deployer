/**
 * @copyright  Copyright (c) 2026 pengmingyang All Rights Reserved.
 * @file io.hpp
 * @brief 点云io
 * @author pengmingyang
 */
#ifndef IO_HPP
#define IO_HPP

#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <Eigen/Dense>
#include <cmath>
#include <iostream>

#include "util.hpp"

namespace pcllibs {

namespace io {

template <typename PointT>
void loadPointCloud(const std::string& pcd_path,
                    typename pcl::PointCloud<PointT>::Ptr& cloud, float scale) {
    pcl::io::loadPCDFile(pcd_path, *cloud);
    if (scale != 1.0) {
        Eigen::Affine3f scale_transform = Eigen::Affine3f::Identity();
        scale_transform.scale(0.001);
        pcl::transformPointCloud(*cloud, *cloud, scale_transform);
    }
}

template <typename PointT>
bool savePointCloud(const std::string& file_name,
                    const typename pcl::PointCloud<PointT>::Ptr& cloud) {
    util::createFolder(file_name, false);
    try {
        pcl::io::savePCDFileASCII(file_name, *cloud);
    } catch (const pcl::IOException& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return false;
    }
    return true;
}

}  // namespace io

}  // namespace pcllibs

#endif  // IO_HPP
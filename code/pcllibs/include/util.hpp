#ifndef UTIL_HPP_
#define UTIL_HPP_

#include <iostream>
#include <Eigen/Dense>
#include <cmath>
#include <pcl/io/pcd_io.h> 
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>


template<typename PointT>
void loadPointCloud(const std::string& pcd_path, typename pcl::PointCloud<PointT>::Ptr& cloud, float scale=1.0);


#endif //UTIL_HPP_
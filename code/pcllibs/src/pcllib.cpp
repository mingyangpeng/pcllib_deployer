#include "pcllib.hpp"
PCLUTILS::PCLUTILS() {
    // 构造函数实现
}

PCLUTILS::~PCLUTILS() {
    // 析构函数实现
}

// pcllib.cpp
template<typename PointT>
void PCLUTILS::showPointCloud(const typename pcl::PointCloud<PointT>::Ptr& cloud, const std::string& window_name) {
    show_1<PointT>(cloud, window_name);  // 调用viewer.hpp中的函数
}

template void PCLUTILS::showPointCloud<pcl::PointXYZ>(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, const std::string& window_name);
template void PCLUTILS::showPointCloud<pcl::PointXYZI>(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud, const std::string& window_name);
template void PCLUTILS::showPointCloud<pcl::PointXYZRGB>(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud, const std::string& window_name);
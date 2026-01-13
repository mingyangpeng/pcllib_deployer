#ifndef VIEWER_HPP_
#define VIEWER_HPP_

#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/point_cloud_color_handlers.h>




// 添加模板参数声明
template <typename PointT>
void show_1(const typename pcl::PointCloud<PointT>::Ptr &cloud, const std::string &window_name = "3D Viewer");

#endif // VIEWER_HPP_
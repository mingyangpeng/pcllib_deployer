#ifndef VIEWER_HPP_
#define VIEWER_HPP_
#include <thread>    
#include <chrono>    
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/point_cloud_color_handlers.h>
#include <pcl/common/transforms.h>

#include <vtkObject.h>
#include <vtkLogger.h>


namespace pcllibs{
     
namespace viewer{

// 添加模板参数声明
template <typename PointT>
void show_1(const typename pcl::PointCloud<PointT>::Ptr &cloud, const std::string &window_name = "3D Viewer"){
    // 直接创建智能指针而非复制对象
    pcl::visualization::PCLVisualizer::Ptr viewer_ptr(new pcl::visualization::PCLVisualizer(window_name));
    viewer_ptr->addPointCloud<PointT>(cloud, "pcd");
    viewer_ptr->setBackgroundColor(0, 0, 0);

    viewer_ptr->addCoordinateSystem(1.0);
    viewer_ptr->initCameraParameters();

    bool exit_flag = false;
    viewer_ptr->registerKeyboardCallback([&exit_flag](const pcl::visualization::KeyboardEvent& event) {
        if (event.getKeySym() == "q" && event.keyDown()) {
            exit_flag = true;  // 按下 q 键触发退出
        }
    });

    while (!viewer_ptr->wasStopped()&& !exit_flag) {
        viewer_ptr->spinOnce(100); 
    }
}

// 第一个点云为红色，第二个点云为蓝色
template<typename PointT>
void show_2(const typename pcl::PointCloud<PointT>::Ptr &cloud1, 
            const typename pcl::PointCloud<PointT>::Ptr &cloud2,  
            const std::string& window_name="3D Viewer"){
    pcl::visualization::PCLVisualizer viewer(window_name);
    viewer.setBackgroundColor(0, 0, 0); 

    viewer.addPointCloud<PointT>(cloud1, "cloud1");
    viewer.addPointCloud<PointT>(cloud2, "cloud2");
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "cloud1");
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 0.0, 0.0, "cloud1"); // Red color for cloud1
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "cloud2");
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.0, 0.0, 1.0, "cloud2"); // Blue color for cloud2
    viewer.addCoordinateSystem(1.0);
    viewer.initCameraParameters();
    while (!viewer.wasStopped()) {
        viewer.spinOnce();
    }   
    viewer.close(); 
}

// 第一个点云为红色，第二个点云为蓝色
template<typename PointT>
void show_2(const typename pcl::PointCloud<PointT>::Ptr &cloud1, 
            const typename pcl::PointCloud<PointT>::Ptr &cloud2, 
            const Eigen::Matrix4f& t1,
            const Eigen::Matrix4f& t2,
            const std::string& window_name="3D Viewer"){
    pcl::visualization::PCLVisualizer viewer(window_name);
    viewer.setBackgroundColor(0, 0, 0);
    typename pcl::PointCloud<PointT>::Ptr cloud1_transformed(new pcl::PointCloud<PointT>); 
    pcl::transformPointCloud<PointT>(*cloud1, *cloud1_transformed, t1);
    typename pcl::PointCloud<PointT>::Ptr cloud2_transformed(new pcl::PointCloud<PointT>); 
    pcl::transformPointCloud<PointT>(*cloud2, *cloud2_transformed, t2);

    viewer.addPointCloud<PointT>(cloud1_transformed, "cloud1");
    viewer.addPointCloud<PointT>(cloud2_transformed, "cloud2");
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "cloud1");
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 0.0, 0.0, "cloud1"); // Red color for cloud1
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "cloud2");
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.0, 0.0, 1.0, "cloud2"); // Blue color for cloud2
    viewer.addCoordinateSystem(1.0);
    viewer.initCameraParameters();
    while (!viewer.wasStopped()) {
        viewer.spinOnce();
    }   
    viewer.close(); 
}

} // namespace viewer


} // namespace pcllibs




#endif // VIEWER_HPP_
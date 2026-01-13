#include "viewer.hpp"

template <typename PointT>
void show_1(const typename pcl::PointCloud<PointT>::Ptr &cloud, const std::string &window_name) {
    // 直接创建智能指针而非复制对象
    pcl::visualization::PCLVisualizer::Ptr viewer_ptr(new pcl::visualization::PCLVisualizer(window_name));
    viewer_ptr->addPointCloud<PointT>(cloud, "fusion_xyzrgb");
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
        // std::this_thread::sleep_for(std::chrono::milliseconds(100)); // 添加睡眠以避免CPU占用过高
    }
}
template void show_1<pcl::PointXYZ>(const pcl::PointCloud<pcl::PointXYZ>::Ptr&, const std::string&);
template void show_1<pcl::PointXYZI>(const pcl::PointCloud<pcl::PointXYZI>::Ptr&, const std::string&);
template void show_1<pcl::PointNormal>(const pcl::PointCloud<pcl::PointNormal>::Ptr&, const std::string&);
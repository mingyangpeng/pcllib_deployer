#include "pcllib.hpp"

int main() {
    
    std::string pcd_path = "/root/code/temp/stands_cloud_Module_C.pcd";
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    loadPointCloud<pcl::PointXYZ>(pcd_path, cloud);
    auto utils_ptr = std::make_shared<PCLUTILS>();
    utils_ptr->showPointCloud<pcl::PointXYZ>(cloud);

    // std::cout << cloud->size() << std::endl;
    // std::cout << cloud->points[0].x << std::endl;
}
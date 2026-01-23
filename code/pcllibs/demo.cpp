#include "pcllib.hpp"
using namespace pcllibs;
int main() {
    
    std::string pcd_path = "/root/code/temp/stands_cloud_Module_C.pcd";
    std::string pcd_path1 = "/root/code/temp/ideal_stands_pcd_Module_C.pcd";
    auto pcl_ptr = std::make_shared<PCLUTILS>(); 
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1(new pcl::PointCloud<pcl::PointXYZ>);
    
    { 
        TIME_SCOPE("计时器");
        pcl_ptr->loadPointCloud<pcl::PointXYZ>(pcd_path, cloud);
        pcl_ptr->loadPointCloud<pcl::PointXYZ>(pcd_path1, cloud1);
        pcl_ptr->showPointCloud<pcl::PointXYZ>(cloud, cloud1);
    }
    
}
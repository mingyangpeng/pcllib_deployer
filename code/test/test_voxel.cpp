#include "pcllib.hpp"
using namespace pcllibs;
int main() {
    
    std::string pcd_path = "/root/code/temp/stands_cloud_Module_C.pcd";
    std::string pcd_path1 = "/root/code/temp/ideal_stands_pcd_Module_C.pcd";
    auto pcl_ptr = std::make_shared<PCLUTILS>(); 
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1(new pcl::PointCloud<pcl::PointXYZ>);

    Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
    Eigen::Matrix4f transform1 = Eigen::Matrix4f::Identity();
    transform(0, 3) =7;
    transform(1, 3) =1.5;
    transform(2, 3) =0.5;
    
    { 
        TIME_SCOPE("计时器");
        pcl_ptr->loadPointCloud<pcl::PointXYZ>(pcd_path, cloud);
        pcl_ptr->loadPointCloud<pcl::PointXYZ>(pcd_path1, cloud1);
        pcl_ptr->showPointCloud<pcl::PointXYZ>(cloud, cloud1);
        pcl_ptr->showPointCloud<pcl::PointXYZ>(cloud, cloud1, transform, transform1);
        pcl_ptr->savePointCloud<pcl::PointXYZ>( "/root/code/temp/stands_cloud_Module_C1.pcd",cloud);
    }
    
}
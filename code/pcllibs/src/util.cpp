#include "util.hpp"

template<typename PointT>
void loadPointCloud(const std::string& pcd_path, typename pcl::PointCloud<PointT>::Ptr& cloud, float scale){
 
    pcl::io::loadPCDFile(pcd_path, *cloud); 
    if (scale != 1.0){
        Eigen::Affine3f scale_transform = Eigen::Affine3f::Identity();
        scale_transform.scale(0.001);
        pcl::transformPointCloud(*cloud, *cloud, scale_transform);
    }
    
}
template void loadPointCloud<pcl::PointXYZ>(const std::string&, pcl::PointCloud<pcl::PointXYZ>::Ptr&, float);
template void loadPointCloud<pcl::PointXYZI>(const std::string&, pcl::PointCloud<pcl::PointXYZI>::Ptr&, float);
template void loadPointCloud<pcl::PointXYZRGB>(const std::string&, pcl::PointCloud<pcl::PointXYZRGB>::Ptr&, float);

 

// void savePointCloud(std::shared_ptr<pcl::PointCloud<pcl::PointXYZI>> &pcd,
//     const std::string &save_pcd_path, const std::string &lidar_name,
//     const std::string &msg_id, const std::string &timestamp_str) {
//     std::string pcd_path = save_pcd_path + "/" + lidar_name + "/" + msg_id + "/" +
//                     timestamp_str + ".pcd";
//     std::filesystem::path dir_path =
//     std::filesystem::path(pcd_path).parent_path();
//     if (!std::filesystem::exists(dir_path)) {
//     std::filesystem::create_directories(dir_path);
//     }
//     pcl::io::savePCDFileASCII(pcd_path, *pcd);
 
// }

#include "pcllib.hpp"

namespace pcllibs
{
    


PCLUTILS::PCLUTILS() {
    vtkObject::GlobalWarningDisplayOff();  // 关闭vtk的警告信息
    // 构造函数实现
}

PCLUTILS::~PCLUTILS() {
    // 析构函数实现
}

// pcllib.cpp
template<typename PointT>
void PCLUTILS::showPointCloud(const typename pcl::PointCloud<PointT>::Ptr& cloud, const std::string& window_name) {
    viewer::show_1<PointT>(cloud, window_name);  // 调用viewer.hpp中的函数
}

template void PCLUTILS::showPointCloud<pcl::PointXYZ>(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, const std::string& window_name);
template void PCLUTILS::showPointCloud<pcl::PointXYZI>(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud, const std::string& window_name);
template void PCLUTILS::showPointCloud<pcl::PointXYZRGB>(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud, const std::string& window_name);
template void PCLUTILS::showPointCloud<pcl::PointNormal>(const pcl::PointCloud<pcl::PointNormal>::Ptr& cloud, const std::string& window_name);


template<typename PointT>
void PCLUTILS::showPointCloud(  const typename pcl::PointCloud<PointT>::Ptr& cloud1, 
                                const typename pcl::PointCloud<PointT>::Ptr& cloud2, 
                                const std::string& window_name){
    viewer::show_2<PointT>(cloud1,cloud2,window_name);  // 调用viewer.hpp中的函数
}  
template void PCLUTILS::showPointCloud<pcl::PointXYZ>(  const pcl::PointCloud<pcl::PointXYZ>::Ptr&, 
                                                        const pcl::PointCloud<pcl::PointXYZ>::Ptr&,  
                                                        const std::string&);
template void PCLUTILS::showPointCloud<pcl::PointXYZI>( const pcl::PointCloud<pcl::PointXYZI>::Ptr&, 
                                                        const pcl::PointCloud<pcl::PointXYZI>::Ptr&,  
                                                        const std::string&);
template void PCLUTILS::showPointCloud<pcl::PointXYZRGB>(   const pcl::PointCloud<pcl::PointXYZRGB>::Ptr&, 
                                                            const pcl::PointCloud<pcl::PointXYZRGB>::Ptr&, 
                                                            const std::string&);
template void PCLUTILS::showPointCloud<pcl::PointNormal>(   const pcl::PointCloud<pcl::PointNormal>::Ptr&, 
                                                            const pcl::PointCloud<pcl::PointNormal>::Ptr&,  
                                                            const std::string&);
template<typename PointT>
void PCLUTILS::showPointCloud(  const typename pcl::PointCloud<PointT>::Ptr& cloud1, 
                                const typename pcl::PointCloud<PointT>::Ptr& cloud2,
                                const Eigen::Matrix4f &T1, 
                                const Eigen::Matrix4f &T2, 
                                const std::string& window_name){
    viewer::show_2<PointT>(cloud1,cloud2,T1,T2,window_name);  // 调用viewer.hpp中的函数
}  
template void PCLUTILS::showPointCloud<pcl::PointXYZ>(  const pcl::PointCloud<pcl::PointXYZ>::Ptr&, 
                                                        const pcl::PointCloud<pcl::PointXYZ>::Ptr&, 
                                                        const Eigen::Matrix4f&, 
                                                        const Eigen::Matrix4f&, 
                                                        const std::string&);
template void PCLUTILS::showPointCloud<pcl::PointXYZI>( const pcl::PointCloud<pcl::PointXYZI>::Ptr&, 
                                                        const pcl::PointCloud<pcl::PointXYZI>::Ptr&, 
                                                        const Eigen::Matrix4f&, 
                                                        const Eigen::Matrix4f&, 
                                                        const std::string&);
template void PCLUTILS::showPointCloud<pcl::PointXYZRGB>(   const pcl::PointCloud<pcl::PointXYZRGB>::Ptr&, 
                                                            const pcl::PointCloud<pcl::PointXYZRGB>::Ptr&, 
                                                            const Eigen::Matrix4f&, 
                                                            const Eigen::Matrix4f&, 
                                                            const std::string&);
template void PCLUTILS::showPointCloud<pcl::PointNormal>(   const pcl::PointCloud<pcl::PointNormal>::Ptr&, 
                                                            const pcl::PointCloud<pcl::PointNormal>::Ptr&, 
                                                            const Eigen::Matrix4f&, 
                                                            const Eigen::Matrix4f&, 
                                                            const std::string&);


template<typename PointT>
void PCLUTILS::loadPointCloud(const std::string& file_name, typename pcl::PointCloud<PointT>::Ptr& cloud, float scale){
    io::loadPointCloud<PointT>(file_name, cloud, scale);  // 调用io.hpp中的函数
}
template void PCLUTILS::loadPointCloud<pcl::PointXYZ>(const std::string& file_name, pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, float scale);
template void PCLUTILS::loadPointCloud<pcl::PointXYZI>(const std::string& file_name, pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud, float scale);
template void PCLUTILS::loadPointCloud<pcl::PointXYZRGB>(const std::string& file_name, pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud, float scale);
template void PCLUTILS::loadPointCloud<pcl::PointNormal>(const std::string& file_name, pcl::PointCloud<pcl::PointNormal>::Ptr& cloud, float scale);

template<typename PointT>
void PCLUTILS::savePointCloud(const std::string& file_name, const typename pcl::PointCloud<PointT>::Ptr& cloud){
    io::savePointCloud<PointT>(file_name, cloud);  // 调用util.hpp中的函数
}
template void PCLUTILS::savePointCloud<pcl::PointXYZ>(const std::string& file_name, const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud);
template void PCLUTILS::savePointCloud<pcl::PointXYZI>(const std::string& file_name, const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud);
template void PCLUTILS::savePointCloud<pcl::PointXYZRGB>(const std::string& file_name, const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud);
template void PCLUTILS::savePointCloud<pcl::PointNormal>(const std::string& file_name, const pcl::PointCloud<pcl::PointNormal>::Ptr& cloud);

template<typename PointT>
void downSampleVoxelGridFilter(const typename pcl::PointCloud<PointT>::Ptr& cloud_in, typename pcl::PointCloud<PointT>::Ptr& cloud_out, float leaf_size, bool calc_center){
    filter::voxelGridFilter<PointT>(cloud_in, cloud_out, leaf_size, calc_center);  // 调用filter.hpp中的函数
}
template void downSampleVoxelGridFilter<pcl::PointXYZ>(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_in, pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_out, float leaf_size, bool calc_center);
template void downSampleVoxelGridFilter<pcl::PointXYZI>(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud_in, pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud_out, float leaf_size, bool calc_center);
template void downSampleVoxelGridFilter<pcl::PointXYZRGB>(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud_in, pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud_out, float leaf_size, bool calc_center);
template void downSampleVoxelGridFilter<pcl::PointNormal>(const pcl::PointCloud<pcl::PointNormal>::Ptr& cloud_in, pcl::PointCloud<pcl::PointNormal>::Ptr& cloud_out, float leaf_size, bool calc_center);





} // namespace pcllibs
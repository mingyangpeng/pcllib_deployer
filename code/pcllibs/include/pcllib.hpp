#ifndef PCLLIB_HPP_
#define PCLLIB_HPP_

#include "util.hpp"
#include "viewer.hpp"
#include "io.hpp"
#include "filter.hpp"



namespace pcllibs
{
// #define TIME_SCOPE(name) ScopedTimer _timer_##name(name)
#define TIME_SCOPE(name) util::ScopedTimer __timer_instance__(name)


class PCLUTILS
{
public:
    PCLUTILS();
    ~PCLUTILS();

    // 显示单点云
    template<typename PointT>
    void showPointCloud(const typename pcl::PointCloud<PointT>::Ptr& cloud, const std::string& window_name = "3D Viewer");

    // 显示两幅点云，并显示变换关系
    template<typename PointT>
    void showPointCloud(const typename pcl::PointCloud<PointT>::Ptr& cloud1, const typename pcl::PointCloud<PointT>::Ptr& cloud2,
        const Eigen::Matrix4f &T1, const Eigen::Matrix4f &T2, const std::string& window_name = "3D Viewer");

    // 显示两幅点云
    template<typename PointT>
    void showPointCloud(const typename pcl::PointCloud<PointT>::Ptr& cloud1, const typename pcl::PointCloud<PointT>::Ptr& cloud2,
        const std::string& window_name = "3D Viewer");
        
    // 加载点云
    template<typename PointT>
    void loadPointCloud(const std::string& file_name, typename pcl::PointCloud<PointT>::Ptr& cloud, float scale = 1.0f);

    // 保存点云
    template<typename PointT>
    void savePointCloud(const std::string& file_name, const typename pcl::PointCloud<PointT>::Ptr& cloud);

    // 降采样--体素网格滤波
    template<typename PointT>
    void downSampleVoxelGridFilter(const typename pcl::PointCloud<PointT>::Ptr& cloud_in, typename pcl::PointCloud<PointT>::Ptr& cloud_out, float leaf_size, bool calc_center);

    // 降采样--随机采样一致性
    template<typename PointT>
    void downSampleRandomFilter(const typename pcl::PointCloud<PointT>::Ptr& cloud_in, typename pcl::PointCloud<PointT>::Ptr& cloud_out, float sampling_ratio);
    
    // 降采样--均匀采样
    template<typename PointT>
    void downSampleUniformFilter(const typename pcl::PointCloud<PointT>::Ptr& cloud_in, typename pcl::PointCloud<PointT>::Ptr& cloud_out, float radius);
    
    // 滤波--Nan值点过滤
    template<typename PointT>
    void filterNaN(const typename pcl::PointCloud<PointT>::Ptr& cloud_in, typename pcl::PointCloud<PointT>::Ptr& cloud_out);

    // 滤波--统计滤波
    template<typename PointT>
    void filterStatisticalOutlierRemoval(const typename pcl::PointCloud<PointT>::Ptr& cloud_in, typename pcl::PointCloud<PointT>::Ptr& cloud_out, int mean_k, float std_dev_mul_thresh, bool filter_nan=true);

    // 滤波--半径滤波
    template<typename PointT>
    void filterRadiusOutlierRemoval(const typename pcl::PointCloud<PointT>::Ptr& cloud_in, typename pcl::PointCloud<PointT>::Ptr& cloud_out, float radius, int min_neighbors);
    





}; // class PCLUTILS

}// namespace pcllibs
 

#endif //PCLLIB_HPP_
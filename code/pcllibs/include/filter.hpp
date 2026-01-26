#ifndef FILTER_HPP_

#define FILTER_HPP_

#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/random_sample.h>
#include <pcl/filters/uniform_sampling.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/radius_outlier_removal.h>

namespace pcllibs{

namespace filter{



/**
 * @brief 体素网格滤波器
 * @param cloud_in 输入点云
 * @param cloud_out 输出点云
 * @param leaf_size 体素大小
 * @param calc_center 是否计算重心
 * @note calc_center为true时，开启所有数据降采样（包括计算重心），可提升重心计算效率
 * @note 适用于需要保持点云空间结构特征的场景, 大规模点云数据的预处理，减少点云数据量, 点云配准前的预处理,点云特征提取前的降采样
 */
template<typename PointT>
void voxelGridFilter(const typename pcl::PointCloud<PointT>::Ptr &cloud_in, typename pcl::PointCloud<PointT>::Ptr &cloud_out, float leaf_size, bool calc_center)
{
    pcl::VoxelGrid<PointT> voxel_grid_default;
    voxel_grid_default.setInputCloud(cloud_in);
    voxel_grid_default.setLeafSize(leaf_size,leaf_size,leaf_size); // 体素大小
    if(calc_center){
        // 关键：开启所有数据降采样（包括计算重心）
        voxel_grid_default.setDownsampleAllData(true); 
        // 可选：关闭leaf layout保存，提升重心计算效率
        voxel_grid_default.setSaveLeafLayout(false);
    }
    voxel_grid_default.filter(*cloud_out);
}

/**
 * @brief 随机采样降采样
 * @param cloud_in 输入点云
 * @param cloud_out 输出点云
 * @param sampling_ratio 采样比例
 * @note 适用于对计算速度要求较高，对空间结构要求不严格的场景,使用随机采样一致性方法进行下采样,通过sampling_ratio参数控制采样密度,计算效率较高,但可能丢失空间结构信息
 */
template<typename PointT>
void randomSampleFilter(const typename pcl::PointCloud<PointT>::Ptr &cloud_in, typename pcl::PointCloud<PointT>::Ptr &cloud_out, float sampling_ratio){
    unsigned int sample  = cloud_in->size() * sampling_ratio;
    pcl::RandomSample<PointT> random_sample_default;
    random_sample_default.setInputCloud(cloud_in);
    random_sample_default.setSample(sample);
    random_sample_default.filter(*cloud_out);
}

/**
 * @brief 均匀采样降采样
 * @param cloud_in 输入点云
 * @param cloud_out 输出点云
 * @param radius 搜索半径
 * @note 能较好保持空间分布特征
 */
template<typename PointT>
void uniformSampleFilter(const typename pcl::PointCloud<PointT>::Ptr &cloud_in, typename pcl::PointCloud<PointT>::Ptr &cloud_out, float radius){
     pcl::UniformSampling<PointT> uniform_sampling;
     uniform_sampling.setInputCloud(cloud_in);
     uniform_sampling.setRadiusSearch(radius);
     uniform_sampling.filter(*cloud_out);
}


/**
 * @brief 去除点云中的NaN点
 * @param cloud_in 输入点云
 * @param cloud_out 输出点云
 * @note 适用于去除无效点，确保后续处理的数据质量
 */
template<typename PointT>
void filterNaN(const typename pcl::PointCloud<PointT>::Ptr &cloud_in, typename pcl::PointCloud<PointT>::Ptr &cloud_out)
{
    std::vector<int> indices;
    pcl::removeNaNFromPointCloud(*cloud_in, *cloud_out, indices);
}


/**
 * @brief 统计滤波去除离群点
 * @param cloud_in 输入点云
 * @param cloud_out 输出点云
 * @param nb_neighbors 计算平均距离的邻居点数
 * @param std_ratio 标准差倍数阈值 
 * @param filter_nan 是否过滤NaN点(默认true)
 * @note 适用于去除噪声点，保持点云主要结构
 */
template<typename PointT>
void statisticalOutlierRemovalFilter(const typename pcl::PointCloud<PointT>::Ptr &cloud_in, 
                                     typename pcl::PointCloud<PointT>::Ptr &cloud_out,
                                     int nb_neighbors, float std_ratio,bool filter_nan)
{
    pcl::StatisticalOutlierRemoval<PointT> sor;
    if(filter_nan)
    {
        typename pcl::PointCloud<PointT>::Ptr temp_cloud(new pcl::PointCloud<PointT>);
        filterNaN<PointT>(cloud_in, temp_cloud);
        sor.setInputCloud(temp_cloud);
    }else{
        sor.setInputCloud(cloud_in);
    }
    sor.setMeanK(nb_neighbors);          // 设置计算平均距离的邻居点数
    sor.setStddevMulThresh(std_ratio);   // 设置标准差倍数阈值
    sor.filter(*cloud_out);
}

/**
 * @brief 半径滤波去除离群点
 * @param cloud_in 输入点云
 * @param cloud_out 输出点云
 * @param radius 搜索半径
 * @param min_neighbors 在搜索半径内要求的最小邻居点数
 */
template<typename PointT>
void radiusOutlierRemovalFilter(const typename pcl::PointCloud<PointT>::Ptr &cloud_in,
                                 typename pcl::PointCloud<PointT>::Ptr &cloud_out,
                                 float radius, int min_neighbors)
{
        // 初始化半径滤波
        pcl::RadiusOutlierRemoval<PointT> ror;
        ror.setInputCloud(cloud_in);
        ror.setRadiusSearch(radius);           
        ror.setMinNeighborsInRadius(min_neighbors);    
        // ror.setNegative(true); // 可选：反向滤波，仅保留孤立噪点（调试用）
        ror.filter(*cloud_out);
}
    
}// namespace filter
}// namespace pcllibs
#endif // FILTER_HPP_
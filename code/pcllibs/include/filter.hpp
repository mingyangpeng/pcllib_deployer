#ifndef FILTER_HPP_

#define FILTER_HPP_

#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>

namespace pcllibs{

namespace filter{



/**
 * @brief 体素网格滤波器
 * @param cloud_in 输入点云
 * @param cloud_out 输出点云
 * @param leaf_size 体素大小
 * @param calc_center 是否计算重心
 * @note calc_center为true时，开启所有数据降采样（包括计算重心），可提升重心计算效率
 * @note 大规模点云数据的预处理，减少点云数据量, 点云配准前的预处理，提高配准效率, 点云特征提取前的降采样
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

    
}// namespace filter
}// namespace pcllibs
#endif // FILTER_HPP_
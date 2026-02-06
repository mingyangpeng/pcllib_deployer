/**
 * @copyright  Copyright (c) 2026 pengmingyang All Rights Reserved.
 * @file pcllib.hpp
 * @brief 点云工具合集类
 * @author pengmingyang
 */
#ifndef PCLLIB_HPP_
#define PCLLIB_HPP_

#include "filter.hpp"
#include "io.hpp"
#include "logger.h"
#include "register.hpp"
#include "util.hpp"
#include "viewer.hpp"
#include "datastruct.hpp"
namespace pcllibs {
// #define TIME_SCOPE(name) ScopedTimer _timer_##name(name)
#define TIME_SCOPE(name) util::ScopedTimer __timer_instance__(name)

class PCLUTILS {
   public:
    PCLUTILS();
    PCLUTILS(const std::string& log_path);
    ~PCLUTILS();

    // 显示单点云
    template <typename PointT>
    void showPointCloud(const typename pcl::PointCloud<PointT>::Ptr& cloud,
                        const std::string& window_name = "3D Viewer");

    // 显示两幅点云，并显示变换关系
    template <typename PointT>
    void showPointCloud(const typename pcl::PointCloud<PointT>::Ptr& cloud1,
                        const typename pcl::PointCloud<PointT>::Ptr& cloud2,
                        const Eigen::Matrix4f& T1, const Eigen::Matrix4f& T2,
                        const std::string& window_name = "3D Viewer");

    // 显示两幅点云
    template <typename PointT>
    void showPointCloud(const typename pcl::PointCloud<PointT>::Ptr& cloud1,
                        const typename pcl::PointCloud<PointT>::Ptr& cloud2,
                        const std::string& window_name = "3D Viewer");

    // 显示点云法线
    void showNormal(const pcl::PointCloud<pcl::PointNormal>::Ptr& cloud,
                    const std::string& window_name = "3D Viewer",
                    int per_num_show = 5);

    // 显示点云法线
    void showNormal(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
                    const pcl::PointCloud<pcl::Normal>::Ptr& normals,
                    const std::string& window_name = "3D Viewer",
                    int per_num_show = 5);

    // 显示PCA方向  
    template<typename PointT>
    void showPCA(const typename pcl::PointCloud<PointT>::Ptr& cloud,
        const Eigen::Vector4f& centroid,
        const Eigen::Matrix3f& eigenvectors,
        const Eigen::Vector3f& eigenvalues); 
    // 显示OBB包围盒
    void showOBB();


    // 加载点云
    template <typename PointT>
    void loadPointCloud(const std::string& file_name,
                        typename pcl::PointCloud<PointT>::Ptr& cloud,
                        float scale = 1.0f);

    // 保存点云
    template <typename PointT>
    void savePointCloud(const std::string& file_name,
                        const typename pcl::PointCloud<PointT>::Ptr& cloud);

    // 降采样--体素网格滤波
    template <typename PointT>
    void downSampleVoxelGridFilter(
        const typename pcl::PointCloud<PointT>::Ptr& cloud_in,
        typename pcl::PointCloud<PointT>::Ptr& cloud_out, float leaf_size,
        bool calc_center);

    // 降采样--随机采样一致性
    template <typename PointT>
    void downSampleRandomFilter(
        const typename pcl::PointCloud<PointT>::Ptr& cloud_in,
        typename pcl::PointCloud<PointT>::Ptr& cloud_out, float sampling_ratio);

    // 降采样--均匀采样
    template <typename PointT>
    void downSampleUniformFilter(
        const typename pcl::PointCloud<PointT>::Ptr& cloud_in,
        typename pcl::PointCloud<PointT>::Ptr& cloud_out, float radius);

    // 滤波--点云索引过滤
    template <typename PointT>
    void filterIndices(const typename pcl::PointCloud<PointT>::Ptr& cloud_in,
                       typename pcl::PointCloud<PointT>::Ptr& cloud_out,
                       pcl::PointIndices::Ptr& indices, bool is_negative);

    // 滤波--Nan值点过滤
    template <typename PointT>
    void filterNaN(const typename pcl::PointCloud<PointT>::Ptr& cloud_in,
                   typename pcl::PointCloud<PointT>::Ptr& cloud_out);

    // 滤波--统计滤波
    template <typename PointT>
    void filterStatisticalOutlierRemoval(
        const typename pcl::PointCloud<PointT>::Ptr& cloud_in,
        typename pcl::PointCloud<PointT>::Ptr& cloud_out, int mean_k,
        float std_dev_mul_thresh, bool filter_nan = true);

    // 滤波--半径滤波
    template <typename PointT>
    void filterRadiusOutlierRemoval(
        const typename pcl::PointCloud<PointT>::Ptr& cloud_in,
        typename pcl::PointCloud<PointT>::Ptr& cloud_out, float radius,
        int min_neighbors);

    // 滤波--设置滤波条件
    template <typename PointT>
    typename pcl::FieldComparison<PointT>::Ptr filterCondition(
        const std::string& field_name, int val, const std::string& comparison);

    // 滤波--设置滤波条件
    template <typename PointT>
    typename pcl::FieldComparison<PointT>::Ptr filterCondition(
        const std::string& field_name, float val,
        const std::string& comparison);

    // 滤波--条件滤波
    template <typename PointT>
    void filterConditionRemovalFilter(
        const typename pcl::PointCloud<PointT>::Ptr& cloud_in,
        typename pcl::PointCloud<PointT>::Ptr& cloud_out,
        const typename std::vector<typename pcl::FieldComparison<PointT>::Ptr>&
            cond_list,
        const std::string& cond_op, bool is_negative);

    // 滤波--直通滤波
    template <typename PointT>
    void filterPassthrough(
        const typename pcl::PointCloud<PointT>::Ptr& cloud_in,
        typename pcl::PointCloud<PointT>::Ptr& cloud_out,
        const std::string& axis, float min, float max);

    // 滤波--裁剪盒滤波
    template <typename PointT>
    void filterCropBox(const typename pcl::PointCloud<PointT>::Ptr& cloud_in,
                       typename pcl::PointCloud<PointT>::Ptr& cloud_out,
                       float x_min, float x_max, float y_min, float y_max,
                       float z_min, float z_max, bool is_negative);

    // 配准--点到点方法
    template <typename PointT>
    bool ICPPointToPoint(const typename pcl::PointCloud<PointT>::Ptr& cloud_src,
                         const typename pcl::PointCloud<PointT>::Ptr& cloud_tgt,
                         Eigen::Matrix4f& transform, int max_iter,
                         float max_corr_dist, float trans_epsilon,
                         float fitness_epsilon);

};  // class PCLUTILS

}  // namespace pcllibs

#endif  // PCLLIB_HPP_
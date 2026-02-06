/**
 * @copyright  Copyright (c) 2026 pengmingyang All Rights Reserved.
 * @file filter.hpp
 * @brief 点云滤波器
 * @author pengmingyang
 */
#ifndef FILTER_HPP_
#define FILTER_HPP_

#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/random_sample.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/uniform_sampling.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_types.h>

#include <Eigen/Core>
#include <Eigen/SVD>
#include <vector>

#include "datastruct.hpp"
#include "util.hpp"

namespace pcllibs {

namespace filter {

/**
 * @brief 点云索引
 * @param cloud_in 输入点云
 * @param cloud_out 输出点云
 * @param indices 点云索引
 * @param is_negative 是否为负索引
 */
template <typename PointT>
void extractIndices(const typename pcl::PointCloud<PointT>::Ptr& cloud_in,
                    typename pcl::PointCloud<PointT>::Ptr& cloud_out,
                    pcl::PointIndices::Ptr& indices, bool is_negative) {
    pcl::ExtractIndices<PointT> extract_indices;
    extract_indices.setInputCloud(cloud_in);
    extract_indices.setIndices(indices);
    extract_indices.setNegative(is_negative);
    extract_indices.filter(*cloud_out);
}

/**
 * @brief 体素网格滤波器
 * @param cloud_in 输入点云
 * @param cloud_out 输出点云
 * @param leaf_size 体素大小
 * @param calc_center 是否计算重心
 * @note
 * calc_center为true时，开启所有数据降采样（包括计算重心），可提升重心计算效率
 * @note 适用于需要保持点云空间结构特征的场景,
 * 大规模点云数据的预处理，减少点云数据量,
 * 点云配准前的预处理,点云特征提取前的降采样
 */
template <typename PointT>
void voxelGridFilter(const typename pcl::PointCloud<PointT>::Ptr& cloud_in,
                     typename pcl::PointCloud<PointT>::Ptr& cloud_out,
                     float leaf_size, bool calc_center) {
    pcl::VoxelGrid<PointT> voxel_grid_default;
    voxel_grid_default.setInputCloud(cloud_in);
    voxel_grid_default.setLeafSize(leaf_size, leaf_size,
                                   leaf_size);  // 体素大小
    if (calc_center) {
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
 * @note
 * 适用于对计算速度要求较高，对空间结构要求不严格的场景,使用随机采样一致性方法进行下采样,通过sampling_ratio参数控制采样密度,计算效率较高,但可能丢失空间结构信息
 */
template <typename PointT>
void randomSampleFilter(const typename pcl::PointCloud<PointT>::Ptr& cloud_in,
                        typename pcl::PointCloud<PointT>::Ptr& cloud_out,
                        float sampling_ratio) {
    unsigned int sample = cloud_in->size() * sampling_ratio;
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
template <typename PointT>
void uniformSampleFilter(const typename pcl::PointCloud<PointT>::Ptr& cloud_in,
                         typename pcl::PointCloud<PointT>::Ptr& cloud_out,
                         float radius) {
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
template <typename PointT>
void filterNaN(const typename pcl::PointCloud<PointT>::Ptr& cloud_in,
               typename pcl::PointCloud<PointT>::Ptr& cloud_out) {
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
template <typename PointT>
void statisticalOutlierRemovalFilter(
    const typename pcl::PointCloud<PointT>::Ptr& cloud_in,
    typename pcl::PointCloud<PointT>::Ptr& cloud_out, int nb_neighbors,
    float std_ratio, bool filter_nan) {
    pcl::StatisticalOutlierRemoval<PointT> sor;
    if (filter_nan) {
        typename pcl::PointCloud<PointT>::Ptr temp_cloud(
            new pcl::PointCloud<PointT>);
        filterNaN<PointT>(cloud_in, temp_cloud);
        sor.setInputCloud(temp_cloud);
    } else {
        sor.setInputCloud(cloud_in);
    }
    sor.setMeanK(nb_neighbors);  // 设置计算平均距离的邻居点数
    sor.setStddevMulThresh(std_ratio);  // 设置标准差倍数阈值
    sor.filter(*cloud_out);
}

/**
 * @brief 半径滤波去除离群点
 * @param cloud_in 输入点云
 * @param cloud_out 输出点云
 * @param radius 搜索半径
 * @param min_neighbors 在搜索半径内要求的最小邻居点数
 */
template <typename PointT>
void radiusOutlierRemovalFilter(
    const typename pcl::PointCloud<PointT>::Ptr& cloud_in,
    typename pcl::PointCloud<PointT>::Ptr& cloud_out, float radius,
    int min_neighbors) {
    // 初始化半径滤波
    pcl::RadiusOutlierRemoval<PointT> ror;
    ror.setInputCloud(cloud_in);
    ror.setRadiusSearch(radius);
    ror.setMinNeighborsInRadius(min_neighbors);
    // ror.setNegative(true); // 可选：反向滤波，仅保留孤立噪点（调试用）
    ror.filter(*cloud_out);
}

/**
 * @brief 条件设置
 * @param field_name 字段名
 (如x,y,z,normal_x,normal_y,normal_z，curvature，intensity...)
 * @param val 比较值
 * @param comparison 比较操作符(>, >=, <, <=, =, !=)
 * @note    pcl::ComparisonOps::GT	>
            pcl::ComparisonOps::GE	>=
            pcl::ComparisonOps::LT	<
            pcl::ComparisonOps::LE	<=
            pcl::ComparisonOps::EQ	=
*/
template <typename PointT>
typename pcl::FieldComparison<PointT>::Ptr removalCondition(
    const std::string& field_name, float val, const std::string& comparison) {
    pcl::ComparisonOps::CompareOp op;
    if (comparison == ">") {
        op = pcl::ComparisonOps::GT;
    } else if (comparison == ">=") {
        op = pcl::ComparisonOps::GE;
    } else if (comparison == "<") {
        op = pcl::ComparisonOps::LT;
    } else if (comparison == "<=") {
        op = pcl::ComparisonOps::LE;
    } else if (comparison == "=") {
        op = pcl::ComparisonOps::EQ;
    } else {
        throw std::runtime_error("Invalid comparison operator");
        exit(-1);
    }
    typename pcl::FieldComparison<PointT>::Ptr cond(
        new pcl::FieldComparison<PointT>(field_name, op, val));
    return cond;
}

/**
 * @brief 条件设置
 * @param field_name 字段名
 (如x,y,z,normal_x,normal_y,normal_z，curvature，intensity...)
 * @param val 比较值
 * @param comparison 比较操作符(>, >=, <, <=, =, !=)
 * @note    pcl::ComparisonOps::GT	>
            pcl::ComparisonOps::GE	>=
            pcl::ComparisonOps::LT	<
            pcl::ComparisonOps::LE	<=
            pcl::ComparisonOps::EQ	=
*/
template <typename PointT>
typename pcl::FieldComparison<PointT>::Ptr removalCondition(
    const std::string& field_name, int val, const std::string& comparison) {
    pcl::ComparisonOps::CompareOp op;
    if (comparison == ">") {
        op = pcl::ComparisonOps::GT;
    } else if (comparison == ">=") {
        op = pcl::ComparisonOps::GE;
    } else if (comparison == "<") {
        op = pcl::ComparisonOps::LT;
    } else if (comparison == "<=") {
        op = pcl::ComparisonOps::LE;
    } else if (comparison == "=") {
        op = pcl::ComparisonOps::EQ;
    } else {
        throw std::runtime_error("Invalid comparison operator");
        exit(-1);
    }
    typename pcl::FieldComparison<PointT>::Ptr cond(
        new pcl::FieldComparison<PointT>(field_name, op, val));
    return cond;
}

/**
 * @brief 条件滤波,支持多条件组合
 * @param cloud_in 输入点云
 * @param cloud_out 输出点云
 * @param cond_list 条件list
 * @param cond_op 条件操作符 (AND, OR)
 * @param is_negative 是否反向滤波
 *
 */
template <typename PointT>
void conditionRemovalFilter(
    const typename pcl::PointCloud<PointT>::Ptr& cloud_in,
    typename pcl::PointCloud<PointT>::Ptr& cloud_out,
    const typename std::vector<typename pcl::FieldComparison<PointT>::Ptr>&
        cond_list,
    const std::string& cond_op, bool is_negative) {
    typename pcl::ConditionBase<PointT>::Ptr conds;
    if (cond_op == "and") {
        conds.reset(new pcl::ConditionAnd<PointT>());
    } else {
        conds.reset(new pcl::ConditionOr<PointT>());
    }
    for (auto cond : cond_list) {
        conds->addComparison(cond);
    }

    pcl::ConditionalRemoval<PointT> cond_rem(true);
    cond_rem.setCondition(conds);
    cond_rem.setInputCloud(cloud_in);
    cond_rem.setKeepOrganized(true);
    cond_rem.filter(*cloud_out);

    if (is_negative) {
        pcl::PointIndices::Ptr removed_indices(new pcl::PointIndices());
        cond_rem.getRemovedIndices(*removed_indices);
        cloud_out->clear();
        extractIndices<PointT>(cloud_in, cloud_out, removed_indices, false);
    }
}

/**
 * @brief 裁剪盒滤波
 * @param cloud_in 输入点云
 * @param cloud_out 输出点云
 * @param x_min x轴最小值
 * @param x_max x轴最大值
 * @param y_min y轴最小值
 * @param y_max y轴最大值
 * @param z_min z轴最小值
 * @param z_max z轴最大值
 * @param is_negative 是否反向滤波
 */
template <typename PointT>
void cropBoxFilter(const typename pcl::PointCloud<PointT>::Ptr& cloud_in,
                   typename pcl::PointCloud<PointT>::Ptr& cloud_out,
                   float x_min, float x_max, float y_min, float y_max,
                   float z_min, float z_max, bool is_negative = false) {
    pcl::CropBox<PointT> crop_filter;
    crop_filter.setInputCloud(cloud_in);
    crop_filter.setMin(Eigen::Vector4f(x_min, y_min, z_min, 1.0f));
    crop_filter.setMax(Eigen::Vector4f(x_max, y_max, z_max, 1.0f));
    crop_filter.setNegative(is_negative);
    crop_filter.filter(*cloud_out);
}

/**
 * @brief 直通滤波
 * @param cloud_in 输入点云
 * @param cloud_out 输出点云
 * @param axis 滤波字段名
 * @param val_min 最小值
 * @param val_max 最大值
 */
template <typename PointT>
void passThroughFilter(const typename pcl::PointCloud<PointT>::Ptr& cloud_in,
                       typename pcl::PointCloud<PointT>::Ptr& cloud_out,
                       const std::string& axis, float val_min, float val_max) {
    pcl::PassThrough<PointT> pass;
    pass.setInputCloud(cloud_in);
    pass.setFilterFieldName(axis);
    pass.setFilterLimits(val_min, val_max);
    pass.filter(*cloud_out);
}

void calcBoxParamsFrom8Vertices(const std::vector<Eigen::Vector3f>& vertices,
                                Eigen::Vector3f& center, Eigen::Vector3f& size,
                                Eigen::Matrix3f& rotation);
/**
 * @brief 基于8顶点包围盒的裁剪盒滤波
 * @param input_cloud 输入点云
 * @param box_vertices 包围盒8顶点
 * @param output_cloud 输出点云
 * @param is_negative 是否反向滤波
 */
template<typename PointT>
void cropPointCloudBy8Vertices111(const typename pcl::PointCloud<PointT>::Ptr& input_cloud,
                               const std::vector<Eigen::Vector3f>& box_vertices,
                               typename pcl::PointCloud<PointT>::Ptr& output_cloud,
                               bool is_negative = false) { 

    // 2. 从8顶点解算：包围盒中心 + 3×3旋转矩阵 + 三轴尺寸
    Eigen::MatrixXf vertices_mat(3, 8);
    for (int i = 0; i < 8; ++i) vertices_mat.col(i) = box_vertices[i];

    Eigen::Vector3f box_center = vertices_mat.rowwise().mean();
    Eigen::MatrixXf centered_vertices = vertices_mat.colwise() - box_center;

    // 2. 修正SVD分解和旋转矩阵计算
    Eigen::JacobiSVD<Eigen::MatrixXf> svd(centered_vertices, Eigen::ComputeFullV);
    Eigen::Matrix3f box_rot_mat = svd.matrixV(); // 修正：不转置
    if (box_rot_mat.determinant() < 0) { // 确保右手坐标系
        box_rot_mat.col(2) *= -1;
    }

    // 3. 计算盒子尺寸
    Eigen::MatrixXf rotated_vertices = box_rot_mat * centered_vertices;
    Eigen::Vector3f min_pt = rotated_vertices.rowwise().minCoeff();
    Eigen::Vector3f max_pt = rotated_vertices.rowwise().maxCoeff();
    Eigen::Vector3f box_size = max_pt - min_pt;
    box_size = box_size.array().max(1e-6f);
    Eigen::Vector3f half_size = box_size / 2.0f;

    // 4. 设置CropBox滤波器
    pcl::CropBox<PointT> crop_box;
    crop_box.setInputCloud(input_cloud);
    crop_box.setNegative(is_negative);

    // 5. 修正变换矩阵设置顺序
    Eigen::Affine3f box_transform = Eigen::Affine3f::Identity();
    box_transform.translate(box_center); // 先平移
    box_transform.rotate(box_rot_mat);    // 后旋转
    crop_box.setTransform(box_transform);

    // 6. 设置裁剪范围（修正w分量）
    crop_box.setMin(Eigen::Vector4f(-half_size.x(), -half_size.y(), -half_size.z(), 1.0f));
    crop_box.setMax(Eigen::Vector4f(half_size.x(), half_size.y(), half_size.z(), 1.0f));

    // 7. 执行滤波
    crop_box.filter(*output_cloud);

    // 简洁日志：裁剪结果统计
    PCL_INFO("裁剪完成：原始点云[%lu] → 裁剪后[%lu]\n", input_cloud->size(), output_cloud->size());
}

template<typename PointT>
void cropPointCloudBy8Vertices(const typename pcl::PointCloud<PointT>::Ptr& cloud_in,
    const std::vector<Eigen::Vector3f>& box_8_vertices,
    typename pcl::PointCloud<PointT>::Ptr& cloud_out,
    float epsilon = 1e-6f)
{
 

// ===================== 步骤2：将8个顶点转为点云，拟合OBB核心参数 =====================
  
typename pcl::PointCloud<PointT>::Ptr box_cloud(new pcl::PointCloud<PointT>);
box_cloud->resize(8);
for (int i = 0; i < 8; ++i) {
    PointT pt;
    pt.x = box_8_vertices[i].x();
    pt.y = box_8_vertices[i].y();
    pt.z = box_8_vertices[i].z();
    box_cloud->at(i) = pt;
}

OBBDate obb;
util::calcOBB<PointT>(box_cloud, obb);

// 拟合OBB（中心、旋转矩阵、三维尺寸）
// pcl::MomentOfInertiaEstimation<PointT> moe;
// moe.setInputCloud(box_cloud);
// moe.compute();
// PointT obb_center;
// Eigen::Matrix3f obb_rot;
// Eigen::Vector3f obb_size;

PointT obb_center;
obb_center.x = obb.center.x();
obb_center.y = obb.center.y();
obb_center.z = obb.center.z();

Eigen::Matrix3f obb_rot = obb.axes;  // 修正：使用axes而非rotation
Eigen::Vector3f obb_size = obb.extents * 2.0f;  // 修正：extents是半尺寸，需乘以2

// 校验OBB尺寸有效性
if (obb_size.x() < epsilon || obb_size.y() < epsilon || obb_size.z() < epsilon) {
    PCL_ERROR("OBB拟合失败：8个顶点构成无效包围盒（如共面/重合）！\n");
    return;
}

// ===================== 步骤3：计算OBB局部系筛选阈值 =====================
// 注意：现在obb_size已经是全尺寸
float x_thresh = obb.extents.x() - epsilon;  // 直接使用extents
float y_thresh = obb.extents.y() - epsilon;
float z_thresh = obb.extents.z() - epsilon;

// ===================== 步骤4：遍历点云，精准筛选OBB内的点 =====================
cloud_out->clear();
cloud_out->reserve(cloud_in->size());

for (const auto& p : *cloud_in) {
    // 坐标变换：世界系 → OBB局部系（平移+旋转）
    Eigen::Vector3f p_w(p.x, p.y, p.z);
    Eigen::Vector3f p_trans = p_w - Eigen::Vector3f(obb_center.x, obb_center.y, obb_center.z);
    Eigen::Vector3f p_local = obb_rot.transpose() * p_trans;

    // 局部系中OBB轴对齐，判断点是否在盒内
    if (std::fabs(p_local.x()) <= x_thresh &&
        std::fabs(p_local.y()) <= y_thresh &&
        std::fabs(p_local.z()) <= z_thresh) {
        cloud_out->push_back(p);
    }
}


// ===================== 步骤5：打印调试信息 =====================
std::cout << "===== OBB裁剪结果 =====" << std::endl;
std::cout << "OBB中心：(" << obb_center.x << "," << obb_center.y << "," << obb_center.z << ")\n";
std::cout << "原始点云数：" << cloud_in->size() << " | 盒内点云数：" << cloud_out->size() << "\n";
}


//////////////////////////////////////////////////////////////////////////
}  // namespace filter
}  // namespace pcllibs
#endif  // FILTER_HPP_
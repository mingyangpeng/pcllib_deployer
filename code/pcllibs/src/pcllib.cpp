#include "pcllib.hpp"

namespace pcllibs {

PCLUTILS::PCLUTILS() {
    vtkObject::GlobalWarningDisplayOff();  // 关闭vtk的警告信息
    // 构造函数实现
}
PCLUTILS::PCLUTILS(const std::string& log_path) {
    vtkObject::GlobalWarningDisplayOff();  // 关闭vtk的警告信息
    // 构造函数实现
    util::createFolder(log_path, false);  // 创建文件夹
    LogConfig conf2 = {
        .level = "trace",
        .path = log_path,
        .size = 5 * 1024 * 1024,
        .count = 10,
        .type = 2,  // 1: 输出到文件  2: 输出到文件和控制台 其他: 输出到控制台
    };
    INITLOG(conf2);
}

PCLUTILS::~PCLUTILS() {
    // 析构函数实现
}

// pcllib.cpp
template <typename PointT>
void PCLUTILS::showPointCloud(
    const typename pcl::PointCloud<PointT>::Ptr& cloud,
    const std::string& window_name) {
    viewer::show_1<PointT>(cloud, window_name);  // 调用viewer.hpp中的函数
}

template void PCLUTILS::showPointCloud<pcl::PointXYZ>(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
    const std::string& window_name);
template void PCLUTILS::showPointCloud<pcl::PointXYZI>(
    const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud,
    const std::string& window_name);
template void PCLUTILS::showPointCloud<pcl::PointXYZRGB>(
    const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud,
    const std::string& window_name);
template void PCLUTILS::showPointCloud<pcl::PointNormal>(
    const pcl::PointCloud<pcl::PointNormal>::Ptr& cloud,
    const std::string& window_name);

template <typename PointT>
void PCLUTILS::showPointCloud(
    const typename pcl::PointCloud<PointT>::Ptr& cloud1,
    const typename pcl::PointCloud<PointT>::Ptr& cloud2,
    const std::string& window_name) {
    viewer::show_2<PointT>(cloud1, cloud2,
                           window_name);  // 调用viewer.hpp中的函数
}
template void PCLUTILS::showPointCloud<pcl::PointXYZ>(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr&,
    const pcl::PointCloud<pcl::PointXYZ>::Ptr&, const std::string&);
template void PCLUTILS::showPointCloud<pcl::PointXYZI>(
    const pcl::PointCloud<pcl::PointXYZI>::Ptr&,
    const pcl::PointCloud<pcl::PointXYZI>::Ptr&, const std::string&);
template void PCLUTILS::showPointCloud<pcl::PointXYZRGB>(
    const pcl::PointCloud<pcl::PointXYZRGB>::Ptr&,
    const pcl::PointCloud<pcl::PointXYZRGB>::Ptr&, const std::string&);
template void PCLUTILS::showPointCloud<pcl::PointNormal>(
    const pcl::PointCloud<pcl::PointNormal>::Ptr&,
    const pcl::PointCloud<pcl::PointNormal>::Ptr&, const std::string&);
template <typename PointT>
void PCLUTILS::showPointCloud(
    const typename pcl::PointCloud<PointT>::Ptr& cloud1,
    const typename pcl::PointCloud<PointT>::Ptr& cloud2,
    const Eigen::Matrix4f& T1, const Eigen::Matrix4f& T2,
    const std::string& window_name) {
    // 调用viewer.hpp中的函数
    viewer::show_2<PointT>(cloud1, cloud2, T1, T2, window_name);
}
template void PCLUTILS::showPointCloud<pcl::PointXYZ>(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr&,
    const pcl::PointCloud<pcl::PointXYZ>::Ptr&, const Eigen::Matrix4f&,
    const Eigen::Matrix4f&, const std::string&);
template void PCLUTILS::showPointCloud<pcl::PointXYZI>(
    const pcl::PointCloud<pcl::PointXYZI>::Ptr&,
    const pcl::PointCloud<pcl::PointXYZI>::Ptr&, const Eigen::Matrix4f&,
    const Eigen::Matrix4f&, const std::string&);
template void PCLUTILS::showPointCloud<pcl::PointXYZRGB>(
    const pcl::PointCloud<pcl::PointXYZRGB>::Ptr&,
    const pcl::PointCloud<pcl::PointXYZRGB>::Ptr&, const Eigen::Matrix4f&,
    const Eigen::Matrix4f&, const std::string&);
template void PCLUTILS::showPointCloud<pcl::PointNormal>(
    const pcl::PointCloud<pcl::PointNormal>::Ptr&,
    const pcl::PointCloud<pcl::PointNormal>::Ptr&, const Eigen::Matrix4f&,
    const Eigen::Matrix4f&, const std::string&);

void PCLUTILS::showNormal(const pcl::PointCloud<pcl::PointNormal>::Ptr& cloud,
                          const std::string& window_name, int per_num_show) {
    // 调用viewer.hpp中的函数
    viewer::showNormal(cloud, window_name, per_num_show);
}

void PCLUTILS::showNormal(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
                          const pcl::PointCloud<pcl::Normal>::Ptr& normals,
                          const std::string& window_name, int per_num_show) {
    // 调用viewer.hpp中的函数
    viewer::showNormal(cloud, normals, window_name, per_num_show);
}

template <typename PointT>
void PCLUTILS::loadPointCloud(const std::string& file_name,
                              typename pcl::PointCloud<PointT>::Ptr& cloud,
                              float scale) {
    // 调用io.hpp中的函数
    io::loadPointCloud<PointT>(file_name, cloud, scale);
}

template void PCLUTILS::loadPointCloud<pcl::PointXYZ>(
    const std::string& file_name, pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
    float scale);
template void PCLUTILS::loadPointCloud<pcl::PointXYZI>(
    const std::string& file_name, pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud,
    float scale);
template void PCLUTILS::loadPointCloud<pcl::PointXYZRGB>(
    const std::string& file_name, pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud,
    float scale);
template void PCLUTILS::loadPointCloud<pcl::PointNormal>(
    const std::string& file_name, pcl::PointCloud<pcl::PointNormal>::Ptr& cloud,
    float scale);

template <typename PointT>
void PCLUTILS::savePointCloud(
    const std::string& file_name,
    const typename pcl::PointCloud<PointT>::Ptr& cloud) {
    io::savePointCloud<PointT>(file_name, cloud);  // 调用util.hpp中的函数
}
template void PCLUTILS::savePointCloud<pcl::PointXYZ>(
    const std::string& file_name,
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud);
template void PCLUTILS::savePointCloud<pcl::PointXYZI>(
    const std::string& file_name,
    const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud);
template void PCLUTILS::savePointCloud<pcl::PointXYZRGB>(
    const std::string& file_name,
    const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud);
template void PCLUTILS::savePointCloud<pcl::PointNormal>(
    const std::string& file_name,
    const pcl::PointCloud<pcl::PointNormal>::Ptr& cloud);

template <typename PointT>
void PCLUTILS::downSampleVoxelGridFilter(
    const typename pcl::PointCloud<PointT>::Ptr& cloud_in,
    typename pcl::PointCloud<PointT>::Ptr& cloud_out, float leaf_size,
    bool calc_center) {
    filter::voxelGridFilter<PointT>(cloud_in, cloud_out, leaf_size,
                                    calc_center);  // 调用filter.hpp中的函数
}
template void PCLUTILS::downSampleVoxelGridFilter<pcl::PointXYZ>(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_in,
    pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_out, float leaf_size,
    bool calc_center);
template void PCLUTILS::downSampleVoxelGridFilter<pcl::PointXYZI>(
    const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud_in,
    pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud_out, float leaf_size,
    bool calc_center);
template void PCLUTILS::downSampleVoxelGridFilter<pcl::PointXYZRGB>(
    const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud_in,
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud_out, float leaf_size,
    bool calc_center);
template void PCLUTILS::downSampleVoxelGridFilter<pcl::PointNormal>(
    const pcl::PointCloud<pcl::PointNormal>::Ptr& cloud_in,
    pcl::PointCloud<pcl::PointNormal>::Ptr& cloud_out, float leaf_size,
    bool calc_center);

// 降采样--均匀降采样
template <typename PointT>
void PCLUTILS::downSampleUniformFilter(
    const typename pcl::PointCloud<PointT>::Ptr& cloud_in,
    typename pcl::PointCloud<PointT>::Ptr& cloud_out, float radius) {
    filter::uniformSampleFilter<PointT>(cloud_in, cloud_out,
                                        radius);  // 调用filter.hpp中的函数
}
template void PCLUTILS::downSampleUniformFilter<pcl::PointXYZ>(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_in,
    pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_out, float radius);
template void PCLUTILS::downSampleUniformFilter<pcl::PointXYZI>(
    const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud_in,
    pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud_out, float radius);
template void PCLUTILS::downSampleUniformFilter<pcl::PointXYZRGB>(
    const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud_in,
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud_out, float radius);
template void PCLUTILS::downSampleUniformFilter<pcl::PointNormal>(
    const pcl::PointCloud<pcl::PointNormal>::Ptr& cloud_in,
    pcl::PointCloud<pcl::PointNormal>::Ptr& cloud_out, float radius);

// 降采样--随机采样一致性
template <typename PointT>
void PCLUTILS::downSampleRandomFilter(
    const typename pcl::PointCloud<PointT>::Ptr& cloud_in,
    typename pcl::PointCloud<PointT>::Ptr& cloud_out, float sampling_ratio) {
    filter::randomSampleFilter<PointT>(
        cloud_in, cloud_out, sampling_ratio);  // 调用filter.hpp中的函数
}
template void PCLUTILS::downSampleRandomFilter<pcl::PointXYZ>(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_in,
    pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_out, float sampling_ratio);
template void PCLUTILS::downSampleRandomFilter<pcl::PointXYZI>(
    const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud_in,
    pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud_out, float sampling_ratio);
template void PCLUTILS::downSampleRandomFilter<pcl::PointXYZRGB>(
    const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud_in,
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud_out, float sampling_ratio);
template void PCLUTILS::downSampleRandomFilter<pcl::PointNormal>(
    const pcl::PointCloud<pcl::PointNormal>::Ptr& cloud_in,
    pcl::PointCloud<pcl::PointNormal>::Ptr& cloud_out, float sampling_ratio);

// 滤波--点云索引过滤
template <typename PointT>
void PCLUTILS::filterIndices(
    const typename pcl::PointCloud<PointT>::Ptr& cloud_in,
    typename pcl::PointCloud<PointT>::Ptr& cloud_out,
    pcl::PointIndices::Ptr& indices, bool is_negative) {
    filter::extractIndices<PointT>(cloud_in, cloud_out, indices,
                                   is_negative);  // 调用filter.hpp中的函数
}
template void PCLUTILS::filterIndices<pcl::PointXYZ>(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_in,
    pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_out,
    pcl::PointIndices::Ptr& indices, bool is_negative);
template void PCLUTILS::filterIndices<pcl::PointXYZI>(
    const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud_in,
    pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud_out,
    pcl::PointIndices::Ptr& indices, bool is_negative);
template void PCLUTILS::filterIndices<pcl::PointXYZRGB>(
    const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud_in,
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud_out,
    pcl::PointIndices::Ptr& indices, bool is_negative);
template void PCLUTILS::filterIndices<pcl::PointNormal>(
    const pcl::PointCloud<pcl::PointNormal>::Ptr& cloud_in,
    pcl::PointCloud<pcl::PointNormal>::Ptr& cloud_out,
    pcl::PointIndices::Ptr& indices, bool is_negative);

// 滤波--Nan值点过滤
template <typename PointT>
void PCLUTILS::filterNaN(const typename pcl::PointCloud<PointT>::Ptr& cloud_in,
                         typename pcl::PointCloud<PointT>::Ptr& cloud_out) {
    filter::filterNaN<PointT>(cloud_in, cloud_out);  // 调用filter.hpp中的函数
}
template void PCLUTILS::filterNaN<pcl::PointXYZ>(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_in,
    pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_out);
template void PCLUTILS::filterNaN<pcl::PointXYZI>(
    const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud_in,
    pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud_out);
template void PCLUTILS::filterNaN<pcl::PointXYZRGB>(
    const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud_in,
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud_out);
template void PCLUTILS::filterNaN<pcl::PointNormal>(
    const pcl::PointCloud<pcl::PointNormal>::Ptr& cloud_in,
    pcl::PointCloud<pcl::PointNormal>::Ptr& cloud_out);

// 滤波-- 统计滤波
template <typename PointT>
void PCLUTILS::filterStatisticalOutlierRemoval(
    const typename pcl::PointCloud<PointT>::Ptr& cloud_in,
    typename pcl::PointCloud<PointT>::Ptr& cloud_out, int mean_k,
    float std_dev_mul_thresh, bool filter_nan) {
    filter::statisticalOutlierRemovalFilter<PointT>(
        cloud_in, cloud_out, mean_k, std_dev_mul_thresh,
        filter_nan);  // 调用filter.hpp中的函数
}
template void PCLUTILS::filterStatisticalOutlierRemoval<pcl::PointXYZ>(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_in,
    pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_out, int mean_k,
    float std_dev_mul_thresh, bool filter_nan);
template void PCLUTILS::filterStatisticalOutlierRemoval<pcl::PointXYZI>(
    const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud_in,
    pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud_out, int mean_k,
    float std_dev_mul_thresh, bool filter_nan);
template void PCLUTILS::filterStatisticalOutlierRemoval<pcl::PointXYZRGB>(
    const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud_in,
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud_out, int mean_k,
    float std_dev_mul_thresh, bool filter_nan);
template void PCLUTILS::filterStatisticalOutlierRemoval<pcl::PointNormal>(
    const pcl::PointCloud<pcl::PointNormal>::Ptr& cloud_in,
    pcl::PointCloud<pcl::PointNormal>::Ptr& cloud_out, int mean_k,
    float std_dev_mul_thresh, bool filter_nan);

// 滤波--半径滤波
template <typename PointT>
void PCLUTILS::filterRadiusOutlierRemoval(
    const typename pcl::PointCloud<PointT>::Ptr& cloud_in,
    typename pcl::PointCloud<PointT>::Ptr& cloud_out, float radius,
    int min_neighbors) {
    filter::radiusOutlierRemovalFilter<PointT>(
        cloud_in, cloud_out, radius, min_neighbors);  // 调用filter.hpp中的函数
}
template void PCLUTILS::filterRadiusOutlierRemoval<pcl::PointXYZ>(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_in,
    pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_out, float radius,
    int min_neighbors);
template void PCLUTILS::filterRadiusOutlierRemoval<pcl::PointXYZI>(
    const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud_in,
    pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud_out, float radius,
    int min_neighbors);
template void PCLUTILS::filterRadiusOutlierRemoval<pcl::PointXYZRGB>(
    const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud_in,
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud_out, float radius,
    int min_neighbors);
template void PCLUTILS::filterRadiusOutlierRemoval<pcl::PointNormal>(
    const pcl::PointCloud<pcl::PointNormal>::Ptr& cloud_in,
    pcl::PointCloud<pcl::PointNormal>::Ptr& cloud_out, float radius,
    int min_neighbors);

// 滤波--设置滤波条件
template <typename PointT>
typename pcl::FieldComparison<PointT>::Ptr PCLUTILS::filterCondition(
    const std::string& field_name, int val, const std::string& comparison) {
    return filter::removalCondition<PointT>(
        field_name, val, comparison);  // 调用filter.hpp中的函数
}
template pcl::FieldComparison<pcl::PointXYZ>::Ptr
PCLUTILS::filterCondition<pcl::PointXYZ>(const std::string& field_name, int val,
                                         const std::string& comparison);
template pcl::FieldComparison<pcl::PointXYZI>::Ptr
PCLUTILS::filterCondition<pcl::PointXYZI>(const std::string& field_name,
                                          int val,
                                          const std::string& comparison);
template pcl::FieldComparison<pcl::PointXYZRGB>::Ptr
PCLUTILS::filterCondition<pcl::PointXYZRGB>(const std::string& field_name,
                                            int val,
                                            const std::string& comparison);
template pcl::FieldComparison<pcl::PointNormal>::Ptr
PCLUTILS::filterCondition<pcl::PointNormal>(const std::string& field_name,
                                            int val,
                                            const std::string& comparison);

// 滤波--设置滤波条件
template <typename PointT>
typename pcl::FieldComparison<PointT>::Ptr PCLUTILS::filterCondition(
    const std::string& field_name, float val, const std::string& comparison) {
    return filter::removalCondition<PointT>(
        field_name, val, comparison);  // 调用filter.hpp中的函数
}
template pcl::FieldComparison<pcl::PointXYZ>::Ptr
PCLUTILS::filterCondition<pcl::PointXYZ>(const std::string& field_name,
                                         float val,
                                         const std::string& comparison);
template pcl::FieldComparison<pcl::PointXYZI>::Ptr
PCLUTILS::filterCondition<pcl::PointXYZI>(const std::string& field_name,
                                          float val,
                                          const std::string& comparison);
template pcl::FieldComparison<pcl::PointXYZRGB>::Ptr
PCLUTILS::filterCondition<pcl::PointXYZRGB>(const std::string& field_name,
                                            float val,
                                            const std::string& comparison);
template pcl::FieldComparison<pcl::PointNormal>::Ptr
PCLUTILS::filterCondition<pcl::PointNormal>(const std::string& field_name,
                                            float val,
                                            const std::string& comparison);

// 滤波--条件滤波
template <typename PointT>
void PCLUTILS::filterConditionRemovalFilter(
    const typename pcl::PointCloud<PointT>::Ptr& cloud_in,
    typename pcl::PointCloud<PointT>::Ptr& cloud_out,
    const typename std::vector<typename pcl::FieldComparison<PointT>::Ptr>&
        cond_list,
    const std::string& cond_op, bool is_negative) {
    filter::conditionRemovalFilter<PointT>(
        cloud_in, cloud_out, cond_list, cond_op,
        is_negative);  // 调用filter.hpp中的函数
}
template void PCLUTILS::filterConditionRemovalFilter<pcl::PointXYZ>(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_in,
    pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_out,
    const typename std::vector<pcl::FieldComparison<pcl::PointXYZ>::Ptr>&
        cond_list,
    const std::string& cond_op, bool is_negative);
template void PCLUTILS::filterConditionRemovalFilter<pcl::PointXYZI>(
    const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud_in,
    pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud_out,
    const typename std::vector<pcl::FieldComparison<pcl::PointXYZI>::Ptr>&
        cond_list,
    const std::string& cond_op, bool is_negative);
template void PCLUTILS::filterConditionRemovalFilter<pcl::PointXYZRGB>(
    const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud_in,
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud_out,
    const typename std::vector<pcl::FieldComparison<pcl::PointXYZRGB>::Ptr>&
        cond_list,
    const std::string& cond_op, bool is_negative);
template void PCLUTILS::filterConditionRemovalFilter<pcl::PointNormal>(
    const pcl::PointCloud<pcl::PointNormal>::Ptr& cloud_in,
    pcl::PointCloud<pcl::PointNormal>::Ptr& cloud_out,
    const typename std::vector<pcl::FieldComparison<pcl::PointNormal>::Ptr>&
        cond_list,
    const std::string& cond_op, bool is_negative);

// 滤波--直通滤波
template <typename PointT>
void PCLUTILS::filterPassthrough(
    const typename pcl::PointCloud<PointT>::Ptr& cloud_in,
    typename pcl::PointCloud<PointT>::Ptr& cloud_out, const std::string& axis,
    float min, float max) {
    filter::passThroughFilter<PointT>(cloud_in, cloud_out, axis, min,
                                      max);  // 调用filter.hpp中的函数
}
template void PCLUTILS::filterPassthrough<pcl::PointXYZ>(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_in,
    pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_out, const std::string& axis,
    float min, float max);
template void PCLUTILS::filterPassthrough<pcl::PointXYZI>(
    const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud_in,
    pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud_out, const std::string& axis,
    float min, float max);
template void PCLUTILS::filterPassthrough<pcl::PointXYZRGB>(
    const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud_in,
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud_out, const std::string& axis,
    float min, float max);
template void PCLUTILS::filterPassthrough<pcl::PointNormal>(
    const pcl::PointCloud<pcl::PointNormal>::Ptr& cloud_in,
    pcl::PointCloud<pcl::PointNormal>::Ptr& cloud_out, const std::string& axis,
    float min, float max);

// 滤波--裁剪盒滤波
template <typename PointT>
void PCLUTILS::filterCropBox(
    const typename pcl::PointCloud<PointT>::Ptr& cloud_in,
    typename pcl::PointCloud<PointT>::Ptr& cloud_out, float x_min, float x_max,
    float y_min, float y_max, float z_min, float z_max, bool is_negative) {
    filter::cropBoxFilter<PointT>(cloud_in, cloud_out, x_min, x_max, y_min,
                                  y_max, z_min, z_max,
                                  is_negative);  // 调用filter.hpp中的函数
}
template void PCLUTILS::filterCropBox<pcl::PointXYZ>(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_in,
    pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_out, float x_min, float x_max,
    float y_min, float y_max, float z_min, float z_max, bool is_negative);
template void PCLUTILS::filterCropBox<pcl::PointXYZI>(
    const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud_in,
    pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud_out, float x_min, float x_max,
    float y_min, float y_max, float z_min, float z_max, bool is_negative);
template void PCLUTILS::filterCropBox<pcl::PointXYZRGB>(
    const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud_in,
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud_out, float x_min, float x_max,
    float y_min, float y_max, float z_min, float z_max, bool is_negative);
template void PCLUTILS::filterCropBox<pcl::PointNormal>(
    const pcl::PointCloud<pcl::PointNormal>::Ptr& cloud_in,
    pcl::PointCloud<pcl::PointNormal>::Ptr& cloud_out, float x_min, float x_max,
    float y_min, float y_max, float z_min, float z_max, bool is_negative);

// 配准--点到点方法
template <typename PointT>
bool PCLUTILS::ICPPointToPoint(
    const typename pcl::PointCloud<PointT>::Ptr& cloud_src,
    const typename pcl::PointCloud<PointT>::Ptr& cloud_tgt,
    Eigen::Matrix4f& transform, int max_iter, float max_corr_dist,
    float trans_epsilon, float fitness_epsilon) {
    bool res = regist::icpPToPoint<PointT>(cloud_src, cloud_tgt, transform,
                                           max_iter, max_corr_dist,
                                           trans_epsilon, fitness_epsilon);
    return res;
}
template bool PCLUTILS::ICPPointToPoint<pcl::PointXYZ>(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_src,
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_tgt,
    Eigen::Matrix4f& transform, int max_iter, float max_corr_dist,
    float trans_epsilon, float fitness_epsilon);
template bool PCLUTILS::ICPPointToPoint<pcl::PointXYZI>(
    const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud_src,
    const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud_tgt,
    Eigen::Matrix4f& transform, int max_iter, float max_corr_dist,
    float trans_epsilon, float fitness_epsilon);
template bool PCLUTILS::ICPPointToPoint<pcl::PointXYZRGB>(
    const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud_src,
    const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud_tgt,
    Eigen::Matrix4f& transform, int max_iter, float max_corr_dist,
    float trans_epsilon, float fitness_epsilon);
template bool PCLUTILS::ICPPointToPoint<pcl::PointNormal>(
    const pcl::PointCloud<pcl::PointNormal>::Ptr& cloud_src,
    const pcl::PointCloud<pcl::PointNormal>::Ptr& cloud_tgt,
    Eigen::Matrix4f& transform, int max_iter, float max_corr_dist,
    float trans_epsilon, float fitness_epsilon);

}  // namespace pcllibs
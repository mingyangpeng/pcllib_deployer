/**
 * @copyright  Copyright (c) 2026 pengmingyang All Rights Reserved.
 * @file register.hpp
 * @brief 点云滤配准
 * @author pengmingyang
 */
#ifndef REGISTER_HPP_
#define REGISTER_HPP_

#include <pcl/common/common.h>
#include <pcl/common/pca.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>

#include "util.hpp"

/**
 * 分层降采样 icp 
 */

namespace pcllibs {
namespace regist {
template <typename PointT>
std::pair<Eigen::Matrix3f, Eigen::Vector3f> pca_compute_shape(
    const typename pcl::PointCloud<PointT>::Ptr& cloud) {
    pcl::MomentOfInertiaEstimation<PointT> mie;
    mie.setInputCloud(cloud);
    mie.compute();
    Eigen::Vector4f Centroid;  // 质心
    pcl::compute3DCentroid(*cloud,
                           Centroid);  // 计算质心，质心为齐次坐标（c0,c1,c2,1）

    Eigen::Matrix3f covariance_matrix;  // 协方差矩阵
    pcl::computeCovarianceMatrix(*cloud, Centroid, covariance_matrix);
    Eigen::JacobiSVD<Eigen::Matrix3f> svd(
        covariance_matrix, Eigen::ComputeFullU | Eigen::ComputeFullV);

    PointT minPt, maxPt;
    pcl::getMinMax3D(*cloud, minPt, maxPt);
    Eigen::Vector3f shapeCenter =
        0.5f * (minPt.getVector3fMap() + maxPt.getVector3fMap());
    // 获取 U, Sigma, V 矩阵
    Eigen::Matrix3f U = svd.matrixU();

    return std::make_pair(U, shapeCenter);
}

/**
 * @brief pca 信息计算
 * @param cloud_in 输入点云
 * @param eigenvalues 返回特征值
 * @param eigenvectors 返回特征向量
 * @param centroid 返回质心
 * @param is_geometric_cebter 是否使用几何质心, 否则使用点云质心
 */
template <typename PointT>
void PCAInfos(const typename pcl::PointCloud<PointT>::Ptr& cloud_in,
              Eigen::Vector3f& eigenvalues,   // 特征值
              Eigen::Matrix3f& eigenvectors,  // 特征向量
              Eigen::Vector3f& centroid,
              bool is_geometric_cebter) {  // 质心
    pcl::PCA<PointT> pca;
    pca.setInputCloud(cloud_in);
    eigenvalues = pca.getEigenValues();
    eigenvectors = pca.getEigenVectors();

    if (is_geometric_cebter) {
        PointT minPt, maxPt;
        pcl::getMinMax3D(*cloud_in, minPt, maxPt);
        Eigen::Vector3f shapeCenter =
            0.5f * (minPt.getVector3fMap() + maxPt.getVector3fMap());
        centroid = shapeCenter;
    } else {
        Eigen::Vector4f cc = pca.getMean();
        centroid = cc.head(3);
    }
}

/**
 * @brief icp 点到点配准
 * @param cloud_src 输入点云
 * @param cloud_tgt 目标点云
 * @param transform 输出变换矩阵
 * @param max_iter 最大迭代次数
 * @param max_corr_dist 最大对应距离
 * @param trans_epsilon 变换矩阵变化阈值
 * @param fitness_epsilon 欧氏距离阈值

    */
template <typename PointT>
bool icpPToPoint(const typename pcl::PointCloud<PointT>::Ptr& cloud_src,
                 const typename pcl::PointCloud<PointT>::Ptr& cloud_tgt,
                 Eigen::Matrix4f& transform, int max_iter, float max_corr_dist,
                 float trans_epsilon, float fitness_epsilon) {
    typename pcl::PointCloud<PointT>::Ptr cloud_icp(
        new pcl::PointCloud<PointT>);
    pcl::IterativeClosestPoint<PointT, PointT> icp;
    icp.setInputSource(cloud_src);
    icp.setInputTarget(cloud_tgt);
    icp.setMaxCorrespondenceDistance(max_corr_dist);
    icp.setMaximumIterations(max_iter);
    icp.setTransformationEpsilon(trans_epsilon);
    icp.setEuclideanFitnessEpsilon(fitness_epsilon);
    icp.align(*cloud_icp);
    if (icp.hasConverged()) {
        transform = icp.getFinalTransformation();
        return true;
    } else {
        PCL_ERROR("ICP failed to converge!");
        return false;
    }
}

/**
 * @brief icp 点到平面配准
 * @param src_normal 输入点云
 * @param tgt_normal 目标点云
 * @param transform 输出变换矩阵
 * @param max_iter 最大迭代次数
 * @param trans_epsilon 变换矩阵变化阈值
 * @param fitness_epsilon 欧氏距离阈值
 * @param max_corr_dist 最大对应距离
 */
// template <typename PointT>
bool icpPToPlane(const pcl::PointCloud<pcl::PointNormal>::Ptr& src_normal,
                 const pcl::PointCloud<pcl::PointNormal>::Ptr& tgt_normal,
                 Eigen::Matrix4f& transform, int max_iter, float trans_epsilon,
                 float fitness_epsilon, float max_corr_dist) {
    pcl::IterativeClosestPointWithNormals<pcl::PointNormal, pcl::PointNormal>
        icp;

    icp.setInputSource(src_normal);
    icp.setInputTarget(tgt_normal);

    icp.setMaxCorrespondenceDistance(max_corr_dist);
    icp.setMaximumIterations(max_iter);
    icp.setTransformationEpsilon(trans_epsilon);
    icp.setEuclideanFitnessEpsilon(fitness_epsilon);
    icp.setRANSACOutlierRejectionThreshold(max_corr_dist / 2.0f);

    pcl::PointCloud<pcl::PointNormal>::Ptr icp_result_normal(
        new pcl::PointCloud<pcl::PointNormal>);
    icp.align(*icp_result_normal);

    if (icp.hasConverged()) {
        transform = icp.getFinalTransformation();
        return true;
    } else {
        PCL_ERROR("ICP PointWithNormals failed to converge!");
        return false;
    }
}

/**
 * @brief icp 普通点云配准
 * @param src 输入点云
 * @param tgt 目标点云
 * @param output 输出点云
 * @param transformation_matrix 变换矩阵
 * @param max_iterations 最大迭代次数
 * @param transformation_epsilon 变换矩阵变化阈值
 */
template <typename PointT>
void icpNormal(const typename pcl::PointCloud<PointT>& src,
               const typename pcl::PointCloud<PointT>& tgt,
               pcl::PointCloud<PointT>& output,
               Eigen::Matrix4f& transformation_matrix, int max_iterations,
               float transformation_epsilon, float euclidean_fitness_epsilon,
               float max_correspondence_distance) {}

/**
 * @brief pca 点云配准
 * @param src 输入点云
 * @param tgt 目标点云
 * @param output 输出点云
 * @param transformation_matrix 变换矩阵
 */
template <typename PointT>
void PCA(const typename pcl::PointCloud<PointT>::Ptr& src,
         const typename pcl::PointCloud<PointT>::Ptr& tgt,
         Eigen::Matrix4f& transformation_matrix) {
    // 计算PCA特征
    Eigen::Vector3f Fp, Fx;  // 特征值
    Eigen::Matrix3f Up, Ux;  // 特征向量
    Eigen::Vector3f Cp, Cx;  // 质心

    PCAInfos<PointT>(src, Fp, Up, Cp, true);
    PCAInfos<PointT>(tgt, Fx, Ux, Cx, true);
    std::cout << "Cp\n" << Cp << std::endl;
    std::cout << "Cx\n" << Cx << std::endl;

    // 特征向量方向对齐
    for (int i = 0; i < 3; ++i) {
        if (Up.col(i).dot(Ux.col(i)) < 0) {
            Ux.col(i) = -Ux.col(i);
        }
    }

    // 计算变换矩阵
    Eigen::Matrix3f R0 = Ux * Up.transpose();
    // Eigen::Vector3f T0 = Cx.head(3) - R0 * Cp.head(3);  // 只取前3个元素
    Eigen::Vector3f T0 = Cx - R0 * Cp;  // 只取前3个元素

    Eigen::Matrix4f T = Eigen::Matrix4f::Identity();
    T.block<3, 3>(0, 0) = R0;
    T.block<3, 1>(0, 3) = T0;
    transformation_matrix = T;
}

/**
 * @brief icp ndt 点云配准
 * @param src 输入点云
 * @param tgt 目标点云
 * @param transformation_matrix 变换矩阵
 * @param max_iterations 最大迭代次数
 * @param transformation_epsilon 变换矩阵变化阈值
 */
template <typename PointT>
void NDT(const pcl::PointCloud<PointT>& src, const pcl::PointCloud<PointT>& tgt,
         Eigen::Matrix4f& transformation_matrix, int max_iterations,
         float transformation_epsilon, float euclidean_fitness_epsilon,
         float max_correspondence_distance) {}

}  // namespace regist
}  // namespace pcllibs
#endif  // REGISTER_HPP_

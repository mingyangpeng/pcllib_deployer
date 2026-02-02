#ifndef ICP_HPP_
#define ICP_HPP_

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/pca.h>

namespace pcllibs {
namespace icp {

/**
 * @brief pca 信息计算
 * @param cloud_in 输入点云
 * @param eigenvalues 返回特征值
 * @param eigenvectors 返回特征向量
 * @param centroid 返回质心
 */
template<typename PointT>
void PCAInfos(const pcl::PointCloud<PointT> &cloud_in,
    Eigen::Vector3f& eigenvalues,   //特征值
    Eigen::Matrix3f& eigenvectors,  //特征向量
    Eigen::Vector4f& centroid){       //质心 
    pcl::PCA<PointT> pca; 
    pca.setInputCloud(cloud_in);
    eigenvalues = pca.getEigenValues();
    eigenvectors = pca.getEigenVectors(); 
    centroid = pca.getMean();
}

template<typename PointT>
void icpNormal(const pcl::PointCloud<PointT> &src,
    const pcl::PointCloud<PointT> &tgt,
    pcl::PointCloud<PointT> &output,
    Eigen::Matrix4f &transformation_matrix,
    int max_iterations = 100,
    float transformation_epsilon = 1e-8,
    float euclidean_fitness_epsilon = 1e-8,
    float max_correspondence_distance = 0.01){
        
    }
    



}//namespace icp 
}// namespace pcllibs
#endif // ICP_HPP_

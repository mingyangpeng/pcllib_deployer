/**
 * @copyright  Copyright (c) 2026 pengmingyang All Rights Reserved.
 * @file register.hpp
 * @brief 点云显示
 * @author pengmingyang
 */
#ifndef VIEWER_HPP_
#define VIEWER_HPP_
#include <pcl/common/transforms.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/point_cloud_color_handlers.h>
#include <vtkLogger.h>
#include <vtkObject.h>

#include <chrono>
#include <thread>

namespace pcllibs {

namespace viewer {

// 添加模板参数声明
template <typename PointT>
void show_1(const typename pcl::PointCloud<PointT>::Ptr& cloud,
            const std::string& window_name = "3D Viewer") {
    // 直接创建智能指针而非复制对象
    pcl::visualization::PCLVisualizer::Ptr viewer_ptr(
        new pcl::visualization::PCLVisualizer(window_name));
    viewer_ptr->addPointCloud<PointT>(cloud, "pcd");
    viewer_ptr->setBackgroundColor(0, 0, 0);
    viewer_ptr->addCoordinateSystem(1.0);
    viewer_ptr->initCameraParameters();
    bool exit_flag = false;
    viewer_ptr->registerKeyboardCallback(
        [&exit_flag](const pcl::visualization::KeyboardEvent& event) {
            if (event.getKeySym() == "q" && event.keyDown()) {
                exit_flag = true;  // 按下 q 键触发退出
            }
        });

    while (!viewer_ptr->wasStopped() && !exit_flag) {
        viewer_ptr->spinOnce(100);
    }
}

// 第一个点云为红色，第二个点云为蓝色
template <typename PointT>
void show_2(const typename pcl::PointCloud<PointT>::Ptr& cloud1,
            const typename pcl::PointCloud<PointT>::Ptr& cloud2,
            const std::string& window_name = "3D Viewer") {
    pcl::visualization::PCLVisualizer viewer(window_name);
    viewer.setBackgroundColor(0, 0, 0);
    viewer.addPointCloud<PointT>(cloud1, "cloud1");
    viewer.addPointCloud<PointT>(cloud2, "cloud2");
    viewer.setPointCloudRenderingProperties(
        pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "cloud1");
    viewer.setPointCloudRenderingProperties(
        pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 0.0, 0.0,
        "cloud1");  // Red color for cloud1
    viewer.setPointCloudRenderingProperties(
        pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "cloud2");
    viewer.setPointCloudRenderingProperties(
        pcl::visualization::PCL_VISUALIZER_COLOR, 0.0, 0.0, 1.0,
        "cloud2");  // Blue color for cloud2
    viewer.addCoordinateSystem(1.0);
    viewer.initCameraParameters();
    while (!viewer.wasStopped()) {
        viewer.spinOnce();
    }
    viewer.close();
}

// 第一个点云为红色，第二个点云为蓝色
template <typename PointT>
void show_2(const typename pcl::PointCloud<PointT>::Ptr& cloud1,
            const typename pcl::PointCloud<PointT>::Ptr& cloud2,
            const Eigen::Matrix4f& t1, const Eigen::Matrix4f& t2,
            const std::string& window_name = "3D Viewer") {
    pcl::visualization::PCLVisualizer viewer(window_name);
    viewer.setBackgroundColor(0, 0, 0);
    typename pcl::PointCloud<PointT>::Ptr cloud1_transformed(
        new pcl::PointCloud<PointT>);
    pcl::transformPointCloud<PointT>(*cloud1, *cloud1_transformed, t1);
    typename pcl::PointCloud<PointT>::Ptr cloud2_transformed(
        new pcl::PointCloud<PointT>);
    pcl::transformPointCloud<PointT>(*cloud2, *cloud2_transformed, t2);

    viewer.addPointCloud<PointT>(cloud1_transformed, "cloud1");
    viewer.addPointCloud<PointT>(cloud2_transformed, "cloud2");
    viewer.setPointCloudRenderingProperties(
        pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "cloud1");
    viewer.setPointCloudRenderingProperties(
        pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 0.0, 0.0,
        "cloud1");  // Red color for cloud1
    viewer.setPointCloudRenderingProperties(
        pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "cloud2");
    viewer.setPointCloudRenderingProperties(
        pcl::visualization::PCL_VISUALIZER_COLOR, 0.0, 0.0, 1.0,
        "cloud2");  // Blue color for cloud2
    viewer.addCoordinateSystem(1.0);
    viewer.initCameraParameters();
    while (!viewer.wasStopped()) {
        viewer.spinOnce();
    }
    viewer.close();
}

void showNormal(const pcl::PointCloud<pcl::PointNormal>::Ptr& cloud,
                const std::string& window_name = "Normal Viewer",
                int per_num_show = 3) {
    pcl::visualization::PCLVisualizer::Ptr viewer(
        new pcl::visualization::PCLVisualizer(window_name));
    viewer->setBackgroundColor(0, 0, 0);

    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointNormal> color(
        cloud, 0, 255, 0);
    viewer->addPointCloud<pcl::PointNormal>(cloud, color, "cloud");
    viewer->setPointCloudRenderingProperties(
        pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "cloud");

    viewer->addPointCloudNormals<pcl::PointNormal>(cloud, per_num_show, 0.1,
                                                   "normals");
    viewer->setPointCloudRenderingProperties(
        pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 1.0, 1.0, "normals");
    viewer->setPointCloudRenderingProperties(
        pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 2, "normals");

    // 添加文本说明
    viewer->addText("绿色: 点云\n白色: 法线向量", 10, 10, "info");

    // 添加坐标系
    viewer->addCoordinateSystem(1.0);
    viewer->initCameraParameters();

    while (!viewer->wasStopped()) {
        viewer->spinOnce();
    }
}

void showNormal(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
                const pcl::PointCloud<pcl::Normal>::Ptr& normals,
                const std::string& window_name = "Normal Viewer",
                int per_num_show = 3) {
    pcl::visualization::PCLVisualizer::Ptr viewer(
        new pcl::visualization::PCLVisualizer(window_name));
    viewer->setBackgroundColor(0, 0, 0);  // 黑色背景

    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_color(
        cloud, 0, 255, 0);
    viewer->addPointCloud<pcl::PointXYZ>(cloud, cloud_color, "cloud");
    viewer->setPointCloudRenderingProperties(
        pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "cloud");

    viewer->addPointCloudNormals<pcl::PointXYZ, pcl::Normal>(
        cloud, normals, per_num_show, 0.1, "normals");
    viewer->setPointCloudRenderingProperties(
        pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 1.0, 1.0, "normals");
    viewer->setPointCloudRenderingProperties(
        pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 2, "normals");

    viewer->addCoordinateSystem(1.0);
    viewer->initCameraParameters();

    while (!viewer->wasStopped()) {
        viewer->spinOnce();
    }
}

// 显示pca方向
template <typename PointT>
void showPCA(const typename pcl::PointCloud<PointT>::Ptr& cloud,
             const Eigen::Vector4f& centroid,
             const Eigen::Matrix3f& eigenvectors,
             const Eigen::Vector3f& eigenvalues) {
    pcl::visualization::PCLVisualizer::Ptr viewer(
        new pcl::visualization::PCLVisualizer("PCA Visualization"));
    viewer->setBackgroundColor(0, 0, 0);  // 黑色背景
    // 添加点云（白色）
    pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_color(
        cloud, 255, 255, 255);
    viewer->addPointCloud<PointT>(cloud, cloud_color, "cloud");
    viewer->setPointCloudRenderingProperties(
        pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "cloud");

    // 质心坐标（x,y,z）
    float cx = centroid(0), cy = centroid(1), cz = centroid(2);
    // 计算箭头长度：特征值开方（归一化，避免长度过长/过短）
    float scale = 1.0 / eigenvalues.maxCoeff();  // 归一化系数
    Eigen::Vector3f ev1 = eigenvectors.col(0) * eigenvalues(0) * scale;  // PC1
    Eigen::Vector3f ev2 = eigenvectors.col(1) * eigenvalues(1) * scale;  // PC2
    Eigen::Vector3f ev3 = eigenvectors.col(2) * eigenvalues(2) * scale;  // PC3

    // 绘制PC1：红色箭头（第一主成分，最长）
    viewer->addLine(PointT(cx, cy, cz),
                    PointT(cx + ev1(0), cy + ev1(1), cz + ev1(2)), 1, 0, 0,
                    "PC1");
    viewer->setShapeRenderingProperties(
        pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 3, "PC1");
    // 绘制PC2：绿色箭头（第二主成分）
    viewer->addLine(PointT(cx, cy, cz),
                    PointT(cx + ev2(0), cy + ev2(1), cz + ev2(2)), 0, 1, 0,
                    "PC2");
    viewer->setShapeRenderingProperties(
        pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 3, "PC2");
    // 绘制PC3：蓝色箭头（第三主成分）
    viewer->addLine(PointT(cx, cy, cz),
                    PointT(cx + ev3(0), cy + ev3(1), cz + ev3(2)), 0, 0, 1,
                    "PC3");
    viewer->setShapeRenderingProperties(
        pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 3, "PC3");
    // 绘制质心：黄色球体
    viewer->addSphere(PointT(cx, cy, cz), 0.02, 1, 1, 0, "centroid");
}

// 显示OBB
template <typename PointT>
void showOBB(const typename pcl::PointCloud<PointT>::Ptr& cloud,
             const OBBDate& obb,
             const std::string& window_name = "OBB Viewer") {
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(
        new pcl::visualization::PCLVisualizer(window_name));
    viewer->setBackgroundColor(0, 0, 0);

    // 添加点云
    pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_color(
        cloud, 175, 238, 238);
    viewer->addPointCloud<PointT>(cloud, cloud_color, "cloud");
    viewer->setPointCloudRenderingProperties(
        pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "cloud");

    // 添加OBB的8个顶点
    pcl::PointCloud<pcl::PointXYZ>::Ptr obb_vertices(
        new pcl::PointCloud<pcl::PointXYZ>);
    for (const auto& vertex : obb.vertices) {
        obb_vertices->push_back(
            pcl::PointXYZ(vertex.x(), vertex.y(), vertex.z()));
    }

    // 绘制OBB的边
    std::vector<std::pair<int, int>> edges = {
        {0, 1},  {1, 2}, {2, 3}, {3, 0},  // 底面
        {4, 5}, {5, 6}, {6, 7}, {7, 4},  // 顶面
        {0, 4}, {1, 5}, {2, 6}, {3, 7}   // 连接顶面和底面的边
    };

    // 为每条边添加线段
    for (size_t i = 0; i < edges.size(); ++i) {
        const auto& edge = edges[i];
        std::string line_id = "line_" + std::to_string(i);
        viewer->addLine<pcl::PointXYZ>(obb_vertices->points[edge.first],
                                       obb_vertices->points[edge.second], 1.0,
                                       0.0, 1.0,  // 红色
                                       line_id);
        viewer->setShapeRenderingProperties(
            pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 2, line_id);
    }

    // 添加坐标系
    viewer->addCoordinateSystem(1.0);
    viewer->initCameraParameters();

    while (!viewer->wasStopped()) {
        viewer->spinOnce();
    }
}

}  // namespace viewer
}  // namespace pcllibs

#endif  // VIEWER_HPP_
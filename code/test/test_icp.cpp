#include "pcllib.hpp"
#include "util.hpp"
using namespace pcllibs;

int main() {
    std::string pcd_path = "/root/code/temp/stands_cloud_Module_C.pcd";
    std::string pcd_path1 = "/root/code/temp/ideal_stands_pcd_Module_C.pcd";
    auto pcl_ptr = std::make_shared<PCLUTILS>();
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(
        new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1(
        new pcl::PointCloud<pcl::PointXYZ>);

    {
        TIME_SCOPE("load infos");
        pcl_ptr->loadPointCloud<pcl::PointXYZ>(pcd_path, cloud);
        pcl_ptr->loadPointCloud<pcl::PointXYZ>(pcd_path1, cloud1);
    }
    pcl::PointCloud<pcl::PointNormal>::Ptr cloud_normals(
        new pcl::PointCloud<pcl::PointNormal>);
    {
        TIME_SCOPE("icp");

        // 计算法线1
        // pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new
        // pcl::PointCloud<pcl::Normal>);
        // util::computeCloudNormal<pcl::PointXYZ, pcl::Normal>(
        //         cloud, cloud_normals, 0.1, 30);
        // pcl_ptr->showNormal(cloud, cloud_normals);

        // 计算法线2
        // util::computeCloudNormal<pcl::PointXYZ, pcl::PointNormal>(
        //     cloud, cloud_normals, 0.1, 30);
        // pcl_ptr->showNormal(cloud_normals, "cloud_normals");
        // pcl_ptr->showPointCloud<pcl::PointXYZ>(cloud, cloud1,
        //                                        "cloud1 -- cloud");

        // 先裁剪

        Eigen::Matrix4f transform;
        bool isok =
            pcl_ptr->PCARegister<pcl::PointXYZ>(cloud, cloud1, transform);

        std::cout << "is ok:" << isok << std::endl;
        std::cout << "transform:\n" << transform << std::endl;
        pcl_ptr->showPointCloud<pcl::PointXYZ>(cloud, cloud1, transform,
                                               Eigen::Matrix4f::Identity(),
                                               "pca cloud1 -- cloud");

        OBBDate obb1;
        util::calcOBB<pcl::PointXYZ>(cloud1, obb1);
        float length = 1.6;
        for (int i = 0; i < 4; i++) {
            obb1.vertices[i].z() = obb1.vertices[i + 4].z() - length;
        }
        pcl::PointCloud<pcl::PointXYZ>::Ptr ideal_stands(
            new pcl::PointCloud<pcl::PointXYZ>);
        filter::cropPointCloudBy8Vertices<pcl::PointXYZ>(cloud1, obb1.vertices,
            ideal_stands, 1e-6f);
        viewer::showOBB<pcl::PointXYZ>(cloud1, obb1, "obb");



        OBBDate obb;
        util::calcOBB<pcl::PointXYZ>(cloud, obb);
    
        for (int i = 0; i < 4; i++) {
            obb.vertices[i].z() = obb.vertices[i + 4].z() - length;
        }
        viewer::showOBB<pcl::PointXYZ>(cloud, obb, "obb");
        pcl::PointCloud<pcl::PointXYZ>::Ptr stands(
            new pcl::PointCloud<pcl::PointXYZ>);
        filter::cropPointCloudBy8Vertices<pcl::PointXYZ>(cloud, obb.vertices,
                                                         stands, 1e-6f);
        pcl::PointCloud<pcl::PointXYZ>::Ptr pca_stands(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::transformPointCloud<pcl::PointXYZ>(*stands,*pca_stands, transform);    
        pcl_ptr->showPointCloud<pcl::PointXYZ>(pca_stands, ideal_stands  ,"pca");                                             

        // icp p2p
        Eigen::Matrix4f icp_t = Eigen::Matrix4f::Identity();
        isok = pcl_ptr->ICPPointToPoint<pcl::PointXYZ>(
            pca_stands, ideal_stands, icp_t, 3000, 0.5, 1e-8, 1e-6);
        pcl_ptr->showPointCloud<pcl::PointXYZ>(pca_stands, ideal_stands, icp_t,
                                               Eigen::Matrix4f::Identity(),
                                               "cloud1 -- cloud");
        std::cout << "isok: " << isok << std::endl;
    }
}
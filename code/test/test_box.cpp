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

    {
        TIME_SCOPE("BOX");
        OBBDate obb;
        util::calcOBB<pcl::PointXYZ>(cloud, obb);
        float length = 1.3;
         for (int i = 0; i < 4; i++) {
            obb.vertices[i].z() = obb.vertices[i + 4].z() - length;
        }

        viewer::showOBB<pcl::PointXYZ>(cloud, obb, "obb");
        pcl::PointCloud<pcl::PointXYZ>::Ptr stands(new pcl::PointCloud<pcl::PointXYZ>);
        filter::cropPointCloudBy8Vertices<pcl::PointXYZ>(cloud, obb.vertices, stands, false);
        pcl_ptr->showPointCloud<pcl::PointXYZ>( stands , "stands");
        pcl_ptr->showPointCloud<pcl::PointXYZ>(cloud, stands , "all stands");


        // pcl_ptr->showPointCloud<pcl::PointXYZ>(cloud, cloud1,
        //                                        "cloud1 -- cloud");
    }
}
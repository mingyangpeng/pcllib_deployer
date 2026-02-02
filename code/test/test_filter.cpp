#include "pcllib.hpp"
 
using namespace pcllibs;


int main()
{
    std::string pcd_path = "/root/code/temp/stands_cloud_Module_C.pcd";
    std::string pcd_path1 = "/root/code/temp/ideal_stands_pcd_Module_C.pcd";
    auto pcl_ptr = std::make_shared<PCLUTILS>("mylog/123.log");  //添加日志 路径
    LOG_INFO("test LOG {}",123);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1(new pcl::PointCloud<pcl::PointXYZ>);
    
    { 
        TIME_SCOPE("load infos");
        pcl_ptr->loadPointCloud<pcl::PointXYZ>(pcd_path, cloud);
        pcl_ptr->loadPointCloud<pcl::PointXYZ>(pcd_path1, cloud1);
        pcl::PointCloud<pcl::PointXYZ>::Ptr f_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl_ptr->filterNaN<pcl::PointXYZ>(cloud, cloud);
 
        // pcl_ptr->filterStatisticalOutlierRemoval<pcl::PointXYZ>(cloud, f_cloud, 50, 0.95, false);  
        // pcl_ptr->filterRadiusOutlierRemoval<pcl::PointXYZ>(cloud, f_cloud, 0.05,1);
        pcl_ptr->showPointCloud<pcl::PointXYZ>(cloud, "cloud");



        auto cx = pcl_ptr->filterCondition<pcl::PointXYZ>("x",0,"<=");
        auto cy =pcl_ptr->filterCondition<pcl::PointXYZ>("y",0,"<=");
        auto cz =pcl_ptr->filterCondition<pcl::PointXYZ>("z",0,"<=");
        std::vector<pcl::FieldComparison<pcl::PointXYZ>::Ptr> conditions;
        conditions.push_back(cx);
        conditions.push_back(cy);
        conditions.push_back(cz);
        pcl_ptr->filterConditionRemovalFilter <pcl::PointXYZ>(cloud, f_cloud, conditions, "and", false);
        

        pcl_ptr->showPointCloud<pcl::PointXYZ>(f_cloud, "f_cloud");

    }


}
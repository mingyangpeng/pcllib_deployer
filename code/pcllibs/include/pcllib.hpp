#ifndef PCLLIB_HPP_
#define PCLLIB_HPP_

#include "util.hpp"
#include "viewer.hpp"


class PCLUTILS
{
public:
    PCLUTILS();
    ~PCLUTILS();

    template<typename PointT>
    void showPointCloud(const typename pcl::PointCloud<PointT>::Ptr& cloud, const std::string& window_name = "3D Viewer");
};





#endif //PCLLIB_HPP_
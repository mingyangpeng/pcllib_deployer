#include "filter.hpp"

namespace pcllibs{


    void calcBoxParamsFrom8Vertices(const std::vector<Eigen::Vector3f>& vertices,
        Eigen::Vector3f& center, Eigen::Vector3f& size,
        Eigen::Matrix3f& rotation) {
// 步骤1：严格检查顶点数量
if (vertices.size() != 8) {
PCL_ERROR("包围盒顶点数量必须为8！当前数量：%zu\n", vertices.size());
exit(-1);
}

// 步骤2：将Eigen向量顶点转换为3×8的Eigen矩阵，用于数值计算
Eigen::MatrixXf vertices_mat(3, 8);
for (int i = 0; i < 8; ++i) {
vertices_mat.col(i) = vertices[i];
}

// 步骤3：计算包围盒中心（8顶点的均值）
center = vertices_mat.rowwise().mean();

// 步骤4：顶点去中心（减去中心坐标，用于拟合主轴）
Eigen::MatrixXf centered_mat = vertices_mat.colwise() - center;

// 步骤5：SVD奇异值分解 →
// 拟合OBB主轴，得到旋转矩阵（核心：适配任意定向包围盒）
Eigen::JacobiSVD<Eigen::MatrixXf> svd(centered_mat, Eigen::ComputeFullV);
rotation = svd.matrixV().transpose();  // 旋转矩阵对齐包围盒主轴

// 步骤6：将去中心顶点投影到旋转后的主轴，计算三轴实际尺寸
Eigen::MatrixXf rotated_mat = rotation * centered_mat;
Eigen::Vector3f min_pt, max_pt;
for (int i = 0; i < 3; ++i) {
min_pt(i) = rotated_mat.row(i).minCoeff();
max_pt(i) = rotated_mat.row(i).maxCoeff();
}
size = max_pt - min_pt;  // 三轴尺寸为投影后的最大-最小值

// 打印包围盒参数（调试用，可注释）
std::cout << "\n===== 8顶点包围盒参数（Eigen输入）=====" << std::endl;
std::cout << "中心(xyz)：" << center.transpose() << std::endl;
std::cout << "三轴尺寸(xyz)：" << size.transpose() << std::endl;
std::cout << "旋转矩阵：\n" << rotation << std::endl;
std::cout << "=====================================\n" << std::endl;
}

}
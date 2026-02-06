#include "util.hpp"

namespace pcllibs {
namespace util {

// 析构函数：离开作用域时自动调用，计算并打印耗时
ScopedTimer::~ScopedTimer() {
    auto end_time = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(
        end_time - start_time);
    std::cout << "[" << task_name << "] 耗时: " << duration.count() / 1000.0f
              << " ms" << std::endl;
}

// 检查变换矩阵是否为正交变换矩阵
bool isRotationMatrixOrthogonal(const Eigen::Matrix4f& T, float tolerance) {
    // 提取旋转矩阵部分
    Eigen::Matrix3f R = T.block<3, 3>(0, 0);

    // 检查 R * R^T 是否接近单位矩阵
    Eigen::Matrix3f product = R * R.transpose();
    Eigen::Matrix3f identity = Eigen::Matrix3f::Identity();

    // 计算差值矩阵的Frobenius范数
    float diff = (product - identity).norm();

    // 检查行列式是否接近1（右手坐标系）
    float det = R.determinant();

    // 判断是否满足正交性条件
    return (diff < tolerance) && (std::abs(det - 1.0f) < tolerance);
}

bool isRotationMatrixOrthogonal(const Eigen::Matrix3f& R, float tolerance) {
    // 提取旋转矩阵部分

    // 检查 R * R^T 是否接近单位矩阵
    Eigen::Matrix3f product = R * R.transpose();
    Eigen::Matrix3f identity = Eigen::Matrix3f::Identity();

    // 计算差值矩阵的Frobenius范数
    float diff = (product - identity).norm();

    // 检查行列式是否接近1（右手坐标系）
    float det = R.determinant();

    // 判断是否满足正交性条件
    return (diff < tolerance) && (std::abs(det - 1.0f) < tolerance);
}

static Eigen::Matrix3f orthogonalizeToRotationMatrix(
    const Eigen::Matrix3f& mat) {
    // SVD分解实现正交化
    Eigen::JacobiSVD<Eigen::Matrix3f> svd(
        mat, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::Matrix3f rotOrtho = svd.matrixU() * svd.matrixV().transpose();

    // 修正行列式为1（避免镜像变换）
    if (std::abs(rotOrtho.determinant() + 1.0f) < 1e-6) {
        rotOrtho.col(2) *= -1.0f;
    }

    return rotOrtho;
}

// 计算T1*T2的复合变换矩阵, 并确保结果矩阵是正交变换矩阵
Eigen::Matrix4f mulT(const Eigen::Matrix4f& T1, const Eigen::Matrix4f& T2) {
    // 输入校验：确保是合法的4x4刚体变换矩阵（第四行必须为[0,0,0,1]）
    const float eps = 1e-6;

    if (!T1.row(3).isApprox(Eigen::RowVector4f(0, 0, 0, 1), eps)) {
        // 输出错误信息到stderr（所有模式都生效）
        std::cerr << "[ERROR] computeCompositeTransform: T1 is not a valid "
                     "rigid transform matrix! "
                  << "4th row expected [0,0,0,1], but got: "
                  << T1.row(3).transpose() << std::endl;
        std::abort();
    }
    // 校验T2的第四行
    if (!T2.row(3).isApprox(Eigen::RowVector4f(0, 0, 0, 1), eps)) {
        std::cerr << "[ERROR] computeCompositeTransform: T2 is not a valid "
                     "rigid transform matrix! "
                  << "4th row expected [0,0,0,1], but got: "
                  << T2.row(3).transpose() << std::endl;
        std::abort();
    }

    // 1. 提取旋转和平移部分
    Eigen::Matrix3f rot1 = T1.block<3, 3>(0, 0);
    Eigen::Matrix3f rot2 = T2.block<3, 3>(0, 0);
    Eigen::Vector3f trans1 = T1.block<3, 1>(0, 3);
    Eigen::Vector3f trans2 = T2.block<3, 1>(0, 3);

    // 2. 对输入旋转矩阵做正交化（处理数值误差/非正交情况）
    rot1 = orthogonalizeToRotationMatrix(rot1);
    rot2 = orthogonalizeToRotationMatrix(rot2);

    // 3. 旋转矩阵转四元数并归一化
    Eigen::Quaternionf q1(rot1);
    Eigen::Quaternionf q2(rot2);
    q1.normalize();
    q2.normalize();

    // 4. 四元数乘法：q1*q2 对应 rot1*rot2（复合旋转）
    Eigen::Quaternionf qResult = q1 * q2;

    // 5. 确保结果四元数是单位四元数
    qResult.normalize();

    // 6. 转回旋转矩阵并验证合法性，不合法则兜底正交化
    Eigen::Matrix3f rotResult = qResult.toRotationMatrix();
    if (!isRotationMatrixOrthogonal(rotResult)) {
        rotResult = orthogonalizeToRotationMatrix(rotResult);
    }

    // 7. 计算复合平移：T1*T2的平移部分 = R1*trans2 + trans1
    Eigen::Vector3f transResult = rot1 * trans2 + trans1;

    // 8. 构建最终的4x4变换矩阵
    Eigen::Matrix4f TResult = Eigen::Matrix4f::Identity();
    TResult.block<3, 3>(0, 0) = rotResult;
    TResult.block<3, 1>(0, 3) = transResult;
    return TResult;
}

std::string createFolder(const std::string& path, bool isfolder) {
    std::filesystem::path file_path(path);
    std::string folder_path;
    if (!isfolder) {
        folder_path = file_path.parent_path().string();
    } else {
        folder_path = file_path.string();
    }
    if (!std::filesystem::exists(folder_path)) {
        std::filesystem::create_directories(
            folder_path);  // 创建所有不存在的父文件夹
    }
    return folder_path;
}

void getOBBCorners(const OBBDate& obb, std::vector<Eigen::Vector3f>& vertices) {
    vertices.clear();
    vertices.resize(8);

    // 获取OBB的三个主轴
    Eigen::Vector3f axis_x = obb.axes.col(0);
    Eigen::Vector3f axis_y = obb.axes.col(1);
    Eigen::Vector3f axis_z = obb.axes.col(2);

    // 计算OBB的8个顶点
    vertices[0] = obb.center - obb.extents.x() * axis_x -
                  obb.extents.y() * axis_y - obb.extents.z() * axis_z;
    vertices[1] = obb.center + obb.extents.x() * axis_x -
                  obb.extents.y() * axis_y - obb.extents.z() * axis_z;
    vertices[2] = obb.center + obb.extents.x() * axis_x +
                  obb.extents.y() * axis_y - obb.extents.z() * axis_z;
    vertices[3] = obb.center - obb.extents.x() * axis_x +
                  obb.extents.y() * axis_y - obb.extents.z() * axis_z;

    vertices[4] = obb.center - obb.extents.x() * axis_x -
                  obb.extents.y() * axis_y + obb.extents.z() * axis_z;
    vertices[5] = obb.center + obb.extents.x() * axis_x -
                  obb.extents.y() * axis_y + obb.extents.z() * axis_z;
    vertices[6] = obb.center + obb.extents.x() * axis_x +
                  obb.extents.y() * axis_y + obb.extents.z() * axis_z;
    vertices[7] = obb.center - obb.extents.x() * axis_x +
                  obb.extents.y() * axis_y + obb.extents.z() * axis_z;

    // 按Z轴值排序顶点
    std::sort(vertices.begin(), vertices.end(),
              [](const Eigen ::Vector3f& a, const Eigen ::Vector3f& b) {
                  return a.z() < b.z();
              });

    // 对每组点进行XY平面逆时针排序
    auto sortInXYPlane = [](std::vector<Eigen::Vector3f>& points, int start,
                            int end) {
        // 找到XY坐标最小的点作为起点
        int min_xy_idx = start;
        for (int i = start + 1; i < end; ++i) {
            if (points[i].x() < points[min_xy_idx].x() ||
                (points[i].x() == points[min_xy_idx].x() &&
                 points[i].y() < points[min_xy_idx].y())) {
                min_xy_idx = i;
            }
        }

        // 将XY最小的点移到组内首位
        std::swap(points[start], points[min_xy_idx]);

        // 计算当前组的中心点（质心）
        float center_x = 0.0f, center_y = 0.0f;
        for (int i = start; i < end; ++i) {
            center_x += points[i].x();
            center_y += points[i].y();
        }
        center_x /= (end - start);
        center_y /= (end - start);

        // 对剩余点按相对于中心点的角度排序，确保逆时针顺序
        std::sort(points.begin() + start + 1, points.begin() + end,
                  [center_x, center_y](const Eigen ::Vector3f& a,
                                       const Eigen ::Vector3f& b) {
                      // 计算相对于中心点的向量
                      Eigen::Vector2f vec_a(a.x() - center_x, a.y() - center_y);
                      Eigen::Vector2f vec_b(b.x() - center_x, b.y() - center_y);

                      // 计算角度
                      double angle_a = std::atan2(vec_a.y(), vec_a.x());
                      double angle_b = std::atan2(vec_b.y(), vec_b.x());

                      return angle_a < angle_b;

                      //   return angle_a < angle_b;
                  });
    };

    // 分别对底部和顶部顶点进行排序
    sortInXYPlane(vertices, 0, 4);  // 底部4点（Z值较小）
    sortInXYPlane(vertices, 4, 8);  // 顶部4点（Z值较大）

    // 输出排序后的顶点
    for (int i = 0; i < 8; i++) {
        std::cout << i << " : " << vertices[i].transpose() << std::endl;
    }
}

}  // namespace util

}  // namespace pcllibs
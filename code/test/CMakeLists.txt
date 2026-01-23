cmake_minimum_required(VERSION 3.8)
project(test)

# 添加 pcllibs 子目录
add_subdirectory(../pcllibs ${CMAKE_CURRENT_BINARY_DIR}/pcllibs)





# 创建测试可执行文件
add_executable(test_voxel test_voxel.cpp)

# 只需链接 pcllibs，PCL 依赖会自动传递
target_link_libraries(test_voxel pcllibs)

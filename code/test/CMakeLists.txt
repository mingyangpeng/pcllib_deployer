cmake_minimum_required(VERSION 3.8)
project(test)

# 添加 pcllibs 子目录
add_subdirectory(../pcllibs ${CMAKE_CURRENT_BINARY_DIR}/pcllibs)

# 创建测试可执行文件


set(TEST_VOXEL OFF  CACHE BOOL "Build test voxel")
if (TEST_VOXEL)
    add_executable(test_voxel test_voxel.cpp) 
    target_link_libraries(test_voxel pcllibs)
endif (TEST_VOXEL)  

set(TEST_FILTER OFF  CACHE BOOL "Build test filter")
if (TEST_FILTER)
    add_executable(test_filter test_filter.cpp) 
    target_link_libraries(test_filter pcllibs)
endif (TEST_FILTER)  

set(TEST_ICP OFF  CACHE BOOL "Build test icp")
if (TEST_ICP)
    add_executable(test_icp test_icp.cpp) 
    target_link_libraries(test_icp pcllibs)
endif (TEST_ICP)  


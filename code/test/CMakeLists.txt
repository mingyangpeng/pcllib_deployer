cmake_minimum_required(VERSION 3.8)
# 缓存加速
find_program(CCACHE_PROGRAM ccache)
if(CCACHE_PROGRAM)
  # 为C/C++编译器设置ccache包装
  set(CMAKE_C_COMPILER_LAUNCHER "${CCACHE_PROGRAM}")
  set(CMAKE_CXX_COMPILER_LAUNCHER "${CCACHE_PROGRAM}")
  message(STATUS "✅ 成功集成ccache：${CCACHE_PROGRAM}")
endif()

project(test)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
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

set(TEST_ICP ON  CACHE BOOL "Build test icp")
if (TEST_ICP)
    add_executable(test_icp test_icp.cpp) 
    target_link_libraries(test_icp pcllibs)
endif (TEST_ICP)  

set(TEST_BOX ON  CACHE BOOL "Build test box")
if (TEST_BOX)
    add_executable(test_box test_box.cpp) 
    target_link_libraries(test_box pcllibs)
endif (TEST_BOX)  


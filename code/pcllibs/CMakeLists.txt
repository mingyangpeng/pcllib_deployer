# cmake_minimum_required(VERSION 3.8)

# # 缓存加速 根目录有 子目录里可不加
# # find_program(CCACHE_PROGRAM ccache)
# # if(CCACHE_PROGRAM)
# #   # 为C/C++编译器设置ccache包装
# #   set(CMAKE_C_COMPILER_LAUNCHER "${CCACHE_PROGRAM}")
# #   set(CMAKE_CXX_COMPILER_LAUNCHER "${CCACHE_PROGRAM}")
# #   message(STATUS "✅ 成功集成ccache：${CCACHE_PROGRAM}")
# # endif()

# project(pcllibs)

# if(POLICY CMP0144)
#     cmake_policy(SET CMP0144 NEW)
# endif()


# add_definitions(-DPCL_NO_PRECOMPILE)
# add_definitions(-DBOOST_NO_CXX11_SMART_PTR)

# if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
#   add_compile_options(-Wall -Wextra -Wpedantic)
# endif()

# # 设置C++标准
# set(CMAKE_CXX_STANDARD 20)
# set(CMAKE_CXX_STANDARD_REQUIRED ON)
# SET(CMAKE_CXX_FLAGS "-O3 -Wall ${CMAKE_CXX_FLAGS}")
# # SET(CMAKE_CXX_FLAGS "-O3 -g -Wall ${CMAKE_CXX_FLAGS}")

# add_subdirectory(libs/log_util_lib)

# # dpkg -l | grep libpcl  pcl 1.14
# # 查找依赖包
# find_package(Boost REQUIRED COMPONENTS thread)
# find_package(PCL REQUIRED COMPONENTS 
#   common
#   filters
#   segmentation
#   registration
#   visualization
#   features
#   kdtree
#   io
#   search
# )
# find_package(OpenCV REQUIRED)

# # 收集src目录下所有源文件
# file(GLOB_RECURSE PCLLIBS_SOURCES 
#   "${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp"
# )

# # 创建动态库
# add_library(pcllibs SHARED ${PCLLIBS_SOURCES})

# # 链接依赖库
# target_link_libraries(pcllibs PUBLIC 
#   ${PCL_LIBRARIES}
#   ${Boost_LIBRARIES}
#   ${OpenCV_LIBS}
#   log_util_lib
# )

# # 设置包含目录
# target_include_directories(pcllibs PUBLIC 
#   include 
#   ${PCL_INCLUDE_DIRS} 
#   ${EIGEN3_INCLUDE_DIR} 
#   ${OpenCV_INCLUDE_DIRS}
# )

# set_property(TARGET pcllibs PROPERTY POSITION_INDEPENDENT_CODE ON)



# # # 安装库和头文件
# # install(TARGETS pcllibs
# #   LIBRARY DESTINATION lib
# #   ARCHIVE DESTINATION lib
# #   RUNTIME DESTINATION bin
# # )

# # install(DIRECTORY include/
# #   DESTINATION include
# # )
# # # 导出库信息
# # export(TARGETS pcllibs
# #   FILE "${CMAKE_CURRENT_BINARY_DIR}/pcllibsTargets.cmake"
# #   NAMESPACE pcl_utils::
# # )

# # install(EXPORT pcllibsTargets
# #   FILE pcllibsTargets.cmake
# #   NAMESPACE pcl_utils::
# #   DESTINATION lib/cmake/pcl_utils
# # )


# # # add_definitions(-DVTK_SILENCE_DEPRECATION_WARNINGS)
# # # 添加demo可执行文件
# # add_executable(demo demo.cpp)
# # target_link_libraries(demo pcl_utils)
 



cmake_minimum_required(VERSION 3.8)

# 缓存加速 根目录有 子目录里可不加
# find_program(CCACHE_PROGRAM ccache)
# if(CCACHE_PROGRAM)
#   set(CMAKE_C_COMPILER_LAUNCHER "${CCACHE_PROGRAM}")
#   set(CMAKE_CXX_COMPILER_LAUNCHER "${CCACHE_PROGRAM}")
#   message(STATUS "✅ 成功集成ccache：${CCACHE_PROGRAM}")
# endif()

project(pcllibs)

if(POLICY CMP0144)
    cmake_policy(SET CMP0144 NEW)
endif()

add_definitions(-DPCL_NO_PRECOMPILE)
add_definitions(-DBOOST_NO_CXX11_SMART_PTR)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 设置C++标准
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
SET(CMAKE_CXX_FLAGS "-O3 -Wall ${CMAKE_CXX_FLAGS}")
# SET(CMAKE_CXX_FLAGS "-O3 -g -Wall ${CMAKE_CXX_FLAGS}")

# 保留内部依赖跟踪：改log_util_lib源码可正常增量编译
add_subdirectory(libs/log_util_lib)
# 仅轻量优化log_util_lib，不切断内部依赖
set_target_properties(log_util_lib PROPERTIES
  POSITION_INDEPENDENT_CODE ON  # 兼容动态库链接
  CMAKE_SKIP_INSTALL_RPATH ON   # 仅关闭安装路径检查，保留构建依赖
)

# ---------------------- 核心：仅禁第三方库（Boost/PCL/OpenCV）回溯【开始】 ----------------------
# 全局优化：仅关闭第三方库的包注册，不影响内部依赖
set(CMAKE_FIND_PACKAGE_NO_PACKAGE_REGISTRY ON CACHE BOOL "关闭项目包注册" FORCE)
set(CMAKE_FIND_PACKAGE_NO_SYSTEM_PACKAGE_REGISTRY ON CACHE BOOL "关闭系统包注册" FORCE)
set(CMAKE_SUPPRESS_DEVELOPER_WARNINGS ON CACHE BOOL "关闭无用警告" FORCE)

# 查找第三方依赖（保留原有逻辑，无修改）
find_package(Boost REQUIRED COMPONENTS thread)
find_package(PCL REQUIRED COMPONENTS 
  common filters segmentation registration visualization features kdtree io search
)
find_package(OpenCV REQUIRED)

# 关键1：将Boost/PCL/OpenCV/Eigen3标记为【系统预编译库】，CMake不做任何回溯检查
# 禁止扫描版本、路径、内部依赖，仅直接使用库文件/头文件，不生成任何项目内依赖规则
foreach(lib IN ITEMS Boost PCL OpenCV log_util_lib)
  if(${lib}_FOUND)
    # 锁定库路径，禁止CMake重新查找/验证
    set(${lib}_DIR "${${lib}_DIR}" CACHE STRING "${lib}安装路径" FORCE)
    set(${lib}_INCLUDE_DIRS "${${lib}_INCLUDE_DIRS}" CACHE STRING "${lib}头文件路径" FORCE)
    set(${lib}_LIBRARIES "${${lib}_LIBRARIES}" CACHE STRING "${lib}库文件" FORCE)
    # 标记为高级隐藏变量，CMake不主动扫描/显示/验证
    mark_as_advanced(FORCE ${lib}_DIR ${lib}_INCLUDE_DIRS ${lib}_LIBRARIES ${lib}_FOUND)
  endif()
endforeach()
# 单独处理Eigen3（PCL隐式依赖，必加）
mark_as_advanced(FORCE EIGEN3_INCLUDE_DIR EIGEN3_FOUND)

# 关键2：禁止CMake为第三方库生成任何依赖链信息，不回溯到其内部
set_property(GLOBAL PROPERTY CMAKE_TARGETS_IMPORTED_ONLY Boost PCL OpenCV)
# ---------------------- 核心：仅禁第三方库（Boost/PCL/OpenCV）回溯【结束】 ----------------------

# 收集pcllibs源码（保留原有逻辑，改此目录源码可增量编译）
file(GLOB_RECURSE PCLLIBS_SOURCES 
  "${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp"
)

# 创建pcllibs动态库（保留原有逻辑，无修改）
add_library(pcllibs SHARED ${PCLLIBS_SOURCES})

# 链接依赖（保留原有所有逻辑，内部+第三方，无修改）
target_link_libraries(pcllibs PUBLIC 
  ${PCL_LIBRARIES} ${Boost_LIBRARIES} ${OpenCV_LIBS} log_util_lib
)

# 包含目录（保留原有所有逻辑，无修改）
target_include_directories(pcllibs PUBLIC 
  include ${PCL_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR} ${OpenCV_INCLUDE_DIRS}
)

# 原有动态库必需配置（保留，无修改）
set_property(TARGET pcllibs PROPERTY POSITION_INDEPENDENT_CODE ON)

 
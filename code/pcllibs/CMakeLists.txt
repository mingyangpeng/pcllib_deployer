cmake_minimum_required(VERSION 3.8)
project(pcl_utils)

add_definitions(-DPCL_NO_PRECOMPILE)
add_definitions(-DBOOST_NO_CXX11_SMART_PTR)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 设置C++标准
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
SET(CMAKE_CXX_FLAGS "-O3 -g -Wall ${CMAKE_CXX_FLAGS}")

# dpkg -l | grep libpcl  pcl 1.14
# 查找依赖包
find_package(Boost REQUIRED COMPONENTS thread)
find_package(PCL REQUIRED COMPONENTS 
  common
  filters
  segmentation
  registration
  visualization
  features
  kdtree
  io
  search
)
find_package(OpenCV REQUIRED)

# 收集src目录下所有源文件
file(GLOB_RECURSE PCL_UTILS_SOURCES 
  "${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp"
)

# 创建动态库
add_library(pcl_utils SHARED ${PCL_UTILS_SOURCES})

# 链接依赖库
target_link_libraries(pcl_utils PUBLIC 
  ${PCL_LIBRARIES}
  ${Boost_LIBRARIES}
  ${OpenCV_LIBS}
)

# 设置包含目录
target_include_directories(pcl_utils PUBLIC 
  include 
  ${PCL_INCLUDE_DIRS} 
  ${EIGEN3_INCLUDE_DIR} 
  ${OpenCV_INCLUDE_DIRS}
)
# add_definitions(-DVTK_SILENCE_DEPRECATION_WARNINGS)
# 添加demo可执行文件
add_executable(demo demo.cpp)
target_link_libraries(demo pcl_utils)

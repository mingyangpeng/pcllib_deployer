cmake_minimum_required(VERSION 3.8)
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
file(GLOB_RECURSE PCLLIBS_SOURCES 
  "${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp"
)

# 创建动态库
add_library(pcllibs SHARED ${PCLLIBS_SOURCES})

# 链接依赖库
target_link_libraries(pcllibs PUBLIC 
  ${PCL_LIBRARIES}
  ${Boost_LIBRARIES}
  ${OpenCV_LIBS}
)

# 设置包含目录
target_include_directories(pcllibs PUBLIC 
  include 
  ${PCL_INCLUDE_DIRS} 
  ${EIGEN3_INCLUDE_DIR} 
  ${OpenCV_INCLUDE_DIRS}
)

set_property(TARGET pcllibs PROPERTY POSITION_INDEPENDENT_CODE ON)



# # 安装库和头文件
# install(TARGETS pcllibs
#   LIBRARY DESTINATION lib
#   ARCHIVE DESTINATION lib
#   RUNTIME DESTINATION bin
# )

# install(DIRECTORY include/
#   DESTINATION include
# )
# # 导出库信息
# export(TARGETS pcllibs
#   FILE "${CMAKE_CURRENT_BINARY_DIR}/pcllibsTargets.cmake"
#   NAMESPACE pcl_utils::
# )

# install(EXPORT pcllibsTargets
#   FILE pcllibsTargets.cmake
#   NAMESPACE pcl_utils::
#   DESTINATION lib/cmake/pcl_utils
# )


# # add_definitions(-DVTK_SILENCE_DEPRECATION_WARNINGS)
# # 添加demo可执行文件
# add_executable(demo demo.cpp)
# target_link_libraries(demo pcl_utils)

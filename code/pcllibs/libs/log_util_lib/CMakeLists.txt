cmake_minimum_required(VERSION 3.10.0)
project(log_util_lib VERSION 0.1.0 LANGUAGES C CXX)
# 添加编译选项1
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# 添加spdlog子模块
add_subdirectory(third_party/spdlog)

add_library(log_util_lib src/logger.cpp)
target_link_libraries(log_util_lib spdlog::spdlog)
target_include_directories(log_util_lib PUBLIC include)

# 确保使用位置无关代码1
set_property(TARGET log_util_lib PROPERTY POSITION_INDEPENDENT_CODE ON)

set(ADD_SPDLOG_TEST OFF)  # Set this to OFF to prevent execution 
if(ADD_SPDLOG_TEST)
    add_executable(spdlog_test demo.cpp)
    target_link_libraries(spdlog_test log_util_lib)
endif()


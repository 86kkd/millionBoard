# 指定最低的 CMake 版本
cmake_minimum_required(VERSION 3.5)

# 包含 ESP-IDF 的 CMake 配置
include($ENV{IDF_PATH}/tools/cmake/project.cmake)

# 设置工程名称
project(skateboard_foc_project)
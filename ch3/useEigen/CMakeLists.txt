cmake_minimum_required(VERSION 2.8)
project(useEigen)

set(CMAKE_BUILD_TYPE "Debug")
set(CMAKE_CXX_FLAGS "-O3")

# 添加 Eigen（由於皆為 header 文件，因此不需要利用 target_link_libraries 去鏈結函式庫）
include_directories("/usr/include/eigen3")
add_executable(eigenMatrix eigenMatrix.cpp)

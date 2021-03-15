# 宣告要求的 cmake 最低版本
cmake_minimum_required(VERSION 2.8)

# 宣告一個 cmake 專案
project(HelloSlam)

set(CMAKE_BUILD_TYPE "Debug")

# 添加可執行程式
add_executable(hello_slam main.cpp)

# 添加函數庫(library)
add_library(hello hello.cpp)

# 將'可執行程式'與'要使用的函式庫'鍊結起來
target_link_libraries(hello_slam hello)

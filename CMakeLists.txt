# CMake最小版本要求
cmake_minimum_required(VERSION 3.15)

# 仓库根工程名称
project(algorithm LANGUAGES CXX)

# 设置C++标准
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 编译公共模块（所有算法共用）
# add_subdirectory(common)

# 编译所有算法（每个算法独立子目录，自动识别CMakeLists.txt）
add_subdirectory(dijkstra)
add_subdirectory(astar)
# 后续新增算法，只需添加这一行（如A*）：
# add_subdirectory(algorithms/a_star)

# 可选：统一输出目录
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/build/bin)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/build/lib)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/build/lib)

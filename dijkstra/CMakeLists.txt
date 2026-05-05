# CMake最小版本要求（兼容VS2019+、Linux GCC）
cmake_minimum_required(VERSION 3.15)

# 工程名称（独立工程名，建议与算法名一致）
project(dijkstra LANGUAGES CXX)

# 设置C++标准（兼容C++11及以上，匹配原有代码）
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 配置多平台编译选项（避免平台差异）
if(WIN32)
    # Windows平台：生成VS工程，关闭控制台窗口（可选，测试用可保留）
    add_compile_options(/EHsc)
    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} /SUBSYSTEM:CONSOLE")
elseif(UNIX)
    # Linux平台：生成makefile，添加编译优化
    add_compile_options(-Wall -O2)
endif()

# 引入头文件目录（dijkstra专属头文件+公共头文件）
include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/../../common/include  # 引用公共模块头文件
)

# 收集源文件（dijkstra核心代码+测试代码）
file(GLOB_RECURSE SRC_FILES
    ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/test/*.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/../../common/src/*.cpp  # 引用公共模块源文件
)

# 生成可执行文件（可执行文件名：dijkstra_test，对应测试用例）
add_executable(${PROJECT_NAME}_test ${SRC_FILES})

# 可选：设置输出目录（统一编译产物路径，便于查找）
set(OUTPUT_DIR ${CMAKE_CURRENT_SOURCE_DIR}/build/${CMAKE_SYSTEM_NAME})
set_target_properties(${PROJECT_NAME}_test PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${OUTPUT_DIR}/bin
    ARCHIVE_OUTPUT_DIRECTORY ${OUTPUT_DIR}/lib
    LIBRARY_OUTPUT_DIRECTORY ${OUTPUT_DIR}/lib
)

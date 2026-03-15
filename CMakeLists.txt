cmake_minimum_required (VERSION 3.10)
cmake_policy(SET CMP0048 NEW)


if(CMAKE_EXPORT_COMPILE_COMMANDS)
    execute_process(
        COMMAND ${CMAKE_COMMAND} -E create_symlink
        ${CMAKE_BINARY_DIR}/compile_commands.json
        ${CMAKE_SOURCE_DIR}/compile_commands.json
    )
endif()

project(PointCloud VERSION 1.0.1)

if(NOT DEFINED PROJECT_SOURCE_DIR)
	set(PROJECT_SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR})
endif()


option(BUILD_SHARED_LIBS "Build shared libraries" OFF)


# 引用编译器配置
include(${PROJECT_SOURCE_DIR}/cmake/config.cmake)

# 引用第三方库配置
include(${PROJECT_SOURCE_DIR}/../ThirdPart/toolchain.cmake)


find_package(OpenMP REQUIRED)

find_package(Vulkan REQUIRED)


# 头文件搜索路径
# include_directories(${PROJECT_SOURCE_DIR}/CommonTools)
# include_directories(${PROJECT_SOURCE_DIR}/PointCloudSegmentation)


# set(DEPENDENCY_LIBS "")

# include_directories(${OSG_INCLUDE_DIR})
# list(APPEND DEPENDENCY_LIBS ${OSG_LIBS})


add_subdirectory(CommonTools)
add_subdirectory(PointCloudSegmentation)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)


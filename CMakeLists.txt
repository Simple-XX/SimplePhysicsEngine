
# This file is a part of Simple-XX/SimplePhysicsEngine
# (https://github.com/Simple-XX/SimplePhysicsEngine).
#
# CMakeLists.txt for Simple-XX/SimplePhysicsEngine.

# 设置最小 cmake 版本
cmake_minimum_required(VERSION 3.27 FATAL_ERROR)

# 设置项目名与版本
project(SimplePhysicsEngine
        VERSION 0.0.1
        DESCRIPTION "A simple physics engine in developing..."
        HOMEPAGE_URL "https://github.com/Simple-XX/SimplePhysicsEngine"
)

# 禁止原地编译
if (${PROJECT_SOURCE_DIR} STREQUAL ${PROJECT_BINARY_DIR})
    # 如果你看到这句话，cmake 此时已经在根目录下生成了一些临时文件，你需要删除它们
    # CMakeFiles, CMakeCache.txt
    message(
            FATAL_ERROR
            "In-source builds not allowed. Please make a new directory (called a build directory) and run CMake from there."
    )
endif ()

# 设置辅助 cmake 脚本路径
list(APPEND CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake")

# 导入函数
include(functions)
# 导入第三方依赖
include(3rd)
# 导入配置
include(project_config)
include(compile_config)

add_subdirectory(src)
add_subdirectory(test)
add_subdirectory(doc)

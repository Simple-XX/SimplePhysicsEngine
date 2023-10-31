
# This file is a part of Simple-XX/SimplePhysicsEngine
# (https://github.com/Simple-XX/SimplePhysicsEngine).
#
# CMakeLists.txt for Simple-XX/SimplePhysicsEngine.

# 设置项目名与版本
project(
        system-test
        VERSION 0.0.1
)

enable_language(CXX)

# add_executable(system_test
#         main.cpp
#         display.cpp
# )

# target_compile_options(system_test PRIVATE
#         ${DEFAULT_TEST_COMPILE_OPTIONS}
# )

# target_link_options(system_test PRIVATE
#         ${DEFAULT_TEST_LINK_OPTIONS}
# )

add_subdirectory(demo2d)
# add_subdirectory(demo3d)
# add_subdirectory(demo_old)

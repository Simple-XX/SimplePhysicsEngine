
# This file is a part of Simple-XX/SimplePhysicsEngine
# (https://github.com/Simple-XX/SimplePhysicsEngine).
#
# CMakeLists.txt for Simple-XX/SimplePhysicsEngine.

# 设置项目名与版本
project(
        test
        VERSION 0.0.1
)

enable_testing()
include(GoogleTest)

include_directories(
        ${SimplePhysicsEngine_SOURCE_DIR}/src/include
)

list(APPEND DEFAULT_TEST_COMPILE_OPTIONS
        ${DEFAULT_COMPILE_OPTIONS}
        --coverage
)

list(APPEND DEFAULT_TEST_LINK_OPTIONS
        --coverage
        $<$<CXX_COMPILER_ID:GNU>:
        -fsanitize=leak
        >
        -fsanitize=address
        -fno-omit-frame-pointer
)

link_libraries(
        ${DEFAULT_LINK_LIB}
        gtest_main
        ${glog_LIBRARIES}
        SimplePhysicsEngine-static
)

add_subdirectory(unit_test)
#add_subdirectory(integration_test)
add_subdirectory(system_test)

add_coverage_target(
        DEPENDS unit_test
        SOURCE_DIR ${SimplePhysicsEngine_SOURCE_DIR}
        BINARY_DIR ${SimplePhysicsEngine_BINARY_DIR}
        EXCLUDE_DIR ${SimplePhysicsEngine_SOURCE_DIR}/3rd/*
)

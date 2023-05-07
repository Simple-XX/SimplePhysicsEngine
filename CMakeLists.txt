cmake_minimum_required(VERSION 3.0)

set(CMAKE_CXX_STANDARD 11)

project(
    SimplePhysicsEngine
    VERSION 0.0.1
    DESCRIPTION "A simple physics engine in developing..."
    HOMEPAGE_URL "https://github.com/Simple-XX/SimplePhysicsEngine"
    LANGUAGES CXX C)

# Uncomment to generate different binary files for Debug/Release version
# set(CMAKE_DEBUG_POSTFIX d)

set(CMAKE_INSTALL_PREFIX "${CMAKE_SOURCE_DIR}/install" CACHE STRING "Installation directory of library files")

# Installation directory of library files
set(DTK_INSTALL_LIBDIR "${CMAKE_INSTALL_LIBDIR}")

# Installation directory for CMake configuration files
set(DTK_INSTALL_CMAKEDIR "${CMAKE_BINARY_DIR}")

# Installation directory for header files
set(DTK_INSTALL_INCLUDEDIR "${CMAKE_INSTALL_INCLUDEDIR}")

if("${DTK_INSTALL_LIBDIR}" STREQUAL "")
    set(DTK_INSTALL_LIBDIR "lib")
endif()

if("${DTK_INSTALL_CMAKEDIR}" STREQUAL "")
    set(DTK_INSTALL_CMAKEDIR "bin")
endif()

if("${DTK_INSTALL_INCLUDEDIR}" STREQUAL "")
    set(DTK_INSTALL_INCLUDEDIR "include")
endif()

set(DTK_INSTALL_INCLUDEDIR "include")

message("DTK_INSTALL_LIBDIR : ${DTK_INSTALL_LIBDIR}")
message("DTK_INSTALL_INCLUDEDIR : ${DTK_INSTALL_INCLUDEDIR}")
message("DTK_INSTALL_CMAKEDIR: ${DTK_INSTALL_CMAKEDIR}")

# Set some frequently used vars
set(SPE_ROOT_DIR ${CMAKE_SOURCE_DIR})
set(SPE_DEPS "${CMAKE_SOURCE_DIR}/_deps")

# Build Options
set(VTK_OPT # whether to use VTK
    "OFF"
    CACHE BOOL "Choose whether to use VTK or not")

# Add global definitions to project

add_subdirectory(engine)
add_subdirectory(demo2d)
add_subdirectory(demo3d)

# This file is a part of Simple-XX/SimplePhysicsEngine
# (https://github.com/Simple-XX/SimplePhysicsEngine).
#
# project_config.cmake for Simple-XX/SimplePhysicsEngine.
# 项目配置


# Uncomment to generate different binary files for Debug/Release version
# set(CMAKE_DEBUG_POSTFIX d)

set(CMAKE_INSTALL_PREFIX "${CMAKE_SOURCE_DIR}/install" CACHE STRING "Installation directory of library files")

# Installation directory of library files
set(DTK_INSTALL_LIBDIR "${CMAKE_INSTALL_LIBDIR}")

# Installation directory for CMake configuration files
set(DTK_INSTALL_CMAKEDIR "${CMAKE_BINARY_DIR}")

# Installation directory for header files
set(DTK_INSTALL_INCLUDEDIR "${CMAKE_INSTALL_INCLUDEDIR}")

if ("${DTK_INSTALL_LIBDIR}" STREQUAL "")
    set(DTK_INSTALL_LIBDIR "lib")
endif ()

if ("${DTK_INSTALL_CMAKEDIR}" STREQUAL "")
    set(DTK_INSTALL_CMAKEDIR "bin")
endif ()

if ("${DTK_INSTALL_INCLUDEDIR}" STREQUAL "")
    set(DTK_INSTALL_INCLUDEDIR "include")
endif ()

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

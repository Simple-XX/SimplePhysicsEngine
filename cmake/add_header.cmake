
# This file is a part of Simple-XX/SimplePhysicsEngine
# (https://github.com/Simple-XX/SimplePhysicsEngine).
#
# add_header.cmake for Simple-XX/SimplePhysicsEngine.
# 将头文件路径添加到 _target 的搜索路径中

function(add_header_3rd _target)
    target_include_directories(${_target} PRIVATE
            ${tinyobjloader_SOURCE_DIR})
endfunction()

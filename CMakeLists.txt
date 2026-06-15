cmake_minimum_required(VERSION 3.16)

# Reuse the WT99P4C5-S1 BSP and the bsp_extra from p4_camera_stream.
# The components in EXTRA_COMPONENT_DIRS are added in addition to "main" and
# the components pulled in via the component manager.
set(EXTRA_COMPONENT_DIRS
    "${CMAKE_CURRENT_SOURCE_DIR}/components/wt99p4c5_s1_board"
    "${CMAKE_CURRENT_SOURCE_DIR}/components/bsp_extra"
)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(p4_face_stream)

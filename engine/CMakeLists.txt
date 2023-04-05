cmake_minimum_required(VERSION 3.8)


file(GLOB_RECURSE source_files 
${CMAKE_CURRENT_SOURCE_DIR}/*.cpp
${CMAKE_CURRENT_SOURCE_DIR}/*.h)

add_library(engine_core SHARED  ${source_files})

target_include_directories(engine_core PUBLIC ${CAMEK_CURRENT_SOURCE_DIR})
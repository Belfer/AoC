cmake_minimum_required (VERSION 3.16)

set (CMAKE_CXX_STANDARD 11)
set (CMAKE_CXX_STANDARD_REQUIRED ON)

project (AoC16)

add_compile_definitions (WD="${CMAKE_CURRENT_SOURCE_DIR}")
add_executable (app "main.cpp")
target_include_directories (app PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
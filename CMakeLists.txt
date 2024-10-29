cmake_minimum_required(VERSION 3.20)
set(CMAKE_CXX_STANDARD 20)

project(out)

find_package(Open3D REQUIRED)
find_package(spdlog REQUIRED)

add_executable(out src/main.cpp)

target_link_libraries(out PRIVATE Open3D::Open3D spdlog)
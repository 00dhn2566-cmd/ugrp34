# controller_cpp — 크로스 머신 빌드 (Gazebo 검증 머신용; Windows는 build.ps1로도 가능)
# 사용: cmake -B build && cmake --build build && ./build/qc_trace --smoke
cmake_minimum_required(VERSION 3.16)
project(qc_controller CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

add_library(qc STATIC src/qc_controller.cpp src/qc_io.cpp)
target_include_directories(qc PUBLIC include)
target_compile_options(qc PRIVATE -Wall -Wextra)

add_executable(qc_trace src/main_trace.cpp)
target_link_libraries(qc_trace PRIVATE qc)

# Gazebo 플러그인/노드는 이 라이브러리(qc)를 링크해서 qc_step()/qc_io를 사용하면 된다.

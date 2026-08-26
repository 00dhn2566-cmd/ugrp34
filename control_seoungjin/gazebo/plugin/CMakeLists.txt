# qc_gz_controller — Gazebo Harmonic (gz-sim8) 시스템 플러그인
#
#   cmake -B build -DCMAKE_BUILD_TYPE=Release
#   cmake --build build -j
#   -> build/libqc_gz_controller.so
#
# 필요한 패키지 (Ubuntu 24.04 + Gazebo Harmonic):
#   sudo apt install -y libgz-sim8-dev libgz-plugin2-dev libgz-math7-dev libgz-common5-dev libsdformat14-dev
# (ros-jazzy-ros-gz 는 런타임만 깔아 주므로 -dev 는 따로 필요하다. setup_gazebo.sh 가 처리한다.)
#
# 제어기 본체는 ../../controller_cpp 를 그대로 서브디렉토리로 끌어온다 — 복사본을
# 만들면 두 벌이 갈라진다. MATLAB/Simscape 는 필요 없다.

cmake_minimum_required(VERSION 3.16)
project(qc_gz_controller CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

# --- 제어기 라이브러리 (controller_cpp 의 CMakeLists 를 재사용) ---
get_filename_component(QC_CPP_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../controller_cpp" ABSOLUTE)
if(NOT EXISTS "${QC_CPP_DIR}/CMakeLists.txt")
  message(FATAL_ERROR "controller_cpp 를 못 찾음: ${QC_CPP_DIR}\n"
                      "이 폴더는 control_seoungjin/ 트리 안에 있어야 한다.")
endif()
add_subdirectory("${QC_CPP_DIR}" "${CMAKE_CURRENT_BINARY_DIR}/controller_cpp")

# --- Gazebo ---
find_package(gz-sim8 REQUIRED)
find_package(gz-plugin2 REQUIRED COMPONENTS register)
find_package(gz-math7 REQUIRED)
find_package(gz-common5 REQUIRED)
find_package(sdformat14 REQUIRED)

add_library(qc_gz_controller SHARED QcGzController.cc)
target_include_directories(qc_gz_controller PRIVATE "${QC_CPP_DIR}/include")
target_link_libraries(qc_gz_controller
  PRIVATE
    qc
    gz-sim8::gz-sim8
    gz-plugin2::register
    gz-math7::gz-math7
    gz-common5::gz-common5
    sdformat14::sdformat14
)
target_compile_options(qc_gz_controller PRIVATE -Wall -Wextra)

# gz sim 이 찾는 이름은 파일명(libqc_gz_controller.so) 이다 — SDF 의
# filename="qc_gz_controller" 와 맞춰야 한다. 접두사/접미사는 기본값 그대로 둘 것.
message(STATUS "빌드 후: export GZ_SIM_SYSTEM_PLUGIN_PATH=${CMAKE_CURRENT_BINARY_DIR}")

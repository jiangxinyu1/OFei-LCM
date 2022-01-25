cmake_minimum_required(VERSION 2.8)
set(CMAKE_BUILD_TYPE debug)
project(depthmap_demo)

#SET(CMAKE_C_COMPILER "/opt/gcc_aarch64/bin/aarch64-linux-gnu-gcc")
#SET(CMAKE_CXX_COMPILER "/opt/gcc_aarch64/bin/aarch64-linux-gnu-g++")


#SET(DATA_DIR "/mnt/sdcard/pmd")
#add_definitions(-DDATA_DIR="${DATA_DIR}")

LINK_DIRECTORIES("${PROJECT_SOURCE_DIR}/third")
add_executable(${PROJECT_NAME} camerademo.c common.c calibration.c)

target_link_libraries(${PROJECT_NAME} depthcomputation depthmapwrapper spectre)

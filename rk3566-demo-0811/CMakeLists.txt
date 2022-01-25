cmake_minimum_required(VERSION 2.8)
set(CMAKE_BUILD_TYPE debug)
project(depthmap_demo)

set(Thirdparty_DIR ~/cleanRobot/thirdparty)
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${Thirdparty_DIR}/CMakeModules")
set(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -std=c++11 -Wall -Wno-unused-result")
SET( EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin) 



find_package(lcm REQUIRED) 
# find_package(OpenCV REQUIRED)

include_directories(
    ${INCLUDE_DIRS}
    ${Thirdparty_DIR}/include
    ${Thirdparty_DIR}/include/lcm_msgs
    ${Thirdparty_DIR}/include/lcm_msgs/lcm_sensor_msgs
    ${Thirdparty_DIR}/include/lcm_msgs/lcm_geometry_msgs
    ${LCM_INCLUDE_DIR}
)

# include_directories(${OpenCV_INCLUDE_DIRS})
# link_directories(${OpenCV_LIBRARY_DIRS})

LINK_DIRECTORIES("${PROJECT_SOURCE_DIR}/third")

add_executable(depthmap_demo  main.cpp imagesHandler.cpp common.c calibration.c camerademo.c )
target_link_libraries(depthmap_demo  ${LCM_LIBRARY}  depthcomputation depthmapwrapper spectre )


add_executable(depthmap_demo_LCM main_lcm.cpp imagesHandler.cpp common.c calibration.c camerademo.c )
target_link_libraries(depthmap_demo_LCM ${LCM_LIBRARY} depthcomputation depthmapwrapper spectre )
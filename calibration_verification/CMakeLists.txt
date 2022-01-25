cmake_minimum_required(VERSION 3.10)

#project name
PROJECT(calibration_verification)

set( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17")

# 设置DEBUG编译模式的宏定义
IF (CMAKE_BUILD_TYPE STREQUAL Debug)
    ADD_DEFINITIONS(-DDISPLAY)
    ADD_DEFINITIONS(-DPCL_DISPLAY)
ENDIF()

# 设置可执行文件的目录
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

#head file path
include_directories("/usr/include/eigen3")
#source directory
AUX_SOURCE_DIRECTORY(./src/planeDetection/  planeDetectionSourceList)

find_package(roscpp REQUIRED)
find_package(OpenCV REQUIRED)
find_package(PCL 1.3 REQUIRED )
find_package(message_filters REQUIRED)
find_package(gflags REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)



link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})


INCLUDE_DIRECTORIES(
    include
    ${PROJECT_SOURCE_DIR}/src
    ${OpenCV_INCLUDE_DIRS}
    ${roscpp_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
    ${message_filters_INCLUDE_DIRS}
)
INCLUDE_DIRECTORIES(   ${roscpp_INCLUDE_DIRS} )
INCLUDE_DIRECTORIES( ${gflags_INCLUDE_DIR})

#add executable file
ADD_EXECUTABLE( calibration_verification  src/calibration_verification.cpp ${planeDetectionSourceList})
# directory of opencv library
link_directories(${OpenCV_LIBRARY_DIRS})
# opencv librarie
TARGET_LINK_LIBRARIES( calibration_verification ${LIBRARIES} ${OpenCV_LIBS} ${PCL_LIBRARIES} ${roscpp_LIBRARIES} ${message_filters_LIBRARIES} gflags)

ADD_EXECUTABLE( convertedPointCloudPuber_node  src/convertedPointCloudPuber.cpp)
TARGET_LINK_LIBRARIES( convertedPointCloudPuber_node ${LIBRARIES}  ${PCL_LIBRARIES} ${roscpp_LIBRARIES}  gflags)

ADD_EXECUTABLE( ir_processed_node  src/irProcess.cpp)
TARGET_LINK_LIBRARIES( ir_processed_node ${LIBRARIES}  ${OpenCV_LIBS} ${roscpp_LIBRARIES}  gflags)

ADD_EXECUTABLE( showErrorPoints_node  src/showErrorPoints.cpp)
TARGET_LINK_LIBRARIES( showErrorPoints_node ${LIBRARIES} ${OpenCV_LIBS} ${PCL_LIBRARIES} ${roscpp_LIBRARIES} ${message_filters_LIBRARIES} gflags)

ADD_EXECUTABLE( showPointCLoud_node  src/showPointCloud.cpp)
TARGET_LINK_LIBRARIES( showPointCLoud_node ${LIBRARIES}  ${PCL_LIBRARIES} gflags)

ADD_EXECUTABLE( showGround_node  src/showGround.cpp)
TARGET_LINK_LIBRARIES( showGround_node ${LIBRARIES} ${OpenCV_LIBS} ${PCL_LIBRARIES} ${roscpp_LIBRARIES} ${message_filters_LIBRARIES} gflags)
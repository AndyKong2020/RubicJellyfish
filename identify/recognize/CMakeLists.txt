cmake_minimum_required(VERSION 3.0.2)
project(recognize)


add_compile_options(-std=c++14)

# 目前支持的识别方法：MobileNet, BP, Template
set(RECOGNITION_METHOD "MobileNet")
# 目前支持的后端引擎：openvino, tensorrt, 也可以设置为auto或none
set(NN_BACKEND "openvino")

include(src/DigitalRecognition/path.cmake)
include(src/DigitalRecognition/func.cmake)

detectBackend()
detectMethod()

find_package(
        catkin REQUIRED COMPONENTS
        cv_bridge
        image_transport
        roscpp
        sensor_msgs
        std_msgs
        geometry_msgs
)
find_package(OpenCV 4 REQUIRED)
find_package(Zbar0)
find_package(Eigen3 REQUIRED)
find_package(Ceres REQUIRED)
## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

IF (RTYPE STREQUAL SENTRY)
    ADD_DEFINITIONS(-DSENTRY)
ENDIF ()
#RelWithDebInfo
set(ROS_BUILD_TYPE Debug)
set(CMAKE_BUILD_TYPE Debug)

catkin_package(
        #  INCLUDE_DIRS include
        #  LIBRARIES windMill
        #  CATKIN_DEPENDS cv_bridge image_transport roscpp sensor_msgs std_msgs
        #  DEPENDS system_lib
)

add_subdirectory(src/DigitalRecognition)

include_directories(
        include
        ${catkin_INCLUDE_DIRS}
        ${EIGEN3_INCLUDE_DIRS}
        src/DigitalRecognition
)

add_executable(
        ${PROJECT_NAME}_node
        src/recognize.cpp
        src/image_recognization.cpp
        src/image_processing.cpp
        src/yolov5.cpp
        src/yolox.cpp)
        
        
target_link_libraries(
        ${PROJECT_NAME}_node
        ${OpenCV_LIBRARIES}
        ${catkin_LIBRARIES}
        ${CERES_LIBRARIES}
        ${ZBAR_LIBS} zbar
        DigitalRecognitionLib
)

install(
        TARGETS ${PROJECT_NAME}_node
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Mark cpp header files for installation
install(
        DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
        FILES_MATCHING PATTERN "*.h"
        PATTERN ".svn" EXCLUDE
)


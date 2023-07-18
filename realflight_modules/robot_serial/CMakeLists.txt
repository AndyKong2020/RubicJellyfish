cmake_minimum_required(VERSION 3.0.2)
project(serial_robot)

set(CMAKE_CXX_STANDARD 17)
add_compile_options(-std=c++17)
find_package(
        catkin REQUIRED COMPONENTS
        geometry_msgs
        nav_msgs
        message_generation
        message_runtime
        roscpp
)
add_subdirectory(serialib)

add_message_files(
        FILES
        aim.msg
        exchange.msg
        kalman.msg
        gimbalPRY.msg
        gimbal.msg
        spinning_control.msg
        dialSwitch.msg
)

generate_messages(
        DEPENDENCIES
        std_msgs
)

catkin_package(
        #  INCLUDE_DIRS include
        #  LIBRARIES serial_port
        CATKIN_DEPENDS geometry_msgs roscpp std_msgs
        #  DEPENDS system_lib
)

include_directories(
        ${catkin_INCLUDE_DIRS}
        serialib/
        include
)

add_executable(${PROJECT_NAME}_node src/main.cpp)

add_dependencies(
        ${PROJECT_NAME}_node
        serial_robot_gencpp
)

target_link_libraries(
        ${PROJECT_NAME}_node
        ${catkin_LIBRARIES}
        serialib
)

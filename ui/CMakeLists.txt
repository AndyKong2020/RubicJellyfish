project(ui)
# QT自动MOC需要cmake3.1.0以上版本支持
cmake_minimum_required(VERSION 3.1.0)
add_definitions(-DRESOURCE_PATH="${PROJECT_SOURCE_DIR}/resources/")

# 启用自动MOC
set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_CXX_STANDARD 17)
Set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIC")

# 设置QT文件位置
include_directories(widgets)
set(QT_SOURCES
        src/main_window.cpp
        src/main.cpp
        src/qnode.cpp
        src/rotation_recongnition.cpp
        )

set(QT_HEADERS
        include/ui/main_window.hpp
        include/ui/qnode.hpp
        )

set(QT_UI_FILES
        ui/main_window.ui
        )

# 如果找不到QT5头文件请修改下面两行地址并解除注释
#set(Qt5_DIR "/home/bismarck/Qt5.9.9/5.9.9/gcc_64/lib/cmake/Qt5")
#set(Qt5_Widgets "/home/bismarck/Qt5.9.9/5.9.9/gcc_64/lib/cmake/Qt5Widgets")

find_package(Qt5 COMPONENTS Core Widgets Gui Core REQUIRED)
find_package(catkin REQUIRED COMPONENTS roscpp cv_bridge image_transport rc_msgs dynamic_reconfigure)
find_package(OpenCV 4 REQUIRED)

# 设置include路径
include_directories(
        ${Qt5Widgets_INCLUDE_DIRS}
)


# 编译为ros包
catkin_package(CATKIN_DEPENDS roscpp mv_driver neural_net)

# 编译UI布局文件
if (Qt5_FOUND)
    QT5_WRAP_CPP(QT_HEADERS_MOC ${QT_HEADERS})
    QT5_WRAP_UI(QT_UI_FILES_MOC ${QT_UI_FILES})
else ()
    message(FATAL_ERROR "Qt not found. So cannot moc.")
endif ()

# 添加QT相关库
add_library(qt_widgets
        ${QT_SOURCES}
        ${QT_UI_FILES_MOC}
        ${QT_HEADERS_MOC}
        )

# 加入UI布局文件编译产生的头文件
include_directories(
        include
        ${CMAKE_CURRENT_BINARY_DIR}
        ${catkin_INCLUDE_DIRS}
        /usr/local/include)

# 连接编译文件
add_executable(${PROJECT_NAME}_node ${QT_HEADERS_MOC} ${QT5_WRAP_UI} ${QT_SOURCES})

add_dependencies(${PROJECT_NAME}_node rc_msgs_generate_messages)

target_link_libraries(${PROJECT_NAME}_node
        ${catkin_LIBRARIES}
        ${OpenCV_LIBRARIES}
        Qt5::Widgets)

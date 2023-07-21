cmake_minimum_required(VERSION 3.0.2)
project(rc_msgs)

find_package(
        catkin REQUIRED COMPONENTS
        message_generation
        message_runtime
        roscpp
        sensor_msgs
        std_msgs
        dynamic_reconfigure
        rospy
)

add_message_files(
        FILES
        point.msg
        detection.msg
        results.msg
        single_result.msg
        identify_results.msg
        calibrateResult.msg
)
generate_dynamic_reconfigure_options(
        cfg/stepConfig.cfg
)
generate_messages(
        DEPENDENCIES
        sensor_msgs#   std_msgs
)

catkin_package(
        #  INCLUDE_DIRS include
        #  LIBRARIES rc_msgs
        #  CATKIN_DEPENDS message_generation message_runtime roscpp senser_msgs std_msgs
        #  DEPENDS system_lib
)

include_directories(
        # include
        ${catkin_INCLUDE_DIRS}
)

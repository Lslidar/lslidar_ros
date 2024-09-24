cmake_minimum_required(VERSION 2.8.3)
project(lslidar_ls_driver)

add_definitions(-std=c++14)
set(libpcap_LIBRARIES -lpcap)

set(CMAKE_BUILD_TYPE Release)
message("cmake_build_type:"  ${CMAKE_BUILD_TYPE})

find_package(catkin REQUIRED COMPONENTS
        roscpp
        pcl_ros
        pcl_conversions
        nodelet
        dynamic_reconfigure
        message_generation
        std_msgs
        sensor_msgs
        )

# 添加消息和服务文件
add_message_files(
        FILES
        LslidarLsPacket.msg
)

add_service_files(
        FILES
        angle_distortion_correction.srv
        frame_rate.srv
        data_ip.srv
        destination_ip.srv
        data_port.srv
        dev_port.srv
        invalid_data.srv
        standby_mode.srv
        time_service.srv
)

generate_messages(
        DEPENDENCIES std_msgs sensor_msgs
)

catkin_package(
        INCLUDE_DIRS include
        CATKIN_DEPENDS
        roscpp nodelet pcl_ros pcl_conversions std_msgs sensor_msgs
)

include_directories(
        include
        ${catkin_INCLUDE_DIRS}
)

link_directories(
        ${catkin_LIBRARY_DIRS}
)

add_library(lslidar_ls_input
        src/input.cc
)

target_link_libraries(lslidar_ls_input
        ${catkin_LIBRARIES}
        ${libpcap_LIBRARIES}
)

# Leishen ls lidar driver
add_library(lslidar_ls_driver
        src/lslidar_ls_driver.cc
)

target_link_libraries(lslidar_ls_driver
        lslidar_ls_input
        ${catkin_LIBRARIES}
)

add_dependencies(lslidar_ls_driver
        ${${PROJECT_NAME}_EXPORTED_TARGETS}
        ${catkin_EXPORTED_TARGETS}
)

# Leishen ls lidar nodelet
add_executable(lslidar_ls_driver_node
        src/lslidar_ls_driver_node.cc
)

target_link_libraries(lslidar_ls_driver_node
        lslidar_ls_driver
        ${catkin_LIBRARIES}
)

add_dependencies(lslidar_ls_driver_node
        ${${PROJECT_NAME}_EXPORTED_TARGETS}
        ${catkin_EXPORTED_TARGETS}
)

if (catkin_EXPORTED_TARGETS)
    add_dependencies(lslidar_ls_input ${catkin_EXPORTED_TARGETS})
endif ()

# install options
install(TARGETS lslidar_ls_input lslidar_ls_driver lslidar_ls_driver_node
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

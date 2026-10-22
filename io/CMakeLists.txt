cmake_minimum_required(VERSION 3.16)

find_package(yaml-cpp REQUIRED)

add_subdirectory(serial)

# 创建目标 io
add_library(io STATIC 
    hikrobot/hikrobot.cpp    
    mindvision/mindvision.cpp  
    usbcamera/usbcamera.cpp  
    camera.cpp
    cboard.cpp
    cboard_motor_pose_buffer.cpp
    damiao_can.cpp
    dm_imu/dm_imu.cpp
    imu_huaner/imu_huaner.cpp
    imu_huaner/huaner_pose_buffer.cpp
    imu_fxxk/hipnuc_imu.cpp
    imu_fxxk/hipnuc_pose_buffer.cpp
    imu_fxxk/hipnuc_dec.c
    gimbal/gimbal.cpp
    can_bridge/aim_bridge_sender.cpp
)

# hikrobot
target_include_directories(io PUBLIC hikrobot/include)
if(CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64")
  target_link_directories(io PUBLIC hikrobot/lib/amd64)
elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64")
  target_link_directories(io PUBLIC hikrobot/lib/arm64)
else()
  message(FATAL_ERROR "Unsupported architecture: ${CMAKE_HOST_SYSTEM_PROCESSOR}!")
endif()

# mindvision
target_include_directories(io PUBLIC mindvision/include)
if(CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64")
  target_link_directories(io PUBLIC mindvision/lib/amd64)
elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64")
  target_link_directories(io PUBLIC mindvision/lib/arm64)
else()
  message(FATAL_ERROR "Unsupported architecture: ${CMAKE_HOST_SYSTEM_PROCESSOR}!")
endif()

# 达妙 DM-USB2FDCAN SDK（can_interface: "damiao" 时使用）
target_link_directories(io PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/damiao/lib/linux)
target_link_libraries(io MvCameraControl MVSDK usb-1.0 dm_device yaml-cpp serial)

################## 检查 ROS 环境 ##################
find_package(ament_cmake QUIET)
find_package(rclcpp QUIET)
find_package(std_msgs QUIET)
find_package(fastcdr QUIET CONFIG)
find_package(rosidl_typesupport_cpp QUIET)
find_package(sp_msgs QUIET)

# can_bridge sender is part of the standard launch path.
if(NOT ament_cmake_FOUND OR NOT rclcpp_FOUND OR NOT std_msgs_FOUND OR NOT fastcdr_FOUND)
    message(FATAL_ERROR "ROS2 rclcpp/std_msgs not found; can_bridge sender requires ROS2.")
endif()
ament_target_dependencies(io rclcpp std_msgs)

# 如果找不到完整导航 ROS 环境，则跳过原有 io/ros2 导航相关代码
if(NOT rosidl_typesupport_cpp_FOUND OR NOT sp_msgs_FOUND)
    message(WARNING "sp_msgs not found, skipping legacy ROS2 navigation helpers.")
else()
    message(STATUS "ROS2 environment found, compiling ROS2-related code.")
    
    # 添加 ROS2 相关的文件
    target_sources(io PRIVATE ros2/publish2nav.cpp ros2/ros2.cpp ros2/subscribe2nav.cpp)

    # 设置 ROS2 依赖
    include_directories(${std_msgs_INCLUDE_DIRS})
    ament_target_dependencies(io sp_msgs)
endif()

cmake_minimum_required(VERSION 3.16)
project( ros2_common_utilities LANGUAGES CXX)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -g")
# SET(CMAKE_BUILD_TYPE Debug)

#==============================================================================
# Dependencies
#==============================================================================

set(INCLUDE_DEPENDS
    ament_cmake
    interactive_markers
    rclcpp
    rclcpp_components
    roboy_middleware_msgs
    std_msgs
    tf2
    tf2_ros
    visualization_msgs
)

find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
include_directories(${Eigen3_INCLUDE_DIRS})

# yaml-cpp
find_package(yaml-cpp REQUIRED yaml-cpp)
include_directories(${yaml-cpp_INCLUDE_DIRS})

include_directories( include )

#==============================================================================
# Application
#==============================================================================
add_library(UDPSocket src/UDPSocket.cpp)
ament_target_dependencies( UDPSocket ${INCLUDE_DEPENDS} )

add_executable( ROS_MASTER_URI_broadcaster src/ROS_MASTER_URI_broadcaster.cpp)
# ament_target_dependencies( ROS_MASTER_URI_broadcaster ${INCLUDE_DEPENDS} )
target_link_libraries( ROS_MASTER_URI_broadcaster UDPSocket )

add_executable( ROS_MASTER_URI_receiver src/ROS_MASTER_URI_receiver.cpp)
# ament_target_dependencies( ROS_MASTER_URI_receiver ${INCLUDE_DEPENDS} )
target_link_libraries( ROS_MASTER_URI_receiver UDPSocket )

add_library( rviz_visualization src/rviz_visualization.cpp )
ament_target_dependencies(rviz_visualization PUBLIC ${INCLUDE_DEPENDS} )

add_library(motor_config SHARED src/MotorConfig.cpp) # include/ros2_common_utilities/MotorConfig.hpp include/ros2_common_utilities/CommonDefinitions.h)
ament_target_dependencies(motor_config PUBLIC ${INCLUDE_DEPENDS} )
target_link_libraries( motor_config ${yaml-cpp_LIBRARIES} )

add_executable( rfid_unlocker src/rfid_unlocker.cpp )
target_link_libraries( rfid_unlocker UDPSocket )

#==============================================================================
# Installation
#==============================================================================

install(TARGETS motor_config
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
        )

install(
  DIRECTORY include/
  DESTINATION include/ros2_common_utilities
)
install(
    TARGETS motor_config
    RUNTIME DESTINATION lib/ros2_common_utilities
)

ament_export_dependencies(${INCLUDE_DEPENDS})
ament_package()
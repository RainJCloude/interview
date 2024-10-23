cmake_minimum_required(VERSION 3.8)
project(points_from_1_to_7)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
#find_package(ament_cmake_python REQUIRED)

find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_geometry REQUIRED)
find_package(image_transport REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(message_filters REQUIRED)

add_executable(pb_frames src/publish_frames.cpp)
ament_target_dependencies(
    pb_frames
    geometry_msgs
    rclcpp
    tf2
    tf2_ros
)

add_executable(transform src/transform.cpp)
ament_target_dependencies(
    transform
    geometry_msgs
    rclcpp
    tf2
    tf2_ros
)

add_executable(drawer src/drawer.cpp)
ament_target_dependencies(
    drawer
    geometry_msgs
    rclcpp
    tf2
    tf2_ros
    sensor_msgs
    cv_bridge
    OpenCV
    image_geometry
    image_transport
    message_filters
)

add_executable(im_pub src/im_pub.cpp)
ament_target_dependencies(
    im_pub
    geometry_msgs
    rclcpp
    sensor_msgs
    cv_bridge
    OpenCV
    image_transport
    ament_index_cpp
)

install(TARGETS
    pb_frames drawer transform im_pub
    DESTINATION lib/${PROJECT_NAME})
    
install(
  DIRECTORY config launch img
  DESTINATION share/${PROJECT_NAME}
)
    
ament_package()

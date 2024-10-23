cmake_minimum_required(VERSION 3.8)
project(point_8)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
 

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

 


add_executable(ray_tracer src/trace3dPoint.cpp)
ament_target_dependencies(
    ray_tracer
    geometry_msgs
    rclcpp
    sensor_msgs
    cv_bridge
    OpenCV
    tf2
    tf2_ros
)
 
install(TARGETS
    ray_tracer
    DESTINATION lib/${PROJECT_NAME})
    
install(
  DIRECTORY config img
  DESTINATION share/${PROJECT_NAME}
)
    
ament_package()

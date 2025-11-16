cmake_minimum_required(VERSION 3.8)
project(group14_ex4)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(group14_interfaces REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

#sever
add_executable(turtlenode_server src/turtlenode_server.cpp)
ament_target_dependencies(turtlenode_server
  "rclcpp"
  "group14_interfaces"
  "sensor_msgs"
  "geometry_msgs"
)

#client
add_executable(burrow_client src/burrow_client.cpp) 
ament_target_dependencies(burrow_client
  "rclcpp"
  "group14_interfaces"
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS
  turtlenode_server
  burrow_client
  DESTINATION lib/${PROJECT_NAME}
)

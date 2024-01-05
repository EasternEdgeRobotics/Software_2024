cmake_minimum_required(VERSION 3.5)
project(eer_healthcheck)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(healthCheckResponse src/healthCheckResponse.cpp)
ament_target_dependencies(healthCheckResponse rclcpp std_msgs)
add_executable(healthCheckCaller src/healthCheckCaller.cpp)
ament_target_dependencies(healthCheckCaller rclcpp std_msgs)

install(TARGETS
  healthCheckResponse
  healthCheckCaller
  DESTINATION lib/${PROJECT_NAME})

ament_package()
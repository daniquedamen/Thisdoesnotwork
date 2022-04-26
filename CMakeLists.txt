cmake_minimum_required(VERSION 3.8)
project(dynamics_package)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(input_package REQUIRED)
find_package(message_package REQUIRED)    

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

add_executable(dynamics src/Dynamics.cpp)
ament_target_dependencies(dynamics rclcpp message_package)


install(TARGETS
  dynamics
  DESTINATION lib/${PROJECT_NAME})


ament_package()

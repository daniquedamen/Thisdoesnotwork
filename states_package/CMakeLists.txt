cmake_minimum_required(VERSION 3.8)
project(states_package)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(message_package REQUIRED)
find_package(input_package REQUIRED)
find_package(dynamics_package REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()


add_executable(test_states src/states.cpp)
ament_target_dependencies(test_states rclcpp message_package)


install(TARGETS
  test_states
  DESTINATION lib/${PROJECT_NAME})

ament_package()

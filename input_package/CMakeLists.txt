cmake_minimum_required(VERSION 3.8)
project(input_package)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(message_package REQUIRED)     
find_package(SDL2 REQUIRED)
string(STRIP ${SDL2_LIBRARIES} SDL2_LIBRARIES)

# Keyboard_input executable
#add_executable(keyboard_input 
 # src/keyboard_input.cpp
  #src/keyboard_input_sdl.cpp)
#ament_target_dependencies(keyboard_input rclcpp std_msgs)
#target_include_directories(keyboard_input PUBLIC
#  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
 # $<INSTALL_INTERFACE:include>)
#target_compile_features(keyboard_input PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
#target_link_libraries(keyboard_input  ${SDL2_LIBRARIES})

add_executable(input_publisher src/input_node.cpp)
ament_target_dependencies(input_publisher rclcpp message_package)


install(TARGETS
  input_publisher
 # keyboard input

  DESTINATION lib/${PROJECT_NAME})


ament_package()







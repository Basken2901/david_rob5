cmake_minimum_required(VERSION 3.5)
project(david_controller)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED True)

if (POLICY CMP0072)
  cmake_policy(SET CMP0072 NEW)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(OpenGL REQUIRED)

include_directories(include)

add_executable(controller
    src/main.cpp
)

target_include_directories(controller PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(controller 
  OpenGL::GL 
)

ament_target_dependencies(controller
  rclcpp
  rclcpp_action
  std_msgs
)

install(TARGETS controller
  DESTINATION lib/${PROJECT_NAME}
)
# Install config files
install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

ament_package()
cmake_minimum_required(VERSION 3.8)
project(robstride_hardware_interface)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 의존성 패키지 찾기
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(roa_interfaces REQUIRED)

add_library(${PROJECT_NAME}_lib
  src/CanTransport.cpp
  src/RobStrideProtocol.cpp
  src/RobStrideMotor.cpp
  src/filter.cpp
)

target_include_directories(${PROJECT_NAME}_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(${PROJECT_NAME}_lib
  rclcpp
)

add_executable(hardware_interface_node
  src/MainControl.cpp
)

target_include_directories(hardware_interface_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(hardware_interface_node
  ${PROJECT_NAME}_lib
)

ament_target_dependencies(hardware_interface_node
  rclcpp
  rclcpp_lifecycle
  lifecycle_msgs
  std_msgs
  roa_interfaces
)

install(TARGETS
  ${PROJECT_NAME}_lib
  hardware_interface_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/can_setup.sh
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

ament_package()

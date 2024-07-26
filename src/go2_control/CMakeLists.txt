cmake_minimum_required(VERSION 3.8)
project(go2_control)

set(CMAKE_BUILD_TYPE "RelWithDebInfo")

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(dua_app_management REQUIRED)
find_package(dua_interfaces REQUIRED)
find_package(dua_node REQUIRED)
find_package(dua_qos_cpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(image_transport REQUIRED)
find_package(params_manager REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(unitree_api REQUIRED)
find_package(unitree_go REQUIRED)
find_package(unitree_sdk2 REQUIRED)

# Generate parameters source code
generate_init_parameters(
  YAML_FILE "${CMAKE_CURRENT_SOURCE_DIR}/src/go2_control/params.yaml"
  OUT_FILE "init_parameters.cpp")

# Configure Go2 Control node source files
file(GLOB GO2_CONTROL_NODE_SRCS
  RELATIVE "${CMAKE_CURRENT_SOURCE_DIR}"
  "src/go2_control/*.cpp")
list(APPEND GO2_CONTROL_NODE_SRCS init_parameters.cpp)

# Go2 Control node
add_library(go2_control SHARED ${GO2_CONTROL_NODE_SRCS})
target_compile_definitions(go2_control PRIVATE COMPOSITION_BUILDING_DLL)
target_include_directories(go2_control PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(go2_control PUBLIC c_std_99 cxx_std_17) # Require C99 and C++17
ament_target_dependencies(go2_control
  dua_interfaces
  dua_node
  dua_qos_cpp
  geometry_msgs
  image_transport
  params_manager
  rclcpp
  rclcpp_action
  rclcpp_components
  sensor_msgs
  std_msgs
  tf2
  tf2_eigen
  tf2_ros
  unitree_api
  unitree_go
  unitree_sdk2)
rclcpp_components_register_nodes(go2_control "go2_control::Go2Control")

# Go2 Control standalone application
add_executable(go2_control_app
  src/go2_control_app.cpp)
target_include_directories(go2_control_app PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(go2_control_app PUBLIC c_std_99 cxx_std_17) # Require C99 and C++17
target_link_libraries(go2_control_app go2_control)
ament_target_dependencies(go2_control_app
  dua_app_management
  rclcpp)

# Install Go2 Control component
install(TARGETS go2_control
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

# Install Go2 Control application
install(TARGETS go2_control_app
  DESTINATION lib/${PROJECT_NAME})

# Install Config files
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME})

# Install Launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

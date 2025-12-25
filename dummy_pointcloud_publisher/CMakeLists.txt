cmake_minimum_required(VERSION 3.5)
project(dummy_pointcloud_publisher)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
  set(CMAKE_CXX_EXTENSIONS OFF)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

# ---- add component library ----
ament_auto_add_library(dummy_pointcloud_publisher_component SHARED
  src/dummy_pointcloud_publisher_node.cpp
)

target_include_directories(dummy_pointcloud_publisher_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

rclcpp_components_register_node(dummy_pointcloud_publisher_component
  PLUGIN "dummy_pointcloud_publisher::DummyPointCloudPublisher"
  EXECUTABLE dummy_pointcloud_publisher_node
)

install(DIRECTORY include/
  DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package(INSTALL_TO_SHARE
  config
  launch
)

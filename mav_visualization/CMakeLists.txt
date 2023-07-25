cmake_minimum_required(VERSION 3.5)
project(mav_visualization)

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

set(CMAKE_MACOSX_RPATH 0)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_auto REQUIRED)
find_package(rclcpp REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

# Include directories
include_directories(
  include
)

# MAV Viz Library
add_library(${PROJECT_NAME}
  src/marker_group.cpp
  src/leica_marker.cpp
  src/hexacopter_marker.cpp
)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  visualization_msgs
)

target_link_libraries(${PROJECT_NAME} Eigen3::Eigen)

# Add executables
add_executable(hexacopter_publisher src/hexacopter_publisher.cpp)
target_link_libraries(hexacopter_publisher ${PROJECT_NAME})
ament_target_dependencies(hexacopter_publisher rclcpp)

add_executable(leica_publisher src/leica_publisher.cpp)
target_link_libraries(leica_publisher ${PROJECT_NAME})
ament_target_dependencies(leica_publisher rclcpp)

# Install exported libraries
ament_export_dependencies(Eigen3)
ament_export_targets(export_lib HAS_LIBRARY_TARGET)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_lib
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})

# Install executables
install(TARGETS
  hexacopter_publisher
  leica_publisher
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()


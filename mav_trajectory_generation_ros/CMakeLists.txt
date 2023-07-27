cmake_minimum_required(VERSION 3.5)
project(mav_trajectory_generation_ros)

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

# set(CMAKE_MACOSX_RPATH 0)
set(CMAKE_POSITION_INDEPENDENT_CODE ON) 

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_auto REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(mav_msgs REQUIRED)
find_package(mav_trajectory_generation REQUIRED)
find_package(mav_visualization REQUIRED)
find_package(mav_planning_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)

# Include directories
include_directories(
  include
  ${YAML_CPP_INCLUDE_DIR}
)

# MAV Trajecotry Generation (ROS) Library
add_library(traj_gen_ros SHARED
  src/feasibility_analytic.cpp
  src/feasibility_base.cpp
  src/feasibility_recursive.cpp
  src/feasibility_sampling.cpp
  src/input_constraints.cpp
  src/ros_conversions.cpp
  src/ros_visualization.cpp
)

target_include_directories(traj_gen_ros PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)

ament_target_dependencies(traj_gen_ros
  rclcpp
  std_msgs
  std_srvs
  geometry_msgs
  trajectory_msgs
  nav_msgs
  mav_msgs
  mav_trajectory_generation
  mav_visualization
  mav_planning_msgs
)

target_link_libraries(traj_gen_ros
  ${YAML_CPP_LIBRARIES}
)

# Add executables
add_executable(trajectory_sampler_node src/trajectory_sampler_node.cpp)
target_link_libraries(trajectory_sampler_node traj_gen_ros)

# Install exported libraries
ament_export_targets(export_lib HAS_LIBRARY_TARGET)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS traj_gen_ros
  EXPORT export_lib
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

ament_export_include_directories(include)
ament_export_libraries(traj_gen_ros)

# Install executables
install(TARGETS
  trajectory_sampler_node
  DESTINATION lib/${PROJECT_NAME}
) # Can add other executables later (time_evaluation_node and test_feasibility). For now, getting MVP.

ament_package()

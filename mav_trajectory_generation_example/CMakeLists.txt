cmake_minimum_required(VERSION 3.5)
project(mav_trajectory_generation_example)

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

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_auto REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_srvs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(mav_msgs REQUIRED)
find_package(mav_trajectory_generation REQUIRED)
find_package(mav_trajectory_generation_ros REQUIRED)
find_package(mav_visualization REQUIRED)
find_package(mav_planning_msgs REQUIRED)

include_directories(
  include/
)

# Add executables
# add_executable(trajectory_generation_example src/example_planner.cc src/example_planner_node.cc)
# ament_target_dependencies(trajectory_generation_example mav_msgs mav_trajectory_generation mav_trajectory_generation_ros mav_visualization mav_planning_msgs)

# add_executable(trajectory_generation_6dof_example src/example_planner_6dof.cc src/example_planner_6dof_node.cc)
# ament_target_dependencies(trajectory_generation_6dof_example mav_msgs mav_trajectory_generation mav_trajectory_generation_ros mav_visualization mav_planning_msgs)

add_executable(trajectory_generation_readme_example src/readme_example.cpp)
ament_target_dependencies(trajectory_generation_readme_example visualization_msgs mav_msgs mav_trajectory_generation
                          mav_trajectory_generation_ros mav_visualization mav_planning_msgs)

# Install directories
# install(
#   DIRECTORY include/
#   DESTINATION include
# )

install(DIRECTORY
  config launch
  DESTINATION share/${PROJECT_NAME}  
)

# Install executables
install(TARGETS
  # trajectory_generation_example
  # trajectory_generation_6dof_example
  trajectory_generation_readme_example
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

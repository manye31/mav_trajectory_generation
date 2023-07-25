# cmake_minimum_required(VERSION 2.8.3)
# project(mav_trajectory_generation)

# find_package(catkin_simple REQUIRED)
# catkin_simple(ALL_DEPS_REQUIRED)

# set(CMAKE_MACOSX_RPATH 0)
# add_definitions(-std=c++11)

# # Link against system catkin yaml-cpp if installed.
# find_package(PkgConfig)
# find_package(yaml_cpp_catkin QUIET)
# if(${yaml_cpp_catkin_FOUND})
#     message(STATUS "Found yaml_cpp_catkin, using instead of system library.")
#     set(YamlCpp_LIBRARIES ${yaml_cpp_catkin_LIBRARIES})
#     set(YamlCpp_INCLUDE_DIRS ${yaml_cpp_catkin_INCLUDE_DIRS})
# else()
#     message(STATUS "No yaml_cpp_catkin, using yaml-cpp system library instead.")
#     pkg_check_modules(YamlCpp REQUIRED yaml-cpp>=0.5)
# endif()

# #############
# # LIBRARIES #
# #############
# cs_add_library(${PROJECT_NAME}
#   src/motion_defines.cpp
#   src/polynomial.cpp
#   src/segment.cpp
#   src/timing.cpp
#   src/trajectory.cpp
#   src/trajectory_sampling.cpp
#   src/vertex.cpp
#   src/io.cpp
#   src/rpoly/rpoly_ak1.cpp
# )
# # Link against yaml-cpp.
# target_link_libraries(${PROJECT_NAME} ${YamlCpp_LIBRARIES})

# ############
# # BINARIES #
# ############
# cs_add_executable(polynomial_timing_evaluation
#   src/polynomial_timing_evaluation.cpp
# )
# target_link_libraries(polynomial_timing_evaluation ${PROJECT_NAME})

# #########
# # TESTS #
# #########
# catkin_add_gtest(test_polynomial
#   test/test_polynomial.cpp
# )
# target_link_libraries(test_polynomial ${PROJECT_NAME} ${catkin_LIBRARIES})

# catkin_add_gtest(test_polynomial_optimization
#   test/test_polynomial_optimization.cpp
# )
# target_link_libraries(test_polynomial_optimization ${PROJECT_NAME} ${catkin_LIBRARIES})

# ##########
# # EXPORT #
# ##########
# cs_install()
# cs_export()

########################################

cmake_minimum_required(VERSION 3.5)
project(mav_trajectory_generation)

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
# find_package(ament_cmake_gtest REQUIRED)
find_package(rclcpp REQUIRED)
find_package(Eigen3 3.3 REQUIRED)
# find_package(eigen_catkin REQUIRED)
# find_package(eigen_checks REQUIRED)
# find_package(glog_catkin REQUIRED)
find_package(mav_msgs REQUIRED)
# find_package(nlopt REQUIRED)

# Include directories
include_directories(
  include
  ${YAML_CPP_INCLUDE_DIR}
  # ${NLOPT_INCLUDE_DIRS}
)

# Trajectory Optimization library
add_library(${PROJECT_NAME}
  src/motion_defines.cpp
  src/polynomial.cpp
  src/segment.cpp
  src/timing.cpp
  src/trajectory.cpp
  src/trajectory_sampling.cpp
  src/vertex.cpp
  src/io.cpp
  src/rpoly/rpoly_ak1.cpp
)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  mav_msgs
)

target_link_libraries(${PROJECT_NAME}
  ${YAML_CPP_INCLUDE_DIR}
  # ${NLOPT_INCLUDE_DIRS}
  Eigen3::Eigen
)

# Add executables
add_executable(polynomial_timing_evaluation src/polynomial_timing_evaluation.cpp)
target_link_libraries(polynomial_timing_evaluation ${PROJECT_NAME})

# Add tests
# ament_cmake_gtest(test_polynomial test/test_polynomial.cpp)
# target_link_libraries(test_polynomial ${PROJECT_NAME} rclcpp)

# ament_cmake_gtest(test_polynomial_optimization test/test_polynomial_optimization.cpp)
# target_link_libraries(test_polynomial_optimization ${PROJECT_NAME} rclcpp)

# Install exported libraries
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

install(TARGETS
  polynomial_timing_evaluation
  # test_polynomial
  # test_polynomial_optimization
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()

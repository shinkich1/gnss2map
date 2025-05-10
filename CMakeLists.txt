cmake_minimum_required(VERSION 3.8)
project(gnss2map)

# Use C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

# GaussKruger node executable (headerless)
add_executable(gauss_kruger
  src/GaussKruger.cpp
)
ament_target_dependencies(gauss_kruger
  rclcpp
  sensor_msgs
  geometry_msgs
  Eigen3
)

# GnssPoser node executable (headerless)
add_executable(gnss_poser
  src/GnssPoser.cpp
)
ament_target_dependencies(gnss_poser
  rclcpp
  geometry_msgs
  tf2
  tf2_geometry_msgs
)

# Enable testing
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  add_subdirectory(test)
endif()

# Install targets
install(TARGETS
  gauss_kruger
  gnss_poser
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()





# cmake_minimum_required(VERSION 3.8)
# project(gnss2map)

# if(NOT CMAKE_CXX_STANDARD)
#   set(CMAKE_CXX_STANDARD 17)
# endif()

# if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
#   add_compile_options(-O3 -Wall -Wextra -Wpedantic)
# endif()

# # find dependencies
# find_package(ament_cmake_auto REQUIRED)
# find_package(eigen3_cmake_module)
# find_package(Eigen3)

# ament_auto_find_build_dependencies()

# ament_auto_add_executable(gauss_kruger_node
#   src/GaussKruger.cpp
# )

# ament_auto_add_executable(gnss_poser
#   src/GnssPoser.cpp
# )

# if(BUILD_TESTING)
#   find_package(ament_lint_auto REQUIRED)
#   set(ament_cmake_copyright_FOUND TRUE)
#   set(ament_cmake_cpplint_FOUND TRUE)
#   ament_lint_auto_find_test_dependencies()
# endif()

# if(BUILD_TESTING)
#   find_package(ament_lint_auto REQUIRED)
#   ament_lint_auto_find_test_dependencies()
#   add_subdirectory(test)
# endif()

# ament_auto_package(
#   INSTALL_TO_SHARE
#   config
#   launch
# )

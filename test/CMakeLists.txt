cmake_minimum_required(VERSION 3.5)
project(gnss2map_unit_tests)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_gtest REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(GTest REQUIRED)
find_package(Eigen3 REQUIRED)

# GaussKruger unit test
ament_add_gtest(test_gauss_kruger
  test_gauss_kruger.cpp
)
if(TARGET test_gauss_kruger)
  target_link_libraries(test_gauss_kruger
    GTest::gtest_main
  )
  ament_target_dependencies(test_gauss_kruger
    rclcpp
    sensor_msgs
    geometry_msgs
    Eigen3
  )
endif()

# GnssPoser unit test
ament_add_gtest(test_gnss_poser
  test_gnss_poser.cpp
)
if(TARGET test_gnss_poser)
  target_link_libraries(test_gnss_poser
    GTest::gtest_main
  )
  ament_target_dependencies(test_gnss_poser
    rclcpp
    geometry_msgs
    tf2
    tf2_geometry_msgs
  )
endif()

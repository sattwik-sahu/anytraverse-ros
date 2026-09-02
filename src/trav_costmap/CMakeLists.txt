cmake_minimum_required(VERSION 3.20)
project(trav_costmap)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wno-unused-parameter)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(pluginlib REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(message_filters REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgproc)
find_package(Eigen3 REQUIRED)

add_library(trav_costmap SHARED src/traversability_layer.cpp)
target_include_directories(trav_costmap PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
set_target_properties(trav_costmap PROPERTIES
  BUILD_EXPORT_NAME trav_costmap
  CXX_STANDARD 17
  CXX_STANDARD_REQUIRED ON)

target_link_libraries(trav_costmap PUBLIC
  rclcpp::rclcpp
  rclcpp_lifecycle::rclcpp_lifecycle
  nav2_costmap_2d::nav2_costmap_2d_core
  pluginlib::pluginlib
  sensor_msgs::sensor_msgs_library
  ${geometry_msgs_TARGETS}
  cv_bridge::cv_bridge
  tf2::tf2
  tf2_ros::tf2_ros
  tf2_geometry_msgs::tf2_geometry_msgs
  tf2_eigen::tf2_eigen
  message_filters::message_filters
  ${OpenCV_LIBRARIES}
  Eigen3::Eigen)

pluginlib_export_plugin_description_file(nav2_costmap_2d plugins.xml)

install(TARGETS trav_costmap
  EXPORT trav_costmap
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include)
install(FILES plugins.xml DESTINATION share/${PROJECT_NAME})
install(DIRECTORY config/ DESTINATION share/${PROJECT_NAME}/config)

ament_export_targets(trav_costmap HAS_LIBRARY_TARGET)
ament_export_dependencies(
  rclcpp rclcpp_lifecycle nav2_costmap_2d pluginlib sensor_msgs geometry_msgs
  cv_bridge tf2 tf2_ros tf2_geometry_msgs tf2_eigen message_filters)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

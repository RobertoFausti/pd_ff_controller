cmake_minimum_required(VERSION 3.16)
project(pd_ff_controller)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_CXX_STANDARD 20)

find_package(ament_cmake REQUIRED)
find_package(controller_interface REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(generate_parameter_library REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(realtime_tools REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

generate_parameter_library(pd_ff_controller_parameters
  src/pd_ff_controller.yaml
)

add_library(pd_ff_controller SHARED
  src/pd_ff_controller.cpp
  src/pd_ff.cpp
)
target_include_directories(pd_ff_controller PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/pd_ff_controller>
)
target_link_libraries(pd_ff_controller PUBLIC
  pd_ff_controller_parameters
  controller_interface::controller_interface
  hardware_interface::hardware_interface
  pluginlib::pluginlib
  rclcpp::rclcpp
  rclcpp_lifecycle::rclcpp_lifecycle
  realtime_tools::realtime_tools
  ${sensor_msgs_TARGETS}
)

pluginlib_export_plugin_description_file(controller_interface pd_ff_controller.xml)

install(DIRECTORY include/ DESTINATION include/pd_ff_controller)
install(TARGETS pd_ff_controller pd_ff_controller_parameters
  EXPORT export_pd_ff_controller
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
ament_export_targets(export_pd_ff_controller HAS_LIBRARY_TARGET)
ament_export_dependencies(
  controller_interface hardware_interface generate_parameter_library
  pluginlib rclcpp rclcpp_lifecycle realtime_tools std_msgs sensor_msgs
)

if(BUILD_TESTING)
  find_package(ament_cmake_ros   REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_lint_auto   REQUIRED)
  find_package(lifecycle_msgs    REQUIRED)

  ament_lint_auto_find_test_dependencies()

  ament_add_ros_isolated_gtest(test_pd_ff_controller
    test/test_pd_ff_controller.cpp
  )
  target_link_libraries(test_pd_ff_controller
    pd_ff_controller
    pd_ff_controller_parameters
    controller_interface::controller_interface
    hardware_interface::hardware_interface
    rclcpp::rclcpp
    rclcpp_lifecycle::rclcpp_lifecycle
    ${sensor_msgs_TARGETS}
    ${std_msgs_TARGETS}
  )
  target_include_directories(test_pd_ff_controller PRIVATE
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/include>
  )
  ament_target_dependencies(test_pd_ff_controller lifecycle_msgs)
endif()

ament_package()

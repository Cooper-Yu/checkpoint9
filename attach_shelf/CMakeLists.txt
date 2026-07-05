cmake_minimum_required(VERSION 3.8)
project(attach_shelf)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(rosidl_default_generators REQUIRED)

option(BUILD_EXTRA_ATTACH_SHELF_NODES "Build experimental and legacy attach_shelf nodes" OFF)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/GoToLoading.srv"
)

add_executable(pre_approach src/pre_approach.cpp)
ament_target_dependencies(pre_approach
  rclcpp
  geometry_msgs
  sensor_msgs
  tf2
  tf2_geometry_msgs
  tf2_ros
)

add_executable(approach_service_server_simple src/approach_service_server_simple.cpp)
ament_target_dependencies(approach_service_server_simple
  rclcpp
  geometry_msgs
  sensor_msgs
  std_msgs
  tf2_ros
)

rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME}
  "rosidl_typesupport_cpp"
)
target_link_libraries(approach_service_server_simple "${cpp_typesupport_target}")

install(TARGETS
  pre_approach
  approach_service_server_simple
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_EXTRA_ATTACH_SHELF_NODES)
  add_executable(pre_approach_simple src/pre_approach_simple.cpp)
  ament_target_dependencies(pre_approach_simple
    rclcpp
    geometry_msgs
    sensor_msgs
  )

  add_executable(pre_approach_v2 src/pre_approach_v2.cpp)
  ament_target_dependencies(pre_approach_v2
    rclcpp
    geometry_msgs
    sensor_msgs
  )

  add_executable(approach_service_server src/approach_service_server.cpp)
  ament_target_dependencies(approach_service_server
    rclcpp
    geometry_msgs
    sensor_msgs
    std_msgs
    tf2_ros
  )

  target_link_libraries(pre_approach_v2 "${cpp_typesupport_target}")
  target_link_libraries(approach_service_server "${cpp_typesupport_target}")

  install(TARGETS
    pre_approach_simple
    pre_approach_v2
    approach_service_server
    DESTINATION lib/${PROJECT_NAME}
  )
endif()

install(DIRECTORY
  launch
  rviz
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

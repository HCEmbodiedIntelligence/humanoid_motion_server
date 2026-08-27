cmake_minimum_required(VERSION 3.8)
project(humanoid_motion_server)

include(GNUInstallDirs)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(humanoid_motion_interfaces REQUIRED)
find_package(kdl_parser REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(yaml_cpp_vendor REQUIRED)
find_package(yaml-cpp REQUIRED)

include(cmake/ConfigureRoboManipSdk.cmake)
configure_file(
  cmake/humanoid_motion_server-sdk-extras.cmake.in
  "${CMAKE_CURRENT_BINARY_DIR}/humanoid_motion_server-sdk-extras.cmake"
  @ONLY
)

add_library(humanoid_motion_runtime SHARED
  src/arbitration/control_arbiter.cpp
  src/channel_config.cpp
  src/kinematics/robo_manip_sdk_api.cpp
  src/kinematics/sdk_kinematics.cpp
  src/motion/command_pipeline.cpp
  src/motion/goal_monitor.cpp
  src/motion/sdk_motion_backend.cpp
  src/tf/tf_runtime.cpp
)
target_include_directories(humanoid_motion_runtime PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_options(humanoid_motion_runtime PRIVATE -Wall -Wextra -Wpedantic)
target_link_libraries(humanoid_motion_runtime
  yaml-cpp
  Eigen3::Eigen
  humanoid_motion_server_sdk::robo_manip
  humanoid_motion_server_sdk::motion_control
  humanoid_motion_server_sdk::placo
  kdl_parser::kdl_parser
)
ament_target_dependencies(humanoid_motion_runtime
  geometry_msgs
  kdl_parser
  rclcpp
  sensor_msgs
  tf2_ros
)
target_link_options(humanoid_motion_runtime PRIVATE "LINKER:--no-as-needed")
set_target_properties(humanoid_motion_runtime PROPERTIES
  BUILD_RPATH "${HUMANOID_MOTION_SDK_BUILD_RPATH}"
  INSTALL_RPATH "${HUMANOID_MOTION_SDK_INSTALL_RPATH}"
  INSTALL_RPATH_USE_LINK_PATH FALSE
)

add_executable(humanoid_motion_control_node src/humanoid_motion_control_node.cpp)
target_compile_options(humanoid_motion_control_node PRIVATE -Wall -Wextra -Wpedantic)
target_link_libraries(humanoid_motion_control_node humanoid_motion_runtime yaml-cpp)
ament_target_dependencies(humanoid_motion_control_node
  geometry_msgs
  humanoid_motion_interfaces
  rclcpp
  rclcpp_action
  sensor_msgs
  tf2
)
target_link_options(humanoid_motion_control_node PRIVATE "LINKER:--allow-shlib-undefined")
set_target_properties(humanoid_motion_control_node PROPERTIES
  BUILD_RPATH
    "${CMAKE_CURRENT_BINARY_DIR};${HUMANOID_MOTION_SDK_BUILD_RPATH}"
  INSTALL_RPATH
    "\$ORIGIN/..;${HUMANOID_MOTION_SDK_DEPS_PREFIX}/${CMAKE_INSTALL_LIBDIR};/opt/ros/humble/lib;/opt/ros/humble/lib/x86_64-linux-gnu"
  INSTALL_RPATH_USE_LINK_PATH FALSE
)

install(TARGETS
  humanoid_motion_runtime
  humanoid_motion_control_node
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY vendor/robo_manip/include/ DESTINATION include)
install(FILES
  vendor/robo_manip/lib/librobo_manip.so
  vendor/robo_manip/lib/libmotion_control.so
  vendor/robo_manip/lib/liblibplaco.so
  DESTINATION lib)
install(FILES
  vendor/robo_manip/NOTICE.md
  vendor/robo_manip/MANIFEST.yaml
  vendor/robo_manip/SHA256SUMS
  DESTINATION share/${PROJECT_NAME}/vendor/robo_manip)
install(DIRECTORY config docs launch urdf DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE
  PATTERN "*.pyc" EXCLUDE)
install(PROGRAMS
  scripts/check_sdk_runtime
  scripts/verify_mock_stack.py
  DESTINATION lib/${PROJECT_NAME})
install(FILES README.md DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  function(add_runtime_gtest target source)
    ament_add_gtest(${target} ${source})
    target_link_libraries(${target} humanoid_motion_runtime)
  endfunction()

  add_runtime_gtest(test_channel_config test/test_channel_config.cpp)
  add_runtime_gtest(test_command_pipeline test/test_command_pipeline.cpp)
  add_runtime_gtest(test_control_arbiter test/test_control_arbiter.cpp)
  add_runtime_gtest(test_goal_monitor test/test_goal_monitor.cpp)
  add_runtime_gtest(test_move_p_waypoint_resolver test/test_move_p_waypoint_resolver.cpp)
  target_include_directories(test_move_p_waypoint_resolver PRIVATE src/motion)
  add_runtime_gtest(test_sdk_kinematics test/test_sdk_kinematics.cpp)
  add_runtime_gtest(test_sdk_units test/test_sdk_units.cpp)
  target_include_directories(test_sdk_units PRIVATE
    src/motion
    $<TARGET_PROPERTY:humanoid_motion_server_sdk::motion_control,INTERFACE_INCLUDE_DIRECTORIES>)

  ament_add_gtest(test_sdk_so_integration test/test_sdk_so_integration.cpp)
  target_include_directories(test_sdk_so_integration PRIVATE
    $<TARGET_PROPERTY:humanoid_motion_server_sdk::motion_control,INTERFACE_INCLUDE_DIRECTORIES>)
  target_compile_definitions(test_sdk_so_integration PRIVATE
    HUMANOID_MOTION_CONTROL_SDK_LIBRARY="$<TARGET_FILE:humanoid_motion_server_sdk::motion_control>")
  target_link_libraries(test_sdk_so_integration
    ${CMAKE_DL_LIBS} humanoid_motion_server_sdk::motion_control)

  add_executable(test_sdk_targets test/test_sdk_targets.cpp)
  target_compile_features(test_sdk_targets PRIVATE cxx_std_17)
  target_link_libraries(test_sdk_targets PRIVATE
    humanoid_motion_server_sdk::robo_manip
    humanoid_motion_server_sdk::motion_control
    humanoid_motion_server_sdk::placo)
  add_test(NAME sdk_targets COMMAND test_sdk_targets)

  add_test(
    NAME sdk_sha256
    COMMAND "${CMAKE_CURRENT_SOURCE_DIR}/scripts/check_sdk_runtime"
      --sdk-root "${HUMANOID_MOTION_SDK_ROOT}"
      --sha-only)
  add_test(
    NAME sdk_ldd_r
    COMMAND "${CMAKE_CURRENT_SOURCE_DIR}/scripts/check_sdk_runtime"
      --sdk-root "${HUMANOID_MOTION_SDK_ROOT}"
      --deps-prefix "${HUMANOID_MOTION_SDK_DEPS_PREFIX}"
      --ldd-only)

endif()

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
  Eigen3
  geometry_msgs
  humanoid_motion_interfaces
  kdl_parser
  rclcpp
  sensor_msgs
  tf2_ros
  yaml_cpp_vendor)

ament_package(
  CONFIG_EXTRAS
    "${CMAKE_CURRENT_BINARY_DIR}/humanoid_motion_server-sdk-extras.cmake"
)

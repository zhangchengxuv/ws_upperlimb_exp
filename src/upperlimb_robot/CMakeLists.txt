cmake_minimum_required(VERSION 3.8)
project(upperlimb_robot)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ========================================
# Dependencies
# ========================================
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)

# ========================================
# ROS2 Interfaces
# ========================================
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ControlCommand.msg"
  "msg/ExperimentConfig.msg"
  "msg/LoggerCommand.msg"
  "msg/RobotState.msg"
  "msg/SystemState.msg"
)

ament_export_dependencies(rosidl_default_runtime)

# 必须放在 rosidl_generate_interfaces 之后，避免 eigenpy 干扰 rosidl 的 FindPythonInterp
find_package(pinocchio REQUIRED)
find_package(Eigen3 REQUIRED)

# ========================================
# Googoltech EtherCAT Libraries (完整链接，不删减)
# ========================================
set(GOOGOLTECH_LIB_DIR ${CMAKE_CURRENT_SOURCE_DIR}/lib)

set(GOOGOLTECH_LIBS
  ${GOOGOLTECH_LIB_DIR}/libgxn_x64.so
  ${GOOGOLTECH_LIB_DIR}/libgt_rn_x64.so
  ${GOOGOLTECH_LIB_DIR}/libGLink_x64.so
  ${GOOGOLTECH_LIB_DIR}/libMachineBuilding_x64.so
  ${GOOGOLTECH_LIB_DIR}/libLAFunc_x64.so
  ${GOOGOLTECH_LIB_DIR}/libecat_master.so
  ${GOOGOLTECH_LIB_DIR}/libecat_config.so
  ${GOOGOLTECH_LIB_DIR}/libecfg_parser.so
)

# ========================================
# Control Algorithms Library
# ========================================
add_library(upperlimb_control_algorithms SHARED
  src/control_algorithms.cpp
)

target_include_directories(upperlimb_control_algorithms PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIRS}
)

target_compile_features(upperlimb_control_algorithms PUBLIC cxx_std_17)

ament_target_dependencies(upperlimb_control_algorithms
  ament_index_cpp
)

target_link_libraries(upperlimb_control_algorithms
  pinocchio::pinocchio
  Eigen3::Eigen
)

# ========================================
# Async CSV Logger Library (不依赖 EtherCAT)
# ========================================
add_library(upperlimb_async_logger SHARED
  src/async_csv_logger.cpp
)

target_include_directories(upperlimb_async_logger PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_compile_features(upperlimb_async_logger PUBLIC cxx_std_17)

# ========================================
# Real-Hardware Control Node
# ========================================
add_executable(robot_control_node
  src/robot_control_node.cpp
)

target_include_directories(robot_control_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIRS}
)

target_compile_features(robot_control_node PUBLIC c_std_99 cxx_std_17)

ament_target_dependencies(robot_control_node
  rclcpp
  sensor_msgs
  std_msgs
  ament_index_cpp
)

rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} "rosidl_typesupport_cpp"
)

target_link_libraries(robot_control_node
  "${cpp_typesupport_target}"
  upperlimb_control_algorithms
  upperlimb_async_logger
  ${GOOGOLTECH_LIBS}
)

# ========================================
# Offline Unit Test Executable (纯软件，不依赖 EtherCAT)
# ========================================
add_executable(test_async_csv_logger
  src/test_async_csv_logger.cpp
)

target_include_directories(test_async_csv_logger PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_compile_features(test_async_csv_logger PUBLIC cxx_std_17)

target_link_libraries(test_async_csv_logger
  upperlimb_async_logger
  upperlimb_control_algorithms
)

# ========================================
# Install Rules
# ========================================
install(TARGETS
  upperlimb_control_algorithms
  upperlimb_async_logger
  EXPORT upperlimb_robot_targets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

install(TARGETS
  robot_control_node
  test_async_csv_logger
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include/
)

ament_export_targets(upperlimb_robot_targets HAS_LIBRARY_TARGET)
ament_export_dependencies(ament_index_cpp Eigen3 pinocchio)

install(
  EXPORT upperlimb_robot_targets
  DESTINATION share/${PROJECT_NAME}/cmake
)

install(
  DIRECTORY msg/
  DESTINATION share/${PROJECT_NAME}/msg
)

# 安装 Googoltech 动态库到 lib/ 与 share/，确保跨机 source install/setup.bash 后可直接运行
install(
  DIRECTORY lib/
  DESTINATION lib
)

install(
  DIRECTORY lib/
  DESTINATION share/${PROJECT_NAME}/lib
)

install(
  DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

install(
  FILES config/Gecat.eni
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY urdf/
  DESTINATION share/${PROJECT_NAME}/urdf
)

install(
  DIRECTORY launch rviz
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS scripts/mock_robot_node.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

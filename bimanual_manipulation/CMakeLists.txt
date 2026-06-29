cmake_minimum_required(VERSION 3.8)
project(bimanual_manipulation)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(shape_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(control_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(urdf REQUIRED)
find_package(srdfdom REQUIRED)
find_package(kdl_parser REQUIRED)
find_package(orocos_kdl REQUIRED)
find_package(resource_retriever REQUIRED)
find_package(bimanual_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(fcl REQUIRED)
find_package(assimp REQUIRED)
find_package(yaml-cpp REQUIRED)

set(ament_deps
  rclcpp
  rclcpp_action
  sensor_msgs
  std_msgs
  geometry_msgs
  shape_msgs
  trajectory_msgs
  control_msgs
  visualization_msgs
  urdf
  srdfdom
  kdl_parser
  resource_retriever
  bimanual_msgs
)

# --- core library ----------------------------------------------------------
add_library(${PROJECT_NAME}_core SHARED
  src/kinematics.cpp
  src/collision_model.cpp
  src/trajectory_generator.cpp
  src/move_group.cpp
  src/config_loader.cpp
  src/manipulation_server.cpp
)
target_include_directories(${PROJECT_NAME}_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${orocos_kdl_INCLUDE_DIRS}
  ${ASSIMP_INCLUDE_DIRS}
)
ament_target_dependencies(${PROJECT_NAME}_core ${ament_deps})
target_link_libraries(${PROJECT_NAME}_core
  Eigen3::Eigen
  fcl
  yaml-cpp
  ${orocos_kdl_LIBRARIES}
  ${ASSIMP_LIBRARIES}
)

# --- node ------------------------------------------------------------------
add_executable(manipulation_server_node src/manipulation_server_node.cpp)
target_link_libraries(manipulation_server_node ${PROJECT_NAME}_core)
ament_target_dependencies(manipulation_server_node ${ament_deps})

# --- install ---------------------------------------------------------------
install(TARGETS ${PROJECT_NAME}_core
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(TARGETS manipulation_server_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME}_core)
ament_export_dependencies(${ament_deps} fcl Eigen3 orocos_kdl assimp yaml-cpp)
ament_package()

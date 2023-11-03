cmake_minimum_required(VERSION 2.8...3.13)
project(youbot_dynamics)

set(CMAKE_CXX_STANDARD 17)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(orocos_kdl)
find_package(kdl_parser)
find_package(youbot_driver REQUIRED)
find_package(Boost REQUIRED COMPONENTS thread date_time filesystem system regex)

include_directories(
  include ${Boost_INCLUDE_DIR}
)

add_executable(youbot_solver_test src/youbot_solver_test.cpp)

ament_target_dependencies(youbot_solver_test
  orocos_kdl
  kdl_parser
  youbot_driver
)

target_link_libraries(youbot_solver_test
  ${Boost_LIBRARIES}
  ${youbot_driver_LIBRARIES}
  yaml-cpp
)

add_executable(youbot_rne_solver_test src/youbot_rne_solver_test.cpp)

ament_target_dependencies(youbot_rne_solver_test
  orocos_kdl
  kdl_parser
  youbot_driver
)

target_link_libraries(youbot_rne_solver_test
  ${Boost_LIBRARIES}
  ${youbot_driver_LIBRARIES}
  yaml-cpp
)

install(TARGETS youbot_rne_solver_test youbot_solver_test
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  urdf/
  DESTINATION share/${PROJECT_NAME}/urdf
)

install(DIRECTORY
  config/
  DESTINATION share/${PROJECT_NAME}/config
)

ament_export_dependencies(
  orocos_kdl
  kdl_parser
  youbot_driver
)

ament_package()
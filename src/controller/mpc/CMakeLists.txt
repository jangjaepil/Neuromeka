cmake_minimum_required(VERSION 3.8)
project(mpc)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(OsqpEigen REQUIRED)
find_package(Eigen3)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

include_directories(SYSTEM ${EIGEN3_INCLUDE_DIR} include/mpc)

add_executable(MPCExample src/demo.cpp src/Main.cpp src/MPC.cpp)
target_link_libraries(MPCExample OsqpEigen::OsqpEigen)
ament_target_dependencies(MPCExample rclcpp std_msgs geometry_msgs) 

install(
  TARGETS MPCExample
  DESTINATION lib/${PROJECT_NAME})

ament_package()

cmake_minimum_required(VERSION 3.8)
project(mpc)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(OsqpEigen REQUIRED)
find_package(Eigen3)

include_directories(SYSTEM ${EIGEN3_INCLUDE_DIR})

add_executable(MPCExample src/MPCExample.cpp)
target_link_libraries(MPCExample OsqpEigen::OsqpEigen)


install(
  TARGETS MPCExample
  DESTINATION lib/${PROJECT_NAME})

ament_package()

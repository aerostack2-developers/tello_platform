cmake_minimum_required(VERSION 3.5)
set(PROJECT_NAME  tello_platform)
project(${PROJECT_NAME})

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(as2_core REQUIRED)
find_package(as2_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

find_package(OpenCV REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

set(HEADER_HPP_FILES
  tello_connection/include/socketudp.hpp
  tello_connection/include/tello.hpp
)
set(SOURCE_CPP_FILES
  tello_connection/src/socketUdp.cpp
  tello_connection/src/tello.cpp
  src/${PROJECT_NAME}.cpp
  src/${PROJECT_NAME}_node.cpp
)
include_directories(
  include
  tello_connection/include
  ${EIGEN3_INCLUDE_DIRS}
)

# Add the exacutables
add_executable(${PROJECT_NAME}_node ${SOURCE_CPP_FILES})

ament_target_dependencies(${PROJECT_NAME}_node 
  rclcpp 
  sensor_msgs 
  std_msgs
  std_srvs
  nav_msgs
  as2_core
  as2_msgs
  geometry_msgs
  tf2
  tf2_ros
  Eigen3
)
target_link_libraries(${PROJECT_NAME}_node ${OpenCV_LIBS})


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

install(TARGETS
  ${PROJECT_NAME}_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()

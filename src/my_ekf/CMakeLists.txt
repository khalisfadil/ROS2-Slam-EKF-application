cmake_minimum_required(VERSION 3.5)
project(my_ekf)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

#SET(CMAKE_BUILD_TYPE "RELEASE")
SET(CMAKE_CXX_FLAGS "-O2 -g ${CMAKE_CXX_FLAGS}")

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(message_filters REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_ros REQUIRED)

find_package(chcv_msgs REQUIRED)
find_package(nmea_msgs REQUIRED)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

add_library(ekf_component SHARED
src/ekf_component.cpp
)

target_compile_definitions(ekf_component PRIVATE "A_EKFC_BUILDING_DLL")

ament_target_dependencies(ekf_component
  rclcpp 
  rclcpp_components 
  tf2 
  tf2_eigen 
  std_msgs
  chcv_msgs
  nmea_msgs
)

add_executable(ekf_node
src/ekf_node.cpp
)

target_link_libraries(ekf_node
ekf_component
${EIGEN_LIBRARIES})

include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
)

ament_target_dependencies(ekf_node
  rclcpp)

rclcpp_components_register_nodes(ekf_component
  "autobin::EKFComponent")

# install executables/libs
install(
  DIRECTORY "include/"
  DESTINATION include
)

install(
  DIRECTORY launch param
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS
  ekf_node
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
  ekf_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

# export
ament_export_include_directories(include)

ament_package()

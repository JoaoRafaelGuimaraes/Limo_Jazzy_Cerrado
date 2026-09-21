cmake_minimum_required(VERSION 3.8)
project(limo_cerrado_sim)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
# opcional: so e preciso para publicar livox_ros_driver2/CustomMsg (livox_format:=custom)
find_package(livox_ros_driver2 QUIET)

add_executable(livox_mid360_emulator src/livox_mid360_emulator.cpp)
target_include_directories(livox_mid360_emulator PRIVATE ${EIGEN3_INCLUDE_DIR})
set(EMULATOR_DEPS rclcpp sensor_msgs nav_msgs)
if(livox_ros_driver2_FOUND)
  message(STATUS "livox_ros_driver2 encontrado: emulador do MID-360 com suporte a CustomMsg")
  target_compile_definitions(livox_mid360_emulator PRIVATE HAVE_LIVOX_CUSTOM_MSG)
  list(APPEND EMULATOR_DEPS livox_ros_driver2)
else()
  message(STATUS "livox_ros_driver2 NAO encontrado: emulador do MID-360 so com PointCloud2")
endif()
ament_target_dependencies(livox_mid360_emulator ${EMULATOR_DEPS})
install(TARGETS livox_mid360_emulator DESTINATION lib/${PROJECT_NAME})

install(
  DIRECTORY urdf worlds models config launch rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

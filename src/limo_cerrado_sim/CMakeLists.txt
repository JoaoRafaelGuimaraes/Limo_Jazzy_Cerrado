cmake_minimum_required(VERSION 3.8)
project(limo_cerrado_sim)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY urdf worlds models config launch rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

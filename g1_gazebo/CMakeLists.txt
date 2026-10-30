cmake_minimum_required(VERSION 3.8)
project(g1_gazebo)

find_package(ament_cmake REQUIRED)

# make_urdf.py で urdf/ と meshes/ を作ってからビルドする
install(DIRECTORY config gui launch meshes urdf worlds
  DESTINATION share/${PROJECT_NAME})

ament_package()

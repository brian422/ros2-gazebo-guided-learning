cmake_minimum_required(VERSION 3.8)
project(hazard_inspection_description)

find_package(ament_cmake REQUIRED)

# Install authored robot-description resources into the package share directory.
# ROS 2 launch tools locate these files through the ament resource index.
install(
  DIRECTORY launch rviz urdf
  DESTINATION share/${PROJECT_NAME}
)

ament_package()


cmake_minimum_required(VERSION 3.8)
project(parking_lot_navigation)

find_package(ament_cmake REQUIRED)

# Install only the static Nav2 parameters, behavior tree and manual launch file.
install(DIRECTORY config launch behavior_trees DESTINATION share/${PROJECT_NAME})

ament_package()

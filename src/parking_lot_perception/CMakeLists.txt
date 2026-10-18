cmake_minimum_required(VERSION 3.8)
project(parking_lot_perception)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgproc)
find_package(geometry_msgs REQUIRED)
find_package(parking_lot_core REQUIRED)
find_package(RapidJSON REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(visualization_msgs REQUIRED)

add_library(traditional_parking_slot_detector SHARED
  src/traditional_parking_slot_detector.cpp
)
target_include_directories(traditional_parking_slot_detector PUBLIC include ${OpenCV_INCLUDE_DIRS})
target_link_libraries(traditional_parking_slot_detector ${OpenCV_LIBRARIES})
target_compile_options(traditional_parking_slot_detector PRIVATE -Wall -Wextra -Wpedantic)
ament_target_dependencies(traditional_parking_slot_detector rclcpp sensor_msgs std_msgs)

add_executable(parking_slot_bev_node src/parking_slot_bev_node.cpp)
target_link_libraries(parking_slot_bev_node traditional_parking_slot_detector)
target_compile_options(parking_slot_bev_node PRIVATE -Wall -Wextra -Wpedantic)
ament_target_dependencies(parking_slot_bev_node rclcpp sensor_msgs std_msgs)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  target_compile_options(traditional_parking_slot_detector PRIVATE -O2)
  target_compile_options(parking_slot_bev_node PRIVATE -O2)
endif()

add_library(parking_slot_map_store SHARED src/parking_slot_map_store.cpp)
target_include_directories(parking_slot_map_store PUBLIC include)
target_compile_options(parking_slot_map_store PRIVATE -Wall -Wextra -Wpedantic)
ament_target_dependencies(parking_slot_map_store parking_lot_core)

add_library(pillar_landmark_store SHARED src/pillar_landmark_store.cpp)
target_include_directories(pillar_landmark_store PUBLIC include)
target_compile_options(pillar_landmark_store PRIVATE -Wall -Wextra -Wpedantic)
ament_target_dependencies(pillar_landmark_store parking_lot_core)

add_executable(parking_slot_map_fusion src/parking_slot_map_fusion.cpp)
target_include_directories(parking_slot_map_fusion PRIVATE ${RapidJSON_INCLUDE_DIRS})
target_link_libraries(parking_slot_map_fusion parking_slot_map_store pillar_landmark_store)
target_compile_options(parking_slot_map_fusion PRIVATE -Wall -Wextra -Wpedantic)
ament_target_dependencies(
  parking_slot_map_fusion
  geometry_msgs
  parking_lot_core
  rclcpp
  std_msgs
  std_srvs
  tf2
  tf2_geometry_msgs
  tf2_ros
  visualization_msgs
)

add_library(pillar_scan_detector SHARED src/pillar_scan_detector.cpp)
target_include_directories(pillar_scan_detector PUBLIC include)
target_compile_options(pillar_scan_detector PRIVATE -Wall -Wextra -Wpedantic)
ament_target_dependencies(pillar_scan_detector sensor_msgs)

add_executable(pillar_scan_detector_node src/pillar_scan_detector_node.cpp)
target_link_libraries(pillar_scan_detector_node pillar_scan_detector)
target_include_directories(pillar_scan_detector_node PRIVATE ${RapidJSON_INCLUDE_DIRS})
target_compile_options(pillar_scan_detector_node PRIVATE -Wall -Wextra -Wpedantic)
ament_target_dependencies(
  pillar_scan_detector_node
  geometry_msgs
  rclcpp
  sensor_msgs
  std_msgs
  tf2
  tf2_geometry_msgs
  tf2_ros
  visualization_msgs
)

install(TARGETS
  traditional_parking_slot_detector
  parking_slot_bev_node
  parking_slot_map_store
  pillar_landmark_store
  parking_slot_map_fusion
  pillar_scan_detector
  pillar_scan_detector_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib/parking_lot_perception
  RUNTIME DESTINATION lib/parking_lot_perception
)
install(DIRECTORY include/ DESTINATION include)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(
    test_traditional_parking_slot_detector
    test/test_traditional_parking_slot_detector.cpp)
  target_link_libraries(test_traditional_parking_slot_detector traditional_parking_slot_detector)
  target_include_directories(test_traditional_parking_slot_detector PRIVATE include)
  target_compile_options(test_traditional_parking_slot_detector PRIVATE -Wall -Wextra -Wpedantic)

  ament_add_gtest(test_parking_slot_map_store test/test_parking_slot_map_store.cpp)
  target_link_libraries(test_parking_slot_map_store parking_slot_map_store)
  target_include_directories(test_parking_slot_map_store PRIVATE include)
  ament_target_dependencies(test_parking_slot_map_store parking_lot_core)

  ament_add_gtest(test_pillar_landmark_store test/test_pillar_landmark_store.cpp)
  target_link_libraries(test_pillar_landmark_store pillar_landmark_store)
  target_include_directories(test_pillar_landmark_store PRIVATE include)
  ament_target_dependencies(test_pillar_landmark_store parking_lot_core)

  ament_add_gtest(test_pillar_scan_detector test/test_pillar_scan_detector.cpp)
  target_link_libraries(test_pillar_scan_detector pillar_scan_detector)
  target_include_directories(test_pillar_scan_detector PRIVATE include)
  ament_target_dependencies(test_pillar_scan_detector sensor_msgs)

  find_package(launch_testing_ament_cmake REQUIRED)
  find_package(Python3 REQUIRED COMPONENTS Interpreter)
  execute_process(
    COMMAND "${Python3_EXECUTABLE}" -c
      "import socket; s=socket.socket(socket.AF_INET, socket.SOCK_DGRAM); s.bind(('127.0.0.1', 0)); s.close()"
    RESULT_VARIABLE LOCAL_UDP_SOCKET_AVAILABLE
    OUTPUT_QUIET
    ERROR_QUIET
  )
  if(LOCAL_UDP_SOCKET_AVAILABLE EQUAL 0)
    add_launch_test(test/test_parking_slot_map_fusion_launch.py TIMEOUT 60)
  else()
    message(STATUS "Skipping ROS launch integration test: local UDP sockets are unavailable")
  endif()
endif()

ament_export_include_directories(include)
ament_export_libraries(
  traditional_parking_slot_detector
  parking_slot_map_store
  pillar_landmark_store
  pillar_scan_detector
)
ament_export_dependencies(parking_lot_core sensor_msgs)
ament_package()

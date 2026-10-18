cmake_minimum_required(VERSION 3.8)
project(parking_lot_core)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake REQUIRED)
find_package(RapidJSON REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/geometry.cpp
  src/map.cpp
  src/time_sync.cpp
  src/virtual_road_graph.cpp
  src/virtual_road_nodes.cpp
)
target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${RapidJSON_INCLUDE_DIRS}
)

target_compile_options(${PROJECT_NAME} PRIVATE -Wall -Wextra -Wpedantic)

install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(
  DIRECTORY include/
  DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_map_contract
    test/test_map_contract.cpp
  )
  target_link_libraries(test_map_contract ${PROJECT_NAME})
  target_include_directories(test_map_contract PRIVATE include ${RapidJSON_INCLUDE_DIRS})

  ament_add_gtest(test_geometry_and_time
    test/test_geometry_and_time.cpp
  )
  target_link_libraries(test_geometry_and_time ${PROJECT_NAME})
  target_include_directories(test_geometry_and_time PRIVATE include ${RapidJSON_INCLUDE_DIRS})

  ament_add_gtest(test_virtual_road_nodes
    test/test_virtual_road_nodes.cpp
  )
  target_link_libraries(test_virtual_road_nodes ${PROJECT_NAME})
  target_include_directories(test_virtual_road_nodes PRIVATE include)

  ament_add_gtest(test_virtual_road_graph
    test/test_virtual_road_graph.cpp
  )
  target_link_libraries(test_virtual_road_graph ${PROJECT_NAME})
  target_include_directories(test_virtual_road_graph PRIVATE include)
endif()

ament_export_include_directories(include)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_package()

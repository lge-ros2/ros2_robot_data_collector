cmake_minimum_required(VERSION 3.16)
project(ros2_robot_data_collector)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(HDF5 REQUIRED COMPONENTS C CXX)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(collector_node
  src/collector_node.cpp
  src/hdf5_writer.cpp
)

target_include_directories(collector_node
  PRIVATE
    include
    ${HDF5_INCLUDE_DIRS}
)

ament_target_dependencies(collector_node
  rclcpp
  sensor_msgs
  std_msgs
)

target_link_libraries(collector_node
  ${HDF5_CXX_LIBRARIES}
  ${HDF5_LIBRARIES}
)

install(TARGETS collector_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_topic_resolution
    test/test_topic_resolution.cpp
  )

  if(TARGET test_topic_resolution)
    target_include_directories(test_topic_resolution PRIVATE include)
  endif()

  ament_add_gtest(test_sync_policy
    test/test_sync_policy.cpp
  )

  if(TARGET test_sync_policy)
    target_include_directories(test_sync_policy PRIVATE include)
  endif()
endif()

ament_export_include_directories(include)
ament_package()
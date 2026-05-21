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
find_package(control_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)

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
  control_msgs
  geometry_msgs
  rclcpp
  sensor_msgs
  std_msgs
  std_srvs
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

  ament_add_gtest(test_action_sample
    test/test_action_sample.cpp
  )

  if(TARGET test_action_sample)
    target_include_directories(test_action_sample PRIVATE include)
    ament_target_dependencies(test_action_sample
      control_msgs
      geometry_msgs
      sensor_msgs
      std_msgs
    )
  endif()

  ament_add_gtest(test_hdf5_writer
    test/test_hdf5_writer.cpp
    src/hdf5_writer.cpp
  )

  if(TARGET test_hdf5_writer)
    target_include_directories(test_hdf5_writer
      PRIVATE
        include
        ${HDF5_INCLUDE_DIRS}
    )
    ament_target_dependencies(test_hdf5_writer
      control_msgs
      geometry_msgs
      rclcpp
      sensor_msgs
      std_msgs
    )
    target_link_libraries(test_hdf5_writer
      ${HDF5_CXX_LIBRARIES}
      ${HDF5_LIBRARIES}
    )
  endif()
endif()

ament_export_include_directories(include)
ament_package()
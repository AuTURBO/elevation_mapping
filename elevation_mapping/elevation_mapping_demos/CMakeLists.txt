cmake_minimum_required(VERSION 3.5.1)
project(elevation_mapping_demos)

## Find package macros and libraries
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(elevation_mapping REQUIRED)
find_package(rviz2 REQUIRED)
find_package(Qt5 COMPONENTS Widgets Core REQUIRED)
find_package(grid_map_rviz_plugin REQUIRED)


## Declare package dependencies
#ament_export_dependencies(include)
ament_export_dependencies(rclcpp elevation_mapping rviz2 grid_map_rviz_plugin)

## Install directories and scripts
install(DIRECTORY config doc launch rviz sample_data
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS scripts/tf_to_pose_publisher.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

## Tests
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_${PROJECT_NAME} test/empty_test.cpp)
  target_include_directories(test_${PROJECT_NAME} PRIVATE include)
  ament_target_dependencies(test_${PROJECT_NAME}
    rclcpp
    elevation_mapping
    rviz2
    grid_map_rviz_plugin
  )
endif()

ament_package()

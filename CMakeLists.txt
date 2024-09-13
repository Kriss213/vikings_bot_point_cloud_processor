cmake_minimum_required(VERSION 3.8)
project(vikings_bot_point_cloud_processor)

set(lib_name ${PROJECT_NAME}_core)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(image_geometry REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(pluginlib REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(PCL REQUIRED)



set(dep_pkgs
    rclcpp
    nav2_costmap_2d
    sensor_msgs
    geometry_msgs
    image_geometry
    cv_bridge
    nav_msgs
    tf2_ros
    tf2_eigen
    pluginlib
    pcl_ros
    PCL)
    

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()


# install Python executables
ament_python_install_package(${PROJECT_NAME})
ament_python_install_module(scripts/classes.py)

install(
  PROGRAMS
    scripts/camera_processor.py
    scripts/depth_processor.py
    scripts/lidar_processor.py
  DESTINATION
    lib/${PROJECT_NAME}
)
install(
  DIRECTORY
    launch
  DESTINATION
    share/${PROJECT_NAME}/
)

# Custom nav2 plugin build & install:
add_library(${lib_name} SHARED
            src/rm_safe_obstacles_layer.cpp)

include_directories(include)

install(
  TARGETS
    ${lib_name}
  DESTINATION
    lib)
      
pluginlib_export_plugin_description_file(nav2_costmap_2d rm_safe_obstacles_layer.xml)
ament_target_dependencies(${lib_name} ${dep_pkgs})

ament_package()

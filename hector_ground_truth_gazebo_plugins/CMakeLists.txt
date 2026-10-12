cmake_minimum_required(VERSION 3.8)
project(hector_ground_truth_gazebo_plugins)

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic -Wshadow)
endif ()

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(gz_common_vendor REQUIRED)
find_package(gz-common REQUIRED COMPONENTS geospatial graphics)
find_package(gz_sim_vendor REQUIRED)
find_package(gz-sim REQUIRED)
find_package(gz_plugin_vendor REQUIRED)
find_package(gz-plugin REQUIRED COMPONENTS register)
find_package(rclcpp REQUIRED)
find_package(map_bag REQUIRED)
find_package(map_bag_msgs REQUIRED)
find_package(map_bag_ros REQUIRED)
find_package(hector_ground_truth_gazebo_plugins_msgs REQUIRED)

include_directories(include)

add_library(elevation_map_ground_truth_plugin SHARED
  src/elevation_map_ground_truth_plugin.cpp
  src/world_triangles.cpp
)
ament_target_dependencies(elevation_map_ground_truth_plugin PUBLIC
  Eigen3
  rclcpp
  map_bag
  map_bag_msgs
  map_bag_ros
  hector_ground_truth_gazebo_plugins_msgs
)
target_link_libraries(elevation_map_ground_truth_plugin PUBLIC
  gz-common::gz-common
  gz-common::geospatial
  gz-common::graphics
  gz-sim::gz-sim
  gz-plugin::gz-plugin
)

add_executable(map_bag_writer src/map_bag_writer.cpp)
ament_target_dependencies(map_bag_writer PUBLIC
  rclcpp
  map_bag
  map_bag_msgs
  map_bag_ros
)

install(TARGETS elevation_map_ground_truth_plugin map_bag_writer
    DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY worlds/
    DESTINATION share/${PROJECT_NAME}/worlds
)

ament_environment_hooks(hooks/${PROJECT_NAME}.dsv.in)
ament_environment_hooks(hooks/${PROJECT_NAME}.sh.in)

if (BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    # the following line skips the linter which checks for copyrights
    # comment the line when a copyright and license is added to all source files
    set(ament_cmake_copyright_FOUND TRUE)
    # the following line skips cpplint (only works in a git repo)
    # comment the line when this package is in a git repo and when
    # a copyright and license is added to all source files
    set(ament_cmake_cpplint_FOUND TRUE)
    ament_lint_auto_find_test_dependencies()
endif ()

ament_package()

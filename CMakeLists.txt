cmake_minimum_required(VERSION 3.8)
project(mapping_2d)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# find_package(Boost REQUIRED COMPONENTS system)
find_package(nav2_common REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(nav2_util REQUIRED)
find_package(octomap_msgs REQUIRED) 
find_package(octomap REQUIRED) 


include_directories(${OCTOMAP_INCLUDE_DIRS} ${octomap_msgs_INCLUDE_DIRS})  
link_libraries(${OCTOMAP_LIBRARIES})

set(library_name mapping_2d_core)

add_library(${library_name} SHARED
  src/mapping_2d.cpp
  src/grid3d.cpp
)

target_include_directories(${library_name} PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(${library_name}
  "rclcpp"
  "rclcpp_lifecycle"
  "rclcpp_components"
  "nav2_util"
  "octomap_msgs"
  "octomap"
  "geometry_msgs"
)

rclcpp_components_register_nodes(${library_name} "mapping_2d::Mapping2D")

install( TARGETS ${library_name}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_package()

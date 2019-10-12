cmake_minimum_required(VERSION 2.8.3)
project(topological_navigation)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  nav_msgs
  geometry_msgs
  std_msgs
)

add_service_files(FILES 
  SE_points.srv
  Path.srv
)

## Generate added messages and services
generate_messages(DEPENDENCIES std_msgs geometry_msgs)

include_directories(${catkin_INCLUDE_DIRS})

catkin_package()

add_executable(topological_navigation src/topological_navigation.cpp)
target_link_libraries(topological_navigation ${catkin_LIBRARIES} yaml-cpp)

add_executable(send_data src/send_data.cpp)
target_link_libraries(send_data ${catkin_LIBRARIES})
add_dependencies(send_data ${catkin_EXPORTED_TARGETS})


if(catkin_EXPORTED_TARGETS)
  add_dependencies(topological_navigation ${catkin_EXPORTED_TARGETS})
endif()

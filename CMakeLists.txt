cmake_minimum_required(VERSION 2.8.3)
project(topological_navigation)

find_package(catkin REQUIRED COMPONENTS roscpp nav_msgs geometry_msgs)

add_message_files(FILES PointArray.msg)
add_service_files(FILES TopologicalPath.srv)

generate_messages(DEPENDENCIES geometry_msgs)

catkin_package()

include_directories(${catkin_INCLUDE_DIRS})

add_executable(topological_navigation src/topological_navigation.cpp)
target_link_libraries(topological_navigation ${catkin_LIBRARIES} yaml-cpp)

add_executable(send_data src/send_data.cpp)
target_link_libraries(send_data ${catkin_LIBRARIES})

if(catkin_EXPORTED_TARGETS)
  add_dependencies(send_data ${catkin_EXPORTED_TARGETS})
  add_dependencies(topological_navigation ${catkin_EXPORTED_TARGETS})
endif()

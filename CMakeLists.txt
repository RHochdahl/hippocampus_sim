cmake_minimum_required(VERSION 3.0.2)
project(hippocampus_sim)


find_package(catkin REQUIRED COMPONENTS
  hippocampus_msgs
  gazebo_ros
  gazebo_plugins
  roscpp
)

add_compile_options(-std=c++14)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

find_package(gazebo REQUIRED)

link_directories(${GAZEBO_LIBRARY_DIRS})


include_directories(include ${catkin_INCLUDE_DIRS}
                            ${GAZEBO_INCLUDE_DIRS}
                            ${Boost_INCLUDE_DIR})

catkin_package(
  INCLUDE_DIRS include
  DEPENDS 
      roscpp 
      gazebo_ros 
)

catkin_install_python(PROGRAMS
  nodes/rviz_visualizer_node
  nodes/tank_generator_node
  scripts/generate_tag_models
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


file(GLOB_RECURSE PLUGIN_FILES
  RELATIVE "${CMAKE_CURRENT_SOURCE_DIR}/plugins"
  "${CMAKE_CURRENT_SOURCE_DIR}/plugins/*.cpp"
)

foreach(PLUGIN_FILE ${PLUGIN_FILES})
  string(REPLACE ".cpp" "" PLUGIN_NAME ${PLUGIN_FILE})
  add_library(${PLUGIN_NAME} plugins/${PLUGIN_FILE})
  add_dependencies(${PLUGIN_NAME} ${catkin_EXPORTED_TARGETS})
  target_link_libraries(${PLUGIN_NAME} ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
endforeach(PLUGIN_FILE ${PLUGIN_FILES})



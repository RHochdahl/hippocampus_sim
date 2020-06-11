cmake_minimum_required(VERSION 2.8.3)
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
                            
                            
add_library(gazebo_barometer_water_plugin plugins/gazebo_barometer_water_plugin.cpp)
add_dependencies(gazebo_barometer_water_plugin ${catkin_EXPORTED_TARGETS})
target_link_libraries(gazebo_barometer_water_plugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})



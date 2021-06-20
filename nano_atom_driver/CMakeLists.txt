cmake_minimum_required(VERSION 3.0.2)
project(nano_atom_driver)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    std_msgs
    std_srvs
    sensor_msgs
    roboticarts_msgs
    jitbus
)


catkin_package(
    CATKIN_DEPENDS 
    roscpp
    std_msgs
    std_srvs
    sensor_msgs
    roboticarts_msgs
    jitbus
)


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)




add_library(${PROJECT_NAME} src/nano_driver.cpp)

# Add nano_driver_node.cpp
add_dependencies(nano_atom_driver ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_executable(nano_driver_node src/nano_driver_node.cpp)
add_dependencies(nano_driver_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(nano_atom_driver ${catkin_LIBRARIES})
target_link_libraries(nano_driver_node ${catkin_LIBRARIES} nano_atom_driver)




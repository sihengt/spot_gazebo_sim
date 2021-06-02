cmake_minimum_required(VERSION 2.8.3)
project(spot_gazebo_sim)

add_compile_options(-std=c++11)

# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS 
  roscpp 
  gazebo_ros 
)

# Depend on system install of Gazebo
find_package(gazebo REQUIRED)


catkin_package(
  DEPENDS 
    roscpp 
    gazebo_ros 
)


link_directories(${GAZEBO_LIBRARY_DIRS})
include_directories(${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS} include)

add_library(drone_control src/drone_control_plugin.cpp)
target_link_libraries(drone_control ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

# install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
# install(DIRECTORY urdf DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
# install(DIRECTORY meshes DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

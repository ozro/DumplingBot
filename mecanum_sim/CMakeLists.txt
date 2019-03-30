cmake_minimum_required(VERSION 2.8.3)
project(mecanum_sim)

find_package(catkin REQUIRED COMPONENTS roscpp tf)

catkin_package()

find_package(gazebo REQUIRED)

include_directories(${GAZEBO_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})
list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")
add_library(mecanum SHARED src/mecanum_controller.cc)
target_link_libraries(mecanum ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES})
add_executable(mecanum_control src/mecanum_control.cpp)
target_link_libraries(mecanum_control ${catkin_LIBRARIES})
add_executable(tf_broadcaster_node src/tf_broadcaster.cpp)
add_dependencies(tf_broadcaster_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(tf_broadcaster_node ${catkin_LIBRARIES})
install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

#install(DIRECTORY worlds
#  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(PROGRAMS
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

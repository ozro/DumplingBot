cmake_minimum_required(VERSION 2.8.3)

project(smach_ros)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
)
add_message_files(
   DIRECTORY
   msg
   FILES
   robot_state.msg
#   Message2.msg
)
#catkin_python_setup()
add_service_files(
   DIRECTORY
   srv
   FILES
   full_path.srv
   reach_table.srv
#   Service2.srv
 )

generate_messages(
   DEPENDENCIES
   std_msgs
)
catkin_package(
  CATKIN_DEPENDS message_runtime rospy rostopic std_msgs std_srvs actionlib actionlib_msgs smach smach_msgs 
)


if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  add_rostest(test/state_machine.test)
  add_rostest(test/sequence.test)
  add_rostest(test/concurrence.test)
  add_rostest(test/introspection.test)
  add_rostest(test/smach_actionlib.test)
  add_rostest(test/monitor.test)
endif()

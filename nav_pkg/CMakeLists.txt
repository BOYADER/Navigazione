cmake_minimum_required(VERSION 3.0.2)
project(nav_pkg)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  message_generation
)


# Generate messages in the 'msg' folder
 add_message_files(
  FILES
   imu.msg
   dvl.msg
   usbl.msg
   ahrs.msg
   depth.msg
   Odom.msg
 )

# Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
   std_msgs
 )


catkin_package(
  CATKIN_DEPENDS roscpp std_msgs
)


include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(test_msg src/test_msg.cpp)
add_dependencies(test_msg nav_pkg_generate_messages_cpp)
target_link_libraries(test_msg ${catkin_LIBRARIES})

add_executable(fake_modellazione src/fake_modellazione.cpp)
add_dependencies(fake_modellazione nav_pkg_generate_messages_cpp)
target_link_libraries(fake_modellazione ${catkin_LIBRARIES})

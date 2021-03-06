cmake_minimum_required(VERSION 3.0.2)
project(nav_pkg)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  message_generation
)


# Generate messages in the 'msg' folder
 add_message_files(
  FILES
   dvl.msg
   usbl.msg
   ahrs.msg
   depth.msg
   Odom.msg
   gps.msg
   nav_error.msg
 )

# Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
   std_msgs
   geometry_msgs
 )


catkin_package(
  CATKIN_DEPENDS roscpp std_msgs geometry_msgs
)


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(EKF src/EKF.cpp)
add_dependencies(EKF nav_pkg_generate_messages_cpp)
target_link_libraries(EKF ${catkin_LIBRARIES})

add_executable(PF src/PF.cpp)
add_dependencies(PF nav_pkg_generate_messages_cpp)
target_link_libraries(PF ${catkin_LIBRARIES})

add_executable(fake_modellazione src/fake_modellazione.cpp)
add_dependencies(fake_modellazione nav_pkg_generate_messages_cpp)
target_link_libraries(fake_modellazione ${catkin_LIBRARIES})

add_executable(performance_analysis src/performance_analysis.cpp)
add_dependencies(performance_analysis nav_pkg_generate_messages_cpp)
target_link_libraries(performance_analysis ${catkin_LIBRARIES})
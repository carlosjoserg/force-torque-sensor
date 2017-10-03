cmake_minimum_required(VERSION 2.8.3)
project(force_torque_sensor)

find_package(LibXml2 REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  message_generation
  std_msgs
  geometry_msgs
)

catkin_package()


include_directories(include
  ${LIBXML2_INCLUDE_DIR}
  ${catkin_INCLUDE_DIRS}
  /home/carlos/Golem/packages/Tools/SM/include
  /home/carlos/Golem/packages/Core/Sys/include
  /home/carlos/Golem/packages/Core/Defs/include
  /home/carlos/Golem/packages/Core/Math/include
)

link_directories(
  /home/carlos/lib
)


add_executable(ft_sensor_hw
  src/ft_sensor_hw.cpp
)
target_link_libraries(ft_sensor_hw
  GolemSys GolemMath GolemSM
  ${catkin_LIBRARIES}
)

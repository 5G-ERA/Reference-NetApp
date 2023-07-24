cmake_minimum_required(VERSION 3.0.2)
project(era_5g_netapp_reference_client)

find_package(catkin REQUIRED COMPONENTS
  sensor_msgs
  std_msgs
)

catkin_package(

)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

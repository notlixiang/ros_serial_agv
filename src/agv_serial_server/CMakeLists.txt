cmake_minimum_required(VERSION 2.8)
add_compile_options(-std=c++11)
project(agv_serial_server)

find_package(catkin REQUIRED COMPONENTS
  	roscpp
  	std_msgs
	std_srvs
	message_generation
  	serial
    agv_srvs
  	ur_msgs
)

#add_message_files(
#    DIRECTORY
#    msg
#  FILES
#  call_grasp.msg
#grasp_status.msg
#)

# add_service_files(
#     DIRECTORY
#     srv
#   FILES
#   call_grasp.srv
# )

#generate_messages(DEPENDENCIES std_msgs std_srvs)

catkin_package(
    CATKIN_DEPENDS
    std_msgs
    message_runtime
    serial
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${PROJECT_SOURCE_DIR}/src
)

add_executable(agv_serial_server src/agv_serial_server.cpp)

target_link_libraries(agv_serial_server
  ${catkin_LIBRARIES}
)



cmake_minimum_required(VERSION 2.8)
add_compile_options(-std=c++11)
project(ur_ee_server)

find_package(catkin REQUIRED COMPONENTS
  	roscpp
  	std_msgs
	std_srvs
	message_generation
  	serial
  	jrc_srvs
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
)

add_executable(ur_ee_server_node src/ur_ee_server.cpp)
add_dependencies(ur_ee_server_node jrc_srvs_gencpp)
add_dependencies(ur_ee_server_node ur_msgs_gencpp)

target_link_libraries(ur_ee_server_node
  ${catkin_LIBRARIES}
)



cmake_minimum_required(VERSION 2.8.3)
project(manager_api)

add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
	roscpp
	std_msgs
	message_generation
        diagnostic_msgs
)

add_service_files(DIRECTORY srv
        FILES
        Manager.srv
)

generate_messages(DEPENDENCIES
	std_msgs
	diagnostic_msgs
)

catkin_package(CATKIN_DEPENDS 
        message_runtime
        INCLUDE_DIRS include
)

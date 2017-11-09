cmake_minimum_required(VERSION 2.8.3)
project(manager_api)

add_definitions(-std=c++11)

find_package(catkin REQUIRED)

catkin_package(
        INCLUDE_DIRS include
)

###########
## Build ##
###########

include_directories(
        include
)

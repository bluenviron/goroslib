cmake_minimum_required(VERSION 2.8.3)

project(shared_actions)

add_compile_options(-std=c++11 -Wall -Wextra)

find_package(
    catkin
    REQUIRED COMPONENTS
    genmsg
    actionlib_msgs
)

add_action_files(
    FILES
    DoSomething.action
)

generate_messages(
    DEPENDENCIES
    actionlib_msgs
)

catkin_package()

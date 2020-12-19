cmake_minimum_required(VERSION 2.8.3)

project(shared_services)

add_compile_options(-std=c++11 -Wall -Wextra)

find_package(
    catkin
    REQUIRED COMPONENTS
    std_msgs
    message_generation
)

add_service_files(
    FILES
    Mysrv.srv
)

generate_messages(
    DEPENDENCIES
    std_msgs
)

catkin_package()

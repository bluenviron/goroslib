cmake_minimum_required(VERSION 2.8.3)

project(nodeactionserver)

add_compile_options(-std=c++11 -Wall -Wextra)

find_package(
    catkin
    REQUIRED COMPONENTS
    roscpp
    std_msgs
    actionlib
    shared_actions
)

catkin_package()

include_directories(${catkin_INCLUDE_DIRS})

add_executable(
    ${PROJECT_NAME}
    main.cpp
)

add_dependencies(
    ${PROJECT_NAME}
    ${${PROJECT_NAME}_EXPORTED_TARGETS}
    ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

install(TARGETS ${PROJECT_NAME} RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

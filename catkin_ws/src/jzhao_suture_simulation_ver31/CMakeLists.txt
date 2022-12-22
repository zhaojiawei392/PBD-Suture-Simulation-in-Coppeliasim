cmake_minimum_required(VERSION 2.8.3)
if (POLICY CMP0048)
  cmake_policy(SET CMP0048 NEW)
endif (POLICY CMP0048)
project(jzhao_suture_simulation_ver31)

#Add custom (non compiling) targets so launch scripts and python files show up in QT Creator's project view.
file(GLOB_RECURSE EXTRA_FILES */*)
add_custom_target(${PROJECT_NAME}_OTHER_FILES ALL WORKING_DIRECTORY ${PROJECT_SOURCE_DIR} SOURCES ${EXTRA_FILES})

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++17)
add_compile_options(-Werror=return-type)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
    roscpp
    rospy
    )

find_package(Threads REQUIRED)


###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
    #INCLUDE_DIRS include
    #LIBRARIES cobotta_kinematics
    CATKIN_DEPENDS 
    #  DEPENDS system_lib
    )

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
    #include
    ${catkin_INCLUDE_DIRS}
    usr/local/include
    )

add_executable(suture_simulation_node
    src/suture_simulation_node.cpp
    src/suture_simulation.cpp
    src/my_line_segment.cpp)
## Add cmake target dependencies of the executable
## same as for the library above

add_dependencies(suture_simulation_node
    ${${PROJECT_NAME}_EXPORTED_TARGETS}
    ${catkin_EXPORTED_TARGETS})
## Specify libraries to link a library or executable target against

target_link_libraries(suture_simulation_node
    Threads::Threads
    dqrobotics
    dqrobotics-interface-vrep
    ${catkin_LIBRARIES})

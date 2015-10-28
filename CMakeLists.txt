cmake_minimum_required( VERSION 2.8.3 )
project( smmap )

set( CATKIN_PACKAGES
    arc_utilities
    geometry_msgs
    deform_simulator
    kinematics_toolbox
)

## Find catkin macros and libraries
find_package( catkin REQUIRED COMPONENTS ${CATKIN_PACKAGES})

## System dependencies are found with CMake's conventions
find_package( cmake_modules REQUIRED )
find_package( Eigen REQUIRED )

################################################
## Declare ROS messages, services and actions ##
################################################

## Generate messages in the 'msg' folder
# add_message_files(
#     FILES
#     Message1.msg
#     Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#     FILES
#     Service1.srv
#     Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#     FILES
#     Action1.action
#     Action2.action
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#     DEPENDENCIES
#     std_msgs    # Or other packages containing msgs
# )

###################################
## catkin specific configuration ##
###################################

catkin_package(
    INCLUDE_DIRS    include
    LIBRARIES       ${PROJECT_NAME}
    CATKIN_DEPENDS  ${CATKIN_PACKAGES}
    DEPENDS         Eigen
                    Boost
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories( include
    SYSTEM 
    ${Eigen_INCLUDE_DIRS}
    ${catkin_INCLUDE_DIRS}
)

set( CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS} -O3 -Wall -Wextra -Wpedantic" )

## Library file for the whole project
add_library( ${PROJECT_NAME}
    src/diminishing_rigidity_model.cpp
    src/model_set.cpp
    src/planner.cpp
)
add_dependencies( ${PROJECT_NAME}
    ${catkin_EXPORTED_TARGETS}
)

## Test files to test specific functionality
add_executable( test_node
    nodes/test_node.cpp
)
add_dependencies( test_node
    ${PROJECT_NAME}
    ${catkin_EXPORTED_TARGETS}
)
target_link_libraries( test_node
    ${PROJECT_NAME}
    ${catkin_LIBRARIES}
)

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#     scripts/my_python_script
#     DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS smmap smmap_node
#     ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#     LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#     RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#     DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#     FILES_MATCHING PATTERN "*.h"
#     PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#     # myfile1
#     # myfile2
#     DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_smmap.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#     target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)

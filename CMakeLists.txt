cmake_minimum_required( VERSION 2.8.3 )
project( smmap )

## Find catkin macros and libraries
find_package( catkin REQUIRED COMPONENTS
  geometry_msgs
  tf_conversions
)

## System dependencies are found with CMake's conventions
find_package( Eigen3 )

################################################
## Declare ROS messages, services and actions ##
################################################

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   std_msgs  # Or other packages containing msgs
# )

###################################
## catkin specific configuration ##
###################################

catkin_package(
  INCLUDE_DIRS    include
                  ${EIGEN3_INCLUDE_DIR}
#  LIBRARIES       deformable_model
  CATKIN_DEPENDS  geometery_msgs
                  tf_conversions
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories( include )
include_directories( ${EIGEN3_INCLUDE_DIR} )

## Declare a C++ library
add_library( deformable_model
  src/deformable_model.cpp
)
add_dependencies( deformable_model
  ${catkin_EXPORTED_TARGETS}
)

## Declare a C++ library
add_library(diminishing_rigidity_model
  src/diminishing_rigidity_model.cpp
)
add_dependencies(diminishing_rigidity_model 
  deformable_model
  ${catkin_EXPORTED_TARGETS}
)

## Declare a executable target
add_executable( test_node
  nodes/test_node.cpp
)
target_link_libraries( test_node
  diminishing_rigidity_model
  ${catkin_LIBRARIES}
)
add_dependencies( test_node
  diminishing_rigidity_model
  ${catkin_EXPORTED_TARGETS}
)

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS smmap smmap_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_smmap.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)

cmake_minimum_required(VERSION 2.8.3)
project(haf_grasping)



## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslib
  sensor_msgs
  std_msgs
  image_geometry
  actionlib_msgs
  geometry_msgs
  actionlib
  pcl_conversions 
  pcl_ros
  tf
  pcl_msgs
  message_generation
  visualization_msgs  
)

find_package(OpenCV REQUIRED)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependencies might have been
##     pulled in transitively but can be declared for certainty nonetheless:
##     * add a build_depend tag for "message_generation"
##     * add a run_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
add_message_files(
   FILES
   GraspInput.msg
   GraspOutput.msg
)

## Generate services in the 'srv' folder
add_service_files(
   FILES
   GraspSearchCenter.srv
   GraspSearchRectangleSize.srv
   GraspCalculationTimeMax.srv
   GraspApproachVector.srv
   ShowOnlyBestGrasp.srv
   GraspPreGripperOpeningWidth.srv
)

## Generate actions in the 'action' folder
add_action_files(
   DIRECTORY action
   FILES
   CalcGraspPointsServer.action
)

## Generate added messages and services with any dependencies listed here
generate_messages(
   DEPENDENCIES
   actionlib_msgs
   actionlib
   std_msgs
   sensor_msgs
   geometry_msgs
   visualization_msgs
 )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES haf_grasping
  CATKIN_DEPENDS 
        actionlib
	actionlib_msgs 
	std_msgs 
	geometry_msgs
        message_runtime
	pcl_conversions
	pcl_ros
        sensor_msgs
        visualization_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(include
  ${catkin_INCLUDE_DIRS}
)

find_package(PCL REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
## Declare a cpp library
# add_library(calc_grasppoints_svm
#   src/${PROJECT_NAME}/calc_grasppoints_svm.cpp
# )

## Declare a cpp executable
# add_executable(calc_grasppoints_svm_node src/calc_grasppoints_svm_node.cpp)



add_library(CHaarFeature src/CHaarFeature.cpp)
add_library(CIntImage_to_Featurevec src/CIntImage_to_Featurevec.cpp)
#add_executable(calc_grasppoints src/calc_grasppoints.cpp)
add_executable(calc_grasppoints_action_server src/calc_grasppoints_action_server.cpp)
add_executable(calc_grasppoints_action_client src/calc_grasppoints_action_client.cpp)

add_dependencies(calc_grasppoints_action_client haf_grasping_generate_messages_cpp)
add_dependencies(calc_grasppoints_action_server haf_grasping_generate_messages_cpp)
add_dependencies(calc_grasppoints_action_client ${catkin_EXPORTED_TARGETS})
add_dependencies(calc_grasppoints_action_server ${catkin_EXPORTED_TARGETS})



target_link_libraries(CIntImage_to_Featurevec CHaarFeature ${catkin_LIBRARIES} ${OpenCV_LIBS} ${PCL_LIBRARIES})
#target_link_libraries(calc_grasppoints CIntImage_to_Featurevec ${catkin_LIBRARIES} ${OpenCV_LIBS} ${PCL_LIBRARIES})
target_link_libraries(calc_grasppoints_action_server CIntImage_to_Featurevec ${catkin_LIBRARIES} ${OpenCV_LIBS} ${PCL_LIBRARIES})
target_link_libraries(calc_grasppoints_action_client ${catkin_LIBRARIES} )


## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
# add_dependencies(calc_grasppoints_svm_node calc_grasppoints_svm_generate_messages_cpp)


## Specify libraries to link a library or executable target against
# target_link_libraries(calc_grasppoints_svm_node
#   ${catkin_LIBRARIES}
# )

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
# install(TARGETS calc_grasppoints_svm calc_grasppoints_svm_node
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
# catkin_add_gtest(${PROJECT_NAME}-test test/test_calc_grasppoints_svm.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)

cmake_minimum_required(VERSION 2.8.3)
project(pm_proj1)
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  sensor_msgs
  std_msgs
  image_transport
)

FIND_PACKAGE(OpenCV REQUIRED)

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

###################################
## catkin specific configuration ##
###################################

catkin_package(
    INCLUDE_DIRS include
   # LIBRARIES msg2tf
    CATKIN_DEPENDS cv_bridge std_msgs image_transport sensor_msgs
    DEPENDS OpenCV 
)

###########
## Build ##
###########

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
)

## Declare a C++ executable
add_executable(vloader_node src/vloader.cpp)

## Rename C++ executable without prefix
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
set_target_properties(vloader_node PROPERTIES OUTPUT_NAME vloader PREFIX "")

## Add cmake target dependencies of the executable
add_dependencies(vloader_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(vloader_node
    ${catkin_LIBRARIES} 
    ${OpenCV_LIBRARIES}
)


## Declare a C++ executable
add_executable(vtracker_node src/vtracker.cpp)

## Rename C++ executable without prefix
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
set_target_properties(vtracker_node PROPERTIES OUTPUT_NAME vtracker PREFIX "")

## Add cmake target dependencies of the executable
add_dependencies(vtracker_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(vtracker_node
    ${catkin_LIBRARIES} 
    ${OpenCV_LIBRARIES}
)

#############
## Install ##
#############

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
install(TARGETS vloader_node vtracker_node
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY include/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
    FILES_MATCHING PATTERN "*.h"
    PATTERN ".svn" EXCLUDE
)

## Mark other files for installation (e.g. launch and bag files, etc.)
#install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)
#install(DIRECTORY cfg DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/cfg)

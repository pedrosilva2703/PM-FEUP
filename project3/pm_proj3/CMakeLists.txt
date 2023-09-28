cmake_minimum_required(VERSION 2.8.3)
project(pm_proj3)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  std_msgs
  pcl_ros
  pcl_conversions
  nav_msgs
  message_generation
  message_filters

)

### PCL
find_package(PCL REQUIRED)
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
)

###################################
## catkin specific configuration ##
###################################

catkin_package(
    INCLUDE_DIRS include 
    CATKIN_DEPENDS roscpp sensor_msgs std_msgs pcl_ros pcl_conversions nav_msgs message_generation message_filters
#    DEPENDS 
)

###########
## Build ##
###########

## Declare a C++ executable
add_executable(detector_node src/detector_node.cpp)

## Rename C++ executable without prefix
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
set_target_properties(detector_node PROPERTIES OUTPUT_NAME detector PREFIX "")

## Add cmake target dependencies of the executable
add_dependencies(detector_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(detector_node
    ${catkin_LIBRARIES} 
    ${OpenCV_LIBRARIES}
)

## Declare a C++ executable
add_executable(risk_assessor_node src/risk_assessor_node.cpp)

## Rename C++ executable without prefix
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
set_target_properties(risk_assessor_node PROPERTIES OUTPUT_NAME risk_assessor PREFIX "")

## Add cmake target dependencies of the executable
add_dependencies(risk_assessor_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(risk_assessor_node
    ${catkin_LIBRARIES} 
    ${OpenCV_LIBRARIES}
)


#############
## Install ##
#############

install(TARGETS detector_node risk_assessor_node
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


## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
#install(TARGETS tp1_tutorial_node
#    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#)

## Mark cpp header files for installation
#install(DIRECTORY include/
#    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#    FILES_MATCHING PATTERN "*.h"
#    PATTERN ".svn" EXCLUDE
#)

## Mark other files for installation (e.g. launch and bag files, etc.)
#install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)
#install(DIRECTORY cfg DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/cfg)
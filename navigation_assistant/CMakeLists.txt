cmake_minimum_required(VERSION 2.8.3)
project(navigation_assistant)
set (CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3")
## Find catkin macros and libraries
find_package(catkin REQUIRED)

## Find actionlib required stuff
find_package(catkin REQUIRED COMPONENTS
    roscpp
    std_msgs
    sensor_msgs
    geometry_msgs
    actionlib_msgs
    actionlib
    nav_msgs
    move_base
    topology_graph
    tf
    genmsg
    rviz
    message_generation
)

## Generate services in the 'srv' folder
add_service_files(
  FILES
  nav_assistant_point.srv
  nav_assistant_poi.srv
  nav_assistant_set_CNP.srv
  make_plan.srv  
)

## Generate actions in the 'action' folder
add_action_files(
   DIRECTORY action
   FILES
   nav_assistant.action
)

## Generate added messages and services with any dependencies listed here
generate_messages(
    DEPENDENCIES
    actionlib_msgs
    std_msgs 
    sensor_msgs 
    geometry_msgs
)


###################################
## catkin specific configuration ##
###################################
catkin_package(DEPENDS CATKIN
  CATKIN_DEPENDS message_runtime)

###########
## Build ##
###########
include_directories(${catkin_INCLUDE_DIRS})
link_directories(${catkin_LIBRARY_DIRS})

##########
##  QT  ##
##########
## This setting causes Qt's "MOC" generation to happen automatically.
set(CMAKE_AUTOMOC ON)

## This plugin includes Qt widgets, so we must include Qt.
## We'll use the version that rviz used so they are compatible.
if(rviz_QT_VERSION VERSION_LESS "5")
  message(STATUS "Using Qt4 based on the rviz_QT_VERSION: ${rviz_QT_VERSION}")
  find_package(Qt4 ${rviz_QT_VERSION} EXACT REQUIRED QtCore QtGui)
  ## pull in all required include dirs, define QT_LIBRARIES, etc.
  include(${QT_USE_FILE})
else()
  message(STATUS "Using Qt5 based on the rviz_QT_VERSION: ${rviz_QT_VERSION}")
  find_package(Qt5 ${rviz_QT_VERSION} EXACT REQUIRED Core Widgets)
  ## make target_link_libraries(${QT_LIBRARIES}) pull in all required dependencies
  set(QT_LIBRARIES Qt5::Widgets)
endif()

## I prefer the Qt signals and slots to avoid defining "emit", "slots",
## etc because they can conflict with boost signals, so define QT_NO_KEYWORDS here.
add_definitions(-DQT_NO_KEYWORDS)

###########
##  TOOL ##
###########
## list of source files that will compose the RVIZ tool
## The generated MOC files are included automatically as headers.
set(SRC_FILES
  src/set_nav_goal_tool.cpp
  src/set_point_of_interest_tool.cpp
)

## An rviz plugin is just a shared library
add_library(nav_assistant_tools ${SRC_FILES})
add_dependencies(nav_assistant_tools ${${PROJECT_NAME}_EXPORTED_TARGETS})

## Link the myviz executable with whatever Qt libraries have been defined
## Although this puts "rviz_plugin_tutorials" (or whatever you have
## called the project) as the name of the library, cmake knows it is a
## library and names the actual file something like
## "librviz_plugin_tutorials.so", or whatever is appropriate for your
## particular OS.
target_link_libraries(nav_assistant_tools ${QT_LIBRARIES} ${catkin_LIBRARIES})




###################
## NAV_ASSISTANT ##
###################
## Declare a C++ executable (Main Action Server)
add_executable(nav_assistant_node src/nav_assistant.cpp)

## Add cmake target dependencies of the executable
add_dependencies(nav_assistant_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} )

## Specify libraries to link a library or executable target against
target_link_libraries(nav_assistant_node  ${catkin_LIBRARIES} )

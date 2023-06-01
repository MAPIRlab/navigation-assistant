cmake_minimum_required(VERSION 3.5)
project(navigation_assistant)


set(CMAKE_BUILD_TYPE "None") 
set(CMAKE_CXX_FLAGS "-std=c++17 -g -O2 ${CMAKE_CXX_FLAGS}")



find_package(ament_cmake REQUIRED)
find_package(Boost REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(topology_graph REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)

find_package(angles REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

find_package(rviz_common REQUIRED)
find_package(rviz_default_plugins REQUIRED)
find_package(rviz_ogre_vendor REQUIRED)

# services and actions
find_package(rosidl_default_generators REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
    srv/NavAssistantPoint.srv
    srv/NavAssistantPOI.srv
    srv/NavAssistantSetCNP.srv
    srv/MakePlan.srv  
    action/NavAssistant.action
    DEPENDENCIES std_msgs geometry_msgs
)
ament_export_dependencies(rosidl_default_runtime)



include_directories(include/nav_assistant)

## MAIN NODE
add_executable(nav_assistant_node src/nav_assistant.cpp)

rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(nav_assistant_node "${cpp_typesupport_target}") 

ament_target_dependencies(nav_assistant_node
    rclcpp
    rclcpp_action
    visualization_msgs
    topology_graph
    tf2
    tf2_geometry_msgs
    angles
    nav2_msgs
    Boost
)




## RVIZ TOOLS

##########
##  QT  ##
##########
## This setting causes Qt's "MOC" generation to happen automatically.
set(CMAKE_AUTOMOC ON)

message(STATUS "Using Qt5 based on the rviz_QT_VERSION: ${rviz_QT_VERSION}")
find_package(Qt5 ${rviz_QT_VERSION} REQUIRED Core Widgets)
## make target_link_libraries(${QT_LIBRARIES}) pull in all required dependencies
set(QT_LIBRARIES Qt5::Widgets)

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
ament_target_dependencies(nav_assistant_tools 
    tf2
    tf2_geometry_msgs
    rviz_common
    rviz_default_plugins
    rviz_ogre_vendor
)


# dependency for self-defined messages
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(nav_assistant_tools "${cpp_typesupport_target}") 


## Link the myviz executable with whatever Qt libraries have been defined
## Although this puts "rviz_plugin_tutorials" (or whatever you have
## called the project) as the name of the library, cmake knows it is a
## library and names the actual file something like
## "librviz_plugin_tutorials.so", or whatever is appropriate for your
## particular OS.
target_link_libraries(nav_assistant_tools ${QT_LIBRARIES})





install(
    TARGETS  nav_assistant_tools
    DESTINATION lib/${PROJECT_NAME}
    )
    ament_package()

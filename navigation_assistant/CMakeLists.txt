cmake_minimum_required(VERSION 3.5)
project(navigation_assistant)


set(CMAKE_BUILD_TYPE "None") 
set(CMAKE_CXX_FLAGS "-std=c++17 -g -O3 ${CMAKE_CXX_FLAGS}")



find_package(ament_cmake REQUIRED)
find_package(Boost REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(topology_graph REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav_assistant_msgs REQUIRED)

find_package(angles REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

find_package(rviz_common REQUIRED)
find_package(rviz_default_plugins REQUIRED)
find_package(rviz_ogre_vendor REQUIRED)



include_directories(include/nav_assistant)

## MAIN NODE
add_executable(nav_assistant_node src/nav_assistant.cpp src/graph_actions.cpp)

ament_target_dependencies(nav_assistant_node
    rclcpp
    rclcpp_action
    nav_assistant_msgs
    visualization_msgs
    topology_graph
    tf2
    tf2_geometry_msgs
    angles
    nav2_msgs
    Boost
)

install(
    TARGETS nav_assistant_node
    DESTINATION lib/${PROJECT_NAME}
)




## RVIZ TOOLS

##########
##  QT  ##
########## 
## This setting causes Qt's "MOC" generation to happen automatically.
set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOMOC ON)

find_package(Qt5 ${rviz_QT_VERSION} REQUIRED Core Widgets)
set(QT_LIBRARIES Qt5::Widgets)
add_definitions(-DQT_NO_KEYWORDS)

set(SRC_FILES
src/set_nav_goal_tool.cpp
src/set_point_of_interest_tool.cpp
include/nav_assistant/set_nav_goal_tool.h
include/nav_assistant/set_point_of_interest_tool.h
)

## An rviz plugin is just a shared library
add_library(nav_assistant_tools SHARED ${SRC_FILES})
ament_target_dependencies(nav_assistant_tools 
    tf2
    tf2_geometry_msgs
    nav_assistant_msgs
    rviz_common
    rviz_default_plugins
    rviz_ogre_vendor
)
target_link_libraries(nav_assistant_tools ${QT_LIBRARIES})

target_compile_definitions(nav_assistant_tools PRIVATE "RVIZ_DEFAULT_PLUGINS_BUILDING_LIBRARY")
pluginlib_export_plugin_description_file(rviz_common plugin_description.xml)
ament_export_targets(nav_assistant_tools HAS_LIBRARY_TARGET)


install(
  DIRECTORY include/
  DESTINATION include/
)

install(
  TARGETS nav_assistant_tools
  EXPORT nav_assistant_tools
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)
ament_package()
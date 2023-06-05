PROJECT(topology_graph)

CMAKE_MINIMUM_REQUIRED(VERSION 3.5)



set(CMAKE_BUILD_TYPE "None") 
set(CMAKE_CXX_FLAGS "-std=c++17 -g -O0 ${CMAKE_CXX_FLAGS}")

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(spdlog REQUIRED)
find_package(Boost REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
    srv/Graph.srv
    DEPENDENCIES std_msgs
)
###########
## Build ##
###########


include_directories(
  include
)
## Declare a C++ executable
add_executable(topology_graph_node src/ahgraph.cpp  src/wrapper.cpp)

ament_target_dependencies(topology_graph_node
    rclcpp
    std_msgs
    nav2_msgs
    tf2_ros
    tf2_geometry_msgs
    Boost
    visualization_msgs
) 

# dependency for self-defined messages
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(topology_graph_node "${cpp_typesupport_target}") 

## Specify libraries to link a library or executable target against
target_link_libraries(topology_graph_node
    -lboost_graph spdlog::spdlog
)

install(
  TARGETS topology_graph_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
    FILES include/topology_graph/json/json.hpp
    DESTINATION include/topology_graph/topology_graph/json
)

ament_package()
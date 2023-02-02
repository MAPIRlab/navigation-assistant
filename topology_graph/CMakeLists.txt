PROJECT(topology_graph)

CMAKE_MINIMUM_REQUIRED(VERSION 3.3)

# Require C++17 (for MRPT v2.0)
if(${CMAKE_VERSION} VERSION_LESS "3.8.0")
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17")
else()
        set(CMAKE_CXX_STANDARD 17)
endif()


## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  message_generation
  roscpp
  std_msgs
  visualization_msgs
)

## Generate services in the 'srv' folder
add_service_files(
   FILES
   graph.srv
)

## Generate added messages and services with any dependencies listed here
generate_messages(
   DEPENDENCIES
   std_msgs
   visualization_msgs
)


catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES grafo_pkg
#  CATKIN_DEPENDS message_generation roscpp std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)


## Declare a C++ executable
add_executable(topology_graph src/ahgraph.cpp  src/wrapper.cpp)

## Add cmake target dependencies of the executable
## same as for the library above
add_dependencies(topology_graph topology_graph_gencpp)

## Specify libraries to link a library or executable target against
target_link_libraries(topology_graph
   ${catkin_LIBRARIES} -lboost_graph
)

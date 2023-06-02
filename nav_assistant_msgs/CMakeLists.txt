cmake_minimum_required(VERSION 3.8)
project(nav_assistant_msgs)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)


# services and actions
find_package(rosidl_default_generators REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
    srv/NavAssistantPoint.srv
    srv/NavAssistantPOI.srv
    srv/NavAssistantSetCNP.srv
    srv/MakePlan.srv  
    action/NavAssistant.action
    DEPENDENCIES std_msgs geometry_msgs nav_msgs
)
ament_export_dependencies(rosidl_default_runtime)



ament_package()

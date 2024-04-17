cmake_minimum_required(VERSION 3.8)
project(nas_test)

find_package(ament_cmake REQUIRED)

install(DIRECTORY 
    launch resources navigation_config
    DESTINATION share/${PROJECT_NAME}
)

ament_package()

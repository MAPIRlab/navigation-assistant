import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument,SetLaunchConfiguration,IncludeLaunchDescription,SetEnvironmentVariable,OpaqueFunction,GroupAction
from launch.launch_description_sources import FrontendLaunchDescriptionSource, PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, PushRosNamespace
from ament_index_python.packages import get_package_share_directory
from launch.frontend.parse_substitution import parse_substitution
import xacro

#===========================
def launch_arguments():
    return [
        DeclareLaunchArgument("worldFile", default_value="test.yaml"),
    ]
#==========================

def launch_setup(context, *args, **kwargs):
    test_dir = os.path.join(get_package_share_directory("nas_test"), "resources")

    # robot description for state_publisher
    robot_desc1 = xacro.process_file(
        os.path.join(test_dir, "giraff.xacro"),
        mappings={"frame_ns": "giraff1"},
    )
    robot_desc1 = robot_desc1.toprettyxml(indent="  ")

    visualization_node1 = GroupAction(
        [
            PushRosNamespace("giraff1"),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                parameters=[{"use_sim_time": True, "robot_description": robot_desc1}],
            )
        ]
    )

    basic_sim = Node(
            package="basic_sim",
            executable="basic_sim",
            prefix = "xterm -e",
            parameters=[
                {"deltaTime": 0.1},
                {"speed": 10.0},
                {"worldFile": os.path.join(test_dir, LaunchConfiguration("worldFile").perform(context))}
                ],
        )
    
    
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz",
        arguments=[
			"-d" + os.path.join(test_dir, "basic_sim.rviz")
		],
    )
    

    nav2_nodes = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(
                    get_package_share_directory("nas_test"),
                    "navigation_config",
                    "nav2_launch.py",
                )
            ]
        ),
        launch_arguments={
            "namespace": "giraff1",
            "map_yaml": os.path.join(test_dir, 'occupancy.yaml')
        }.items(),
    )
    

    nav_assistant = GroupAction(actions=[
            PushRosNamespace("giraff1"),
            Node(
                package="topology_graph",
                executable="topology_graph_node",
                name="topology_graph_node",
                prefix="xterm -e",
                parameters=[
                    {"verbose": True}
                ],
            ),
            Node(
                package="navigation_assistant",
                executable="nav_assistant_node",
                name="nav_assistant_node",
                prefix="xterm -e",
                parameters=[
                    {"verbose": True},
                    {"use_CNP": False},
                    {"init_from_file": ""},
                    {"save_to_file": ""},
                    {"robot_frame": "giraff1_base_link"},
                ],
            ),
            Node(
                package="nav_assistant_functions",
                executable="nav_assistant_functions_node",
                name="nav_assistant_functions",
                prefix="xterm -e",
                parameters=[
                    {"verbose": True}
                ],
            )
        ]
    )
    
    returnList = []
    
    returnList.append(basic_sim)
    returnList.append(rviz)
    returnList.append(visualization_node1)
    returnList.append(nav2_nodes)
    returnList.append(nav_assistant)

    return returnList


def generate_launch_description():

    launch_description = [
        # Set env var to print messages to stdout immediately
        SetEnvironmentVariable("RCUTILS_LOGGING_BUFFERED_STREAM", "1"),
        SetEnvironmentVariable("RCUTILS_COLORIZED_OUTPUT", "1"),
    ]
    
    launch_description.extend(launch_arguments())
    launch_description.append(OpaqueFunction(function=launch_setup))
    
    return  LaunchDescription(launch_description)
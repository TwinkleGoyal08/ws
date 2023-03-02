from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="cpp_parameters",
            executable="minimal_param_node",
            name="custom_minimal_param_node",
            output="screen",
            emulate_tty=True,
            parameters=[
                {"my_parameter": "earth"}
            ]
        )
    ])
# here executable name as set in CMakeLists.txt
# name any you like
# tty no idea yet
# for ensuring output is on screen

# FOR this terminal in which we ran launch script shows the change for one output 
# and later on same my world
# terminal where we ran our node has shown no change
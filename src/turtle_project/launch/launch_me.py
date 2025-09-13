from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        
        Node(
            package='turtlesim',
            executable='turtlesim_node',
            name='turtlesim'
        ),
        Node(
            package='turtle_project',
            executable='spawn_base_turtle',
            name='initiate_turtle'
        ),
        Node(
            package='turtle_project',
            executable='chase_turtle_mov',
            name='chase_turtle_mov'
        ),
        Node(
            package='turtle_project',
            executable='hunter_turtle',
            name='hunter_turtle'
        ),
    ])

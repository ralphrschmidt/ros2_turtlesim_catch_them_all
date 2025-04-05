from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    turlte_sim_node = Node(
        package="turtlesim",
        executable="turtlesim_node"
    )

    ld.add_action(turlte_sim_node)

    return ld

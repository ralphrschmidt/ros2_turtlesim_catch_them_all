# SPAWNER
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()
    turlte_spawner_node = Node(
        package="turtlesim_hunter_target_routine",
        executable="turtle_spawner_exe",
        parameters=[
            {"control_loop_frequency": 20},  # Frequency of the control loop (Hz)
            {"turtle_spawn_frequency": 2000},  # Time between turtle spawns (milliseconds)
            {"max_turtles": 5},  # Maximum number of turtles to spawn
            {"seed": 42},  # Random seed for consistent spawning behavior
            {"debug": 0},  # Debug mode toggle (0 = off, 1 = on)
            {"loggerlv": 0}  # Logging verbosity level (1 = info, higher = more verbose)
        ]
    )

    ld.add_action(turlte_spawner_node)

    return ld

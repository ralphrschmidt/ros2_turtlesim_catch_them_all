# CONTROLLER
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    turtle_controller_node = Node(
        package="turtlesim_hunter_target_routine",
        executable="turtle_controller_exe",
        parameters=[
            {"control_loop_frequency": 20},  # Control loop update frequency (Hz)
            {"min_distance_to_reach": 1.0},  # The minimum distance to target to consider it reached
            {"distance_threshold": 0.1},  # The minimum distance the turtle hunter can be to the target, for it to turn before moving
            {"turn_factor_threshold": 1.0},  # Minimum turn factor before turtle hunter will turn before moving
            {"vel_kp_ki_kd": [0.5, 0.1, 0.05]},  # Velocity PID parameters
            {"ang_kp_ki_kd": [2.0, 0.05, 0.02]},  # Angular PID parameters
            {"vel_max_integral": 10.0},  # Max integral cap for velocity PID
            {"ang_max_integral": 10.0},  # Max integral cap for angular PID
            {"debug": 0},  # Debug mode toggle (0 = off, 1 = on)
            {"loggerlv": 0}  # Logging verbosity level (1 = info, higher = more verbose)
        ]
    )

    ld.add_action(turtle_controller_node)

    return ld

import os
import launch
import launch_ros.actions

def generate_launch_description():
    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package='obstacle_avoidance',
            executable='obstacle_avoidance_node',
            name='laser_scan_subscriber',
            output='screen',
            parameters=[{
                'use_sim_time': False,  # Set to True if using simulation
            }]
        )
    ])


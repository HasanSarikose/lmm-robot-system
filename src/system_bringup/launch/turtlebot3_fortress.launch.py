import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node

def generate_launch_description():
    world_file = os.path.expanduser(
        '~/lmm_robot_ws/src/system_bringup/launch/turtlebot3_world.sdf'
    )

    return LaunchDescription([
        ExecuteProcess(
            cmd=['ign', 'gazebo', world_file, '-r'],
            output='screen'
        ),

        TimerAction(
            period=5.0,
            actions=[
                Node(
                    package='ros_gz_bridge',
                    executable='parameter_bridge',
                    arguments=[
                        '/cmd_vel@geometry_msgs/msg/Twist@ignition.msgs.Twist',
                        '/odom@nav_msgs/msg/Odometry@ignition.msgs.Odometry',
                    ],
                    output='screen'
                )
            ]
        ),
    ])

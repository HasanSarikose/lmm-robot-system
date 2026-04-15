import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node

def generate_launch_description():
    world_file = os.path.expanduser(
        '~/lmm_robot_ws/src/system_bringup/launch/multi_robot_world.sdf'
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
                        '/ugv_cmd_vel@geometry_msgs/msg/Twist@ignition.msgs.Twist',
                        '/ugv_odom@nav_msgs/msg/Odometry@ignition.msgs.Odometry',
                        '/arm_joint1_cmd@std_msgs/msg/Float64@ignition.msgs.Double',
                        '/arm_joint2_cmd@std_msgs/msg/Float64@ignition.msgs.Double',
                        '/arm_joint3_cmd@std_msgs/msg/Float64@ignition.msgs.Double',
                        '/arm_joint4_cmd@std_msgs/msg/Float64@ignition.msgs.Double',
                        '/arm_joint_states@sensor_msgs/msg/JointState@ignition.msgs.Model',
                    ],
                    output='screen'
                )
            ]
        ),
    ])

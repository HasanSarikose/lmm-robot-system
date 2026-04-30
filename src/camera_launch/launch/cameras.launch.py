from launch import LaunchDescription
from launch.actions import ExecuteProcess

def generate_launch_description():

    return LaunchDescription([

        # 🚁 DRONE
        ExecuteProcess(
            cmd=['python3', '/home/hasan/lmm_robot_ws/capture_drone.py'],
            output='screen'
        ),

        # 🚗 IKA
        ExecuteProcess(
            cmd=['python3', '/home/hasan/lmm_robot_ws/capture_ika.py'],
            output='screen'
        ),

        # 🤖 ARM
        ExecuteProcess(
            cmd=['python3', '/home/hasan/lmm_robot_ws/capture_arm.py'],
            output='screen'
        ),

    ])

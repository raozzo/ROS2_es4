import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    
    # Launch for tutor's launchfile 
    ir_launch_package_share = get_package_share_directory('ir_launch')

    exercise_4_launch_file = os.path.join(
        ir_launch_package_share,
        'launch',
        'exercise_4.launch.py'
    )

    include_exercise_4_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(exercise_4_launch_file)
    )
    
    #Launch for the server
    turtlenode_server = Node(
        package='group14_ex4',
        executable='turtlenode_server',  
        output='screen'  # Shows print/log statements in the terminal
    )

    #Launch for the clinet
    burrow_client = Node(
        package='group14_ex4',
        executable='burrow_client',
        output='screen'
    )

    # --- 3. Return the LaunchDescription ---
    return LaunchDescription([
        include_exercise_4_launch,
        turtlenode_server,
        burrow_client
    ])



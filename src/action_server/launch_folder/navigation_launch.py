import os

import launch
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, RegisterEventHandler, EmitEvent
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    # Retrieve the share directory of the external robot package
    bme_pkg = get_package_share_directory('bme_gazebo_sensors')

    # Define the client node separately so we can attach an event handler to it
    client_node = Node(
        package='action_server',
        executable='nav_action_client_node',
        name='nav_action_client',
        parameters=[{'use_sim_time': True}],
        prefix='gnome-terminal --wait --'
    )

    return launch.LaunchDescription([

        # 1. Launch Gazebo simulation and spawn the robot
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(bme_pkg, 'launch', 'spawn_robot.launch.py')
            ),
            launch_arguments={
                'world': 'my.sdf',
                'use_sim_time': 'True'
            }.items()
        ),

        # 2. Start the action server node in a separate terminal
        Node(
            package='action_server',
            executable='nav_action_server_node',
            name='nav_action_server',
            parameters=[{'use_sim_time': True}],
            prefix='gnome-terminal --wait --',
            output='screen'
        ),

        # 3. ROS-Gazebo bridge: odometry, velocity commands, and TF
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            parameters=[{'use_sim_time': True}],
            arguments=[
                '/model/mogi_bot/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
                '/model/mogi_bot/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
                '/model/mogi_bot/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V'
            ],
            remappings=[
                ('/model/mogi_bot/odom', '/odom'),
                ('/model/mogi_bot/cmd_vel', '/cmd_vel'),
                ('/model/mogi_bot/tf', '/tf')
            ],
            output='screen'
        ),

        # 4. Start the action client node in a separate terminal
        client_node,

        # Shut down the entire system when the client node exits
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=client_node,
                on_exit=[EmitEvent(event=Shutdown())]
            )
        )
    ])

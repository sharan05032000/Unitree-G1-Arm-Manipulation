from launch import LaunchDescription
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node

def generate_launch_description():
    node_a = Node(
        package='g1_move',
        executable='default_pick_node',
        output='screen',
    )



    node_b = Node(
        package='g1_move',
        executable='move_place_node',
        output='screen',
    )

    # Launch node B after node A exits
    event_handler_b = RegisterEventHandler(
        OnProcessExit(
            target_action=node_a,
            on_exit=[node_b],
        )
    )



    return LaunchDescription([
        node_a,
        event_handler_b,

    ])


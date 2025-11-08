from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_demo_launch
from launch_ros.actions import Node
from launch import LaunchDescription


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("g1_29dof_with_hand", package_name="g1_arm").to_moveit_configs()
   
 # Generate MoveIt demo launch description
    moveit_launch = generate_demo_launch(moveit_config)

    # Define additional nodes
    custom_node_1 = Node(
        package="g1_move",
        executable="unitree_arm_zero_node",
        name="unitree_arm_zero_node",
        output="screen",
    )

    custom_node_2 = Node(
        package="g1_move",
        executable="moveit_to_unitree_node",
        name="moveit_to_unitree_arm",
        output="screen",
    )

    # Combine everything into a single LaunchDescription
    return LaunchDescription(
        moveit_launch.entities + [custom_node_1, custom_node_2]
    )

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    trajectory_type = DeclareLaunchArgument(
        'trajectory_type',
        default_value='waypoint',
        description='Type of trajectory for the drone to follow. Valid values: waypoint, circle.')

    waypoint_x = DeclareLaunchArgument(
        'waypoint_x',
        default_value="0.0",
        description='X waypoint for trajectory'
    )
    waypoint_y = DeclareLaunchArgument(
        'waypoint_y',
        default_value="0.0",
        description='Y waypoint for trajectory'
    )
    radius = DeclareLaunchArgument(
        'radius',
        default_value='0.0',
        description='Radius for circle trajectory'
    )

    lqr_controller_node = Node(
        package='lqr_controller',
        executable='lqr_controller_node',
        name='lqr_controller_node',
        output='screen',
        parameters=[{
            'trajectory_type': LaunchConfiguration('trajectory_type'),
            'waypoint_x': LaunchConfiguration('waypoint_x'),
            'waypoint_y': LaunchConfiguration('waypoint_y'),
            'radius': LaunchConfiguration('radius'),
        }],
    )
    dlq_k_matrix_calculator = Node(
        package='dlq_k_matrix_calculator',
        executable='dlq_k_matrix_calculator',
        name='dlq_k_matrix_calculator',
        output='screen',
    )

    return LaunchDescription([
        trajectory_type,
        waypoint_x,
        waypoint_y,
        radius,
        lqr_controller_node,
        dlq_k_matrix_calculator
    ])

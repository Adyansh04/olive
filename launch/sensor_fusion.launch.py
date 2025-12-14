"""
Launch file for the complete sensor fusion system
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Get package share directory
    pkg_share = FindPackageShare('olive')
    
    # Declare use_sim_time argument
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time from /clock'
    )
    
    # Configuration files
    # wheel_imu_config = PathJoinSubstitution([pkg_share, 'config', 'wheel_imu_odom.yaml'])
    lidar_config = PathJoinSubstitution([pkg_share, 'config', 'lidar_odom.yaml'])
    # visual_config = PathJoinSubstitution([pkg_share, 'config', 'visual_odom.yaml'])
    # fusion_config = PathJoinSubstitution([pkg_share, 'config', 'graph_fusion.yaml'])
    
    # # Wheel IMU Odometry Node
    # wheel_imu_node = Node(
    #     package='olive',
    #     executable='wheel_imu_odom_node',
    #     name='wheel_imu_odom_node',
    #     parameters=[wheel_imu_config],
    #     output='screen',
    #     emulate_tty=True
    # )
    
    # LiDAR Odometry Node
    lidar_node = Node(
        package='olive',
        executable='lidar_odom_node',
        name='lidar_odom_node',
        parameters=[
            lidar_config,
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ],
        output='screen',
        emulate_tty=True
    )
    
    # # Visual Odometry Node
    # visual_node = Node(
    #     package='olive',
    #     executable='visual_odom_node',
    #     name='visual_odom_node',
    #     parameters=[visual_config],
    #     output='screen',
    #     emulate_tty=True
    # )
    
    # # Graph Fusion Backend Node
    # fusion_node = Node(
    #     package='olive',
    #     executable='graph_fusion_node',
    #     name='graph_fusion_node',
    #     parameters=[fusion_config],
    #     output='screen',
    #     emulate_tty=True
    # )
    
    return LaunchDescription([
        use_sim_time_arg,
        # wheel_imu_node,
        lidar_node
        # visual_node,
        # fusion_node,
    ])

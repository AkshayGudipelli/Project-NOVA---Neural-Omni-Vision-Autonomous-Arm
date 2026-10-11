import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    nova_desc_share = get_package_share_directory('nova_description')

    # Target color argument (red, orange, yellow, green, cyan, blue, purple, magenta, white, black)
    target_color_arg = DeclareLaunchArgument(
        'target_color',
        default_value='red',
        description='Target object color to detect and pick (red, orange, yellow, green, cyan, blue, purple, magenta, white, black)'
    )

    # 1. Include Base Sim and RViz Launch (TF, RViz2, Joint Controller, Robot State Publisher)
    sim_and_rviz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nova_desc_share, 'launch', 'sim_and_rviz.launch.py')
        )
    )

    # 2. 3D Perception Node (HSV segmentation + Depth Back-Projection -> /target_object_pose)
    object_detector_node = Node(
        package='nova_vision',
        executable='object_detector_node',
        name='object_detector_node',
        output='screen',
        parameters=[{
            'target_color': LaunchConfiguration('target_color')
        }]
    )

    # 3. Autonomous Pick-and-Place State Machine (Phase 6 Coordinator)
    pick_place_sm_node = Node(
        package='controller',
        executable='pick_place_sm_node',
        name='pick_place_sm_node',
        output='screen',
        parameters=[{
            'drop_x': -0.10,
            'drop_y': 0.14,
            'drop_z': 0.04,
            'approach_offset_z': 0.07,
            'grasp_offset_z': -0.018,
            'lift_offset_z': 0.10,
            'transit_z': 0.12,
            'joint_tolerance': 0.04,
            'dwell_time_sec': 0.4,
            'gripper_open': 0.025,
            'gripper_close': 0.0
        }]
    )

    return LaunchDescription([
        target_color_arg,
        sim_and_rviz_launch,
        object_detector_node,
        pick_place_sm_node
    ])

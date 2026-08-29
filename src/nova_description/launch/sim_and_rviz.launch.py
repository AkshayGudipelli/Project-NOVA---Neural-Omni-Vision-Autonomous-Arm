import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    nova_desc_share = get_package_share_directory('nova_description')

    urdf_path = os.path.join(nova_desc_share, 'urdf', 'diy_6dof_arm.urdf')
    rviz_config_path = os.path.join(nova_desc_share, 'rviz', 'nova_view.rviz')

    with open(urdf_path, 'r') as infp:
        robot_desc = infp.read()

    # 1. Robot State Publisher (Calculates Forward Kinematics from /joint_states and publishes /tf)
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_desc}]
    )

    # 2. Static Transform Publisher for workspace_camera optical frame
    camera_tf_publisher_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='camera_tf_publisher',
        output='screen',
        arguments=['--x', '0.0', '--y', '-0.28', '--z', '0.88',
                   '--roll', '0.68', '--pitch', '0.0', '--yaw', '0.0',
                   '--frame-id', 'base_link', '--child-frame-id', 'camera_optical_frame']
    )

    # 3. Hardware-Ready Joint Controller Node (Smooth velocity profiling & limit safety)
    joint_controller_node = Node(
        package='controller',
        executable='joint_controller_node',
        name='joint_controller_node',
        output='screen',
        parameters=[{'max_joint_velocity': 1.2}]
    )

    # 4. RViz2 loaded with our custom world-frame config
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_path]
    )

    return LaunchDescription([
        robot_state_publisher_node,
        camera_tf_publisher_node,
        joint_controller_node,
        rviz_node
    ])

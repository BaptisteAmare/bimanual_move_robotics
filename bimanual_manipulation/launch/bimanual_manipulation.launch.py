# Launch the bimanual manipulation server.
#
# The URDF is taken from the /robot_description topic by default (published by
# robot_state_publisher in your robot bringup). All four YAML config files can
# be overridden from the command line, e.g.:
#
#   ros2 launch bimanual_manipulation bimanual_manipulation.launch.py \
#       move_groups_config:=/path/to/my_move_groups.yaml
#
# To feed the URDF directly instead of via topic, set robot_description:=...
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('bimanual_manipulation')
    cfg = os.path.join(pkg, 'config')

    args = [
        DeclareLaunchArgument(
            'move_groups_config', default_value=os.path.join(cfg, 'move_groups.yaml')),
        DeclareLaunchArgument(
            'named_poses_config', default_value=os.path.join(cfg, 'named_poses.yaml')),
        DeclareLaunchArgument(
            'collision_config', default_value=os.path.join(cfg, 'collision.yaml')),
        DeclareLaunchArgument(
            'sequences_config', default_value=os.path.join(cfg, 'sequences.yaml')),
        DeclareLaunchArgument(
            'srdf_config', default_value='',
            description='Optional MoveIt .srdf; its disable_collisions feed the ACM.'),
        DeclareLaunchArgument('robot_description', default_value=''),
        DeclareLaunchArgument(
            'robot_description_file', default_value='',
            description='Path to a URDF with collision/visual meshes; takes priority '
                        'over the topic. Use this when the live /robot_description is '
                        'a kinematics-only URDF (no geometry).'),
        DeclareLaunchArgument('robot_description_topic', default_value='/robot_description'),
        DeclareLaunchArgument('gripper_max_effort', default_value='50.0'),
    ]

    node = Node(
        package='bimanual_manipulation',
        executable='manipulation_server_node',
        name='bimanual_manipulation_server',
        output='screen',
        parameters=[{
            'move_groups_config': LaunchConfiguration('move_groups_config'),
            'named_poses_config': LaunchConfiguration('named_poses_config'),
            'collision_config': LaunchConfiguration('collision_config'),
            'sequences_config': LaunchConfiguration('sequences_config'),
            'srdf_config': LaunchConfiguration('srdf_config'),
            'robot_description': LaunchConfiguration('robot_description'),
            'robot_description_file': LaunchConfiguration('robot_description_file'),
            'robot_description_topic': LaunchConfiguration('robot_description_topic'),
            'gripper_max_effort': LaunchConfiguration('gripper_max_effort'),
        }],
    )

    return LaunchDescription(args + [node])

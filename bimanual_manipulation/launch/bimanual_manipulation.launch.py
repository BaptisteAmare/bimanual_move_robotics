# Launch the bimanual manipulation server.
#
# Everything robot-specific lives in YAML + the SRDF, nothing in the code, so
# supporting several robots is just a matter of pointing at a different config
# folder. Put each robot's files in their own directory and select it with one
# argument:
#
#   ros2 launch bimanual_manipulation bimanual_manipulation.launch.py \
#       config_dir:=/path/to/config/genie \
#       srdf_config:=/path/to/genie.srdf \
#       robot_description_file:=/path/to/genie.urdf
#
# `config_dir` sets where move_groups.yaml / named_poses.yaml / collision.yaml /
# sequences.yaml are read from (default: this package's config/). Any single file
# can still be overridden individually. The URDF is taken from the
# /robot_description topic by default; set robot_description_file:=... to feed a
# full-geometry URDF from disk instead.
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('bimanual_manipulation')
    cfg = os.path.join(pkg, 'config')

    def in_config_dir(name):
        return PathJoinSubstitution([LaunchConfiguration('config_dir'), name])

    args = [
        DeclareLaunchArgument(
            'config_dir', default_value=cfg,
            description='Directory holding this robot\'s move_groups.yaml / '
                        'named_poses.yaml / collision.yaml / sequences.yaml. '
                        'One folder per robot; select it here.'),
        DeclareLaunchArgument(
            'move_groups_config', default_value=in_config_dir('move_groups.yaml')),
        DeclareLaunchArgument(
            'named_poses_config', default_value=in_config_dir('named_poses.yaml')),
        DeclareLaunchArgument(
            'collision_config', default_value=in_config_dir('collision.yaml')),
        DeclareLaunchArgument(
            'sequences_config', default_value=in_config_dir('sequences.yaml')),
        DeclareLaunchArgument(
            'srdf_config', default_value=os.path.join(cfg, 'walker_s2_augmented.srdf'),
            description='MoveIt .srdf; its disable_collisions feed the ACM. Pass your '
                        'robot\'s SRDF, or "" to rely on auto-disable only.'),
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

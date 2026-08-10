# Launch the bimanual manipulation server.
#
# Everything robot-specific lives in one folder per robot, with standard file
# names. Switching robots is a single argument:
#
#     ros2 launch bimanual_manipulation bimanual_manipulation.launch.py robot:=genie
#
# `robot:=<name>` reads config/<name>/ :
#     move_groups.yaml  named_poses.yaml  collision.yaml  sequences.yaml
#     model.srdf   (optional — used if present)
#     model.urdf   (optional — a full-geometry URDF; used if present, else the
#                   /robot_description topic is used)
#
# Point `config_dir` at any absolute folder to use configs outside the package.
# Any single file can still be overridden (move_groups_config, srdf_config, ...).
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    pkg = get_package_share_directory('bimanual_manipulation')
    robot = LaunchConfiguration('robot').perform(context)
    config_dir = LaunchConfiguration('config_dir').perform(context) \
        or os.path.join(pkg, 'config', robot)

    def pick(arg_name, filename, required):
        # An explicit override wins; else use config_dir/filename. Optional
        # files (srdf/urdf) resolve to '' when absent so the server skips them.
        override = LaunchConfiguration(arg_name).perform(context)
        if override:
            return override
        path = os.path.join(config_dir, filename)
        if required or os.path.exists(path):
            return path
        return ''

    if not os.path.isdir(config_dir):
        raise RuntimeError(
            "config folder '%s' not found — pass a valid robot:=<name> "
            "(a folder under config/) or config_dir:=<path>." % config_dir)

    params = {
        'move_groups_config': pick('move_groups_config', 'move_groups.yaml', True),
        'named_poses_config': pick('named_poses_config', 'named_poses.yaml', True),
        'collision_config': pick('collision_config', 'collision.yaml', True),
        'sequences_config': pick('sequences_config', 'sequences.yaml', True),
        'srdf_config': pick('srdf_config', 'model.srdf', False),
        'robot_description': LaunchConfiguration('robot_description').perform(context),
        'robot_description_file': pick('robot_description_file', 'model.urdf', False),
        'robot_description_topic': LaunchConfiguration('robot_description_topic').perform(context),
        'gripper_max_effort': float(LaunchConfiguration('gripper_max_effort').perform(context)),
    }

    return [Node(
        package='bimanual_manipulation',
        executable='manipulation_server_node',
        name='bimanual_manipulation_server',
        output='screen',
        parameters=[params],
    )]


def generate_launch_description():
    args = [
        DeclareLaunchArgument(
            'robot', default_value='walker_s2',
            description='Robot config folder under config/ (config/<robot>/). '
                        'The one flag you need to switch robots.'),
        DeclareLaunchArgument(
            'config_dir', default_value='',
            description='Absolute config folder; overrides config/<robot>/.'),
        # Optional per-file overrides (empty -> taken from the robot folder).
        DeclareLaunchArgument('move_groups_config', default_value=''),
        DeclareLaunchArgument('named_poses_config', default_value=''),
        DeclareLaunchArgument('collision_config', default_value=''),
        DeclareLaunchArgument('sequences_config', default_value=''),
        DeclareLaunchArgument('srdf_config', default_value=''),
        DeclareLaunchArgument('robot_description', default_value=''),
        DeclareLaunchArgument('robot_description_file', default_value=''),
        DeclareLaunchArgument('robot_description_topic', default_value='/robot_description'),
        DeclareLaunchArgument('gripper_max_effort', default_value='50.0'),
    ]
    return LaunchDescription(args + [OpaqueFunction(function=launch_setup)])

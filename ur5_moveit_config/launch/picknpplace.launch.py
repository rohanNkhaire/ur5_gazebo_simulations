import os
from pathlib import Path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder
from launch.substitutions import LaunchConfiguration, TextSubstitution

def generate_launch_description():
    executable_script = LaunchConfiguration('executable_script')
    executable_script_launch_arg = DeclareLaunchArgument(
        'executable_script',
        default_value=''
    )

    ur5_moveit_config = (
        MoveItConfigsBuilder(robot_name="ur", package_name="ur5_moveit_config")
        .robot_description_semantic(Path("srdf") / "ur.srdf", {"name": "ur5"})
        .planning_pipelines("ompl", ["ompl", "chomp", "pilz_industrial_motion_planner", "stomp"])
        .moveit_cpp(file_path=get_package_share_directory("ur5_moveit_config")
            + "/config/moveit_cpp.yaml")
        .to_moveit_configs()
    )

    # PicknPlace script
    picknplace_demo = Node(
        name="moveit_py",
        package="ur5_moveit_config",
        executable=executable_script,
        output="screen",
        parameters=[
            ur5_moveit_config.to_dict(),
            {"use_sim_time": True}
        ],
    )

    return LaunchDescription([executable_script_launch_arg, picknplace_demo])
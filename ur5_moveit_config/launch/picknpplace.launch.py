import os
from pathlib import Path
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():

    ur5_moveit_config = (
        MoveItConfigsBuilder(robot_name="ur", package_name="ur5_moveit_config")
        .robot_description_semantic(Path("srdf") / "ur.srdf", {"name": "ur5"})
        .planning_pipelines("ompl", ["ompl", "chomp", "pilz_industrial_motion_planner"])
        .moveit_cpp(file_path=get_package_share_directory("ur5_moveit_config")
            + "/config/moveit_cpp.yaml")
        .to_moveit_configs()
    )

     # PicknPlace script
    picknplace_demo = Node(
        name="moveit_py",
        package="ur5_moveit_config",
        executable="picknplace",
        output="screen",
        parameters=[
            ur5_moveit_config.to_dict(),
            {"use_sim_time": True}
        ],
    )

    return LaunchDescription([picknplace_demo])
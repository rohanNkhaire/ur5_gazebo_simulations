#!/usr/bin/env python3

# Importing the template
from code_template import ComKinAware

# Testing pick and place of cube
if __name__ == "__main__":
    # Params
    robot_arm_name = "ur5_arm"
    gripper_name = "gripper"
    gripper_joints = ["robotiq_85_left_knuckle_joint", "robotiq_85_right_knuckle_joint"]
    gripper_length = 0.32

    template = ComKinAware(robot_arm=robot_arm_name, gripper=gripper_name,
                           gripper_joint_names=gripper_joints, gripper_length=gripper_length)
    
    # test Goto Goal
    pick_pose = {
        "x": 0.4,
        "y": -0.04,
        "z": -0.1213,
        "roll": 0.0,
        "pitch": 1.57,
        "yaw": 1.57 
    }
    gripper_vec = [0, 0, -1]

    template.pick(pick_pose, gripper_vector=gripper_vec)

    place_pose = {
        "x": -0.45,
        "y": -0.08,
        "z": 0.2467,
        "roll": 0.0,
        "pitch": 1.57,
        "yaw": 1.57 
    }
    template.place(place_pose, gripper_vector=gripper_vec)
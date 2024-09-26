#!/usr/bin/env python

###################################
# This file provides a complete example of a pick and place task
# using the moveit_commander python api. 
###################################

import sys
import rospy
import copy
from moveit_commander import RobotCommander, PlanningSceneInterface, roscpp_initialize, roscpp_shutdown
from geometry_msgs.msg import PoseStamped, Quaternion, Pose
from math import pi
from moveit_msgs import msg
from trajectory_msgs.msg import JointTrajectoryPoint
from tf.transformations import quaternion_from_euler
from gazebo_ros_link_attacher.srv import SetStatic, SetStaticRequest, SetStaticResponse
from gazebo_ros_link_attacher.srv import Attach, AttachRequest, AttachResponse

def close_gripper(gripper_group, gazebo_model_name=None):
    joint_goal = gripper_group.get_current_joint_values()
    joint_goal[0] = 0.45

    # The go command can be called with joint values, poses, or without any
    # parameters if you have already set the pose or joint target for the group
    gripper_group.go(joint_goal, wait=True)

    # Calling ``stop()`` ensures that there is no residual movement
    gripper_group.stop()

    # Destroy dynamic joint
    if gazebo_model_name is not None:
        req = AttachRequest()
        req.model_name_1 = gazebo_model_name
        req.link_name_1 = "my_box"
        req.model_name_2 = "robot"
        req.link_name_2 = "wrist_3_link"
        attach_srv.call(req)


def open_gripper(gripper_group, gazebo_model_name=None):
    joint_goal = gripper_group.get_current_joint_values()
    joint_goal[0] = 0.1

    # The go command can be called with joint values, poses, or without any
    # parameters if you have already set the pose or joint target for the group
    gripper_group.go(joint_goal, wait=True)

    # Calling ``stop()`` ensures that there is no residual movement
    gripper_group.stop()

    # Destroy dynamic joint
    if gazebo_model_name is not None:
        req = AttachRequest()
        req.model_name_1 = gazebo_model_name
        req.link_name_1 = "my_box"
        req.model_name_2 = "robot"
        req.link_name_2 = "wrist_3_link"
        detach_srv.call(req)

def pick(move_group, gripper_group):
    #Specifies, plans, and executes a pick operation.

    pose_goal = Pose()
    q = quaternion_from_euler(-1.57, 1.571, 3.14)
    pose_goal.orientation = Quaternion(q[0],q[1],q[2],q[3])
    pose_goal.position.x = -0.5
    pose_goal.position.y = -0.5
    pose_goal.position.z = 1.25
    move_group.set_pose_target(pose_goal)

    move_group.go(wait=True)
    move_group.stop()
    move_group.clear_pose_targets()
    # Finally we specify the parameters for the open and close
    # posture of the grasp 
    open_gripper(gripper_group, "my_object")

    pose_goal = Pose()
    q = quaternion_from_euler(-1.57, 1.571, 3.14)
    pose_goal.orientation = Quaternion(q[0],q[1],q[2],q[3])
    pose_goal.position.x = -0.5
    pose_goal.position.y = -0.5
    pose_goal.position.z = 1.0  # Adjusted to avoid collision
    move_group.set_pose_target(pose_goal)

    move_group.go(wait=True)
    move_group.stop()
    move_group.clear_pose_targets()
    close_gripper(gripper_group, "my_object")

    pose_goal = Pose()
    q = quaternion_from_euler(-1.57, 1.571, 3.14)
    pose_goal.orientation = Quaternion(q[0],q[1],q[2],q[3])
    pose_goal.position.x = -0.5
    pose_goal.position.y = -0.5
    pose_goal.position.z = 1.4
    move_group.set_pose_target(pose_goal)

    move_group.go(wait=True)
    move_group.stop()
    move_group.clear_pose_targets()

def place(move_group, gripper_group):
    #Specifies, plans, and executes a place operation.
    pose_goal = Pose()
    q = quaternion_from_euler(-pi/2, 1.57, 2.0)
    pose_goal.orientation = Quaternion(q[0],q[1],q[2],q[3])
    pose_goal.position.x = -0.75
    pose_goal.position.y = -0.15
    pose_goal.position.z = 1.4
    move_group.set_pose_target(pose_goal)

    move_group.go(wait=True)
    move_group.stop()
    move_group.clear_pose_targets()

    pose_goal = Pose()
    q = quaternion_from_euler(-pi/2, 1.57, 2.0)
    pose_goal.orientation = Quaternion(q[0],q[1],q[2],q[3])
    pose_goal.position.x = -0.75
    pose_goal.position.y = -0.15
    pose_goal.position.z = 1.0  # Adjusted to avoid collision
    move_group.set_pose_target(pose_goal)

    move_group.go(wait=True)
    move_group.stop()
    move_group.clear_pose_targets()
    # Finally we specify the parameters for the open and close
    # posture of the grasp 
    open_gripper(gripper_group, "my_object")

    rospy.sleep(2)

def goto_home(move_group):
    joint_goal = move_group.get_current_joint_values()
    joint_goal[0] = 0
    joint_goal[1] = -pi/2
    joint_goal[2] = -pi/2
    joint_goal[3] = -pi/2
    joint_goal[4] = pi/2
    joint_goal[5] = pi/2

    # The go command can be called with joint values, poses, or without any
    # parameters if you have already set the pose or joint target for the group
    move_group.go(joint_goal, wait=True) 

    # Calling ``stop()`` ensures that there is no residual movement
    move_group.stop()

if __name__=='__main__':
    # Initialise moveit and ROS node
    roscpp_initialize(sys.argv)
    rospy.init_node('moveit_py_demo', anonymous=True)

    # Get handles to the planning scene and robot 
    scene = PlanningSceneInterface()
    robot = RobotCommander()

    # Select planning group
    group = robot.get_group('ur5_arm')
    gripper = robot.get_group('gripper')
    # Set a liberal planner timeout 
    group.set_planning_time(seconds=45.0)
    # IMPORTANT: you must call sleep after the previous line 
    # to ensure the planning scene object is initialised before
    # using it.
    rospy.sleep(2)

    # Setting the attach/detach service
    setstatic_srv = rospy.ServiceProxy("/link_attacher_node/setstatic", SetStatic)
    attach_srv = rospy.ServiceProxy("/link_attacher_node/attach", Attach)
    detach_srv = rospy.ServiceProxy("/link_attacher_node/detach", Attach)
    setstatic_srv.wait_for_service()
    attach_srv.wait_for_service()
    detach_srv.wait_for_service()

    # IMPORTANT: you must call sleep after the previous line 
    # to ensure the planning scene object is initialised before
    # using it.
    rospy.sleep(2)

    # Goto home position
    goto_home(group)

    rospy.sleep(1)

    # Plan and execute the pick operation
    pick(group, gripper)

    rospy.sleep(1)

    place(group, gripper)
  
    # Goto home position
    goto_home(group)
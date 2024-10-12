"""
TODO:

1) Pick Function-> Home Position | Function Return logic | Updated dummy joint vals in self.Gripper(joint_val=0.0).
2) Place Function-> Home Position | Function Return logic | Updated dummy joint vals in self.Gripper(joint_val=0.0).
3) go_to_goal-> Function Return logic
4) Gripper -> Function Return logic

Note:
Absolute Summation in lines 124 and 92.
math.copysign(gripper_length + abs(value), value) -> 

"""
import time
from math import pi, copysign
# generic ros libraries
import rclpy
from rclpy.logging import get_logger

# moveit python library
from moveit.core.robot_state import RobotState
from moveit.planning import (
    MoveItPy,
    MultiPipelinePlanRequestParameters,
    PlanRequestParameters,)
from moveit.core.kinematic_constraints import construct_joint_constraint
from moveit_configs_utils import MoveItConfigsBuilder
from ament_index_python import get_package_share_directory
from moveit_msgs import msg
from geometry_msgs.msg import PoseStamped, Quaternion
from trajectory_msgs.msg import JointTrajectoryPoint

from tf_transformations import quaternion_from_euler

class ComKinAware():
    def __init__(self,
                 robot_arm:str,
                 gripper:str,
                 gripper_joint_names:list[str],
                 gripper_length:float,
                 end_effector_link= "flange",
                 frame_of_reference="base_link",
                 sleep_time:float= 0.0,
                 ) -> None:
        self.robot_arm= robot_arm
        self.gripper= gripper
        self.sleep_time= sleep_time
        #self.gripper_joint_dict= {}
        self.gripper_joint_names= gripper_joint_names # []
        self.end_effector_link= end_effector_link # End Effector Link
        self.frame_of_reference= frame_of_reference
        self.gripper_length= gripper_length
        

        # Initialise moveit and ROS node
        rclpy.init()
        self.logger = get_logger("moveit_py.pose_goal")

        # instantiate MoveItPy instance and get planning component
        self.robot = MoveItPy(node_name="moveit_py")
        self.robot_arm = self.robot.get_planning_component(self.robot_arm)
        self.gripper = self.robot.get_planning_component(self.gripper)
        self.logger.info("MoveItPy instance created")

        # instantiate a RobotState instance using the current robot model
        robot_model = self.robot.get_robot_model()
        self.robot_state = RobotState(robot_model)


    def pick(self,
            pose:dict,              # {X:,Y:,Z:, Roll:, Pitch:, Yaw:}
            gripper_vector:list,    # XYZ -> [0,0,1] compensate for gripper length in the z-direction.  # $$$$$ GRASP PLANNING $$$$$
            planner:str= "ompl_rrtc"):
        """Specifies, plans, and executes a pick operation."""

        # Compensate pose for gripper length.
        pose_values= list(pose.values())
        updated_pose_list= list(map(lambda x,y: x+y, list(map(lambda x:x*self.gripper_length,  gripper_vector)) , pose_values[:3])) + pose_values[3:]
        updated_pose_dict=  {i:j for i, j in zip(pose.keys(), updated_pose_list)}
        
        # set plan start state to current state
        self.robot_arm.set_start_state_to_current_state()
        
        self.go_to_goal(updated_pose_dict) 
        self.Gripper(joint_val=0.0) #  sample value 

        self.go_to_goal(pose)

        self.Gripper(joint_val=0.7) #  sample value 
        # TODO: self.go_to_goal(home_position)

    def place(self,
            pose:dict,              # {X,Y,Z, Roll, Pitch, Yaw}
            gripper_vector:list,    # XYZ -> [0,0,1] compensate for gripper length in the z-direction. # $$$$$ GRASP PLANNING $$$$$
            planner:str= "ompl_rrtc"):
        
        # set plan start state to current state
        self.robot_arm.set_start_state_to_current_state()

        # Compensate pose for gripper length.
        pose_values= list(pose.values())
        updated_pose_list= list(map(lambda x,y: x+y, list(map(lambda x:x*self.gripper_length,  gripper_vector)) , pose_values[:3])) + pose_values[3:]
        updated_pose_dict=  {i:j for i, j in zip(pose.keys(), updated_pose_list)}
        
        self.go_to_goal(updated_pose_dict)
        self.go_to_goal(pose)
        self.Gripper(joint_val=0.0) # 0.0 is a sample value to open.

    def go_to_goal(self, pose:dict, planner_name:str="ompl_rrtc"):
        "Go there."
        single_plan_request_params = PlanRequestParameters(
                                        self.robot, planner_name)
        
        # Create a goal position for picking
        pose_goal = PoseStamped()
        pose_goal.header.frame_id = self.frame_of_reference
        pose_goal.header.stamp = rclpy.time.Time().to_msg()

        quat = quaternion_from_euler(pose["roll"], pose["pitch"], pose["yaw"])
        pose_goal.pose.orientation.x = quat[0]
        pose_goal.pose.orientation.y = quat[1]
        pose_goal.pose.orientation.z = quat[2]
        pose_goal.pose.orientation.w = quat[3]

        pose_goal.pose.position.x = pose['x']
        pose_goal.pose.position.y = pose['y']
        pose_goal.pose.position.z = pose['z']

        self.robot_arm.set_goal_state(pose_stamped_msg=pose_goal, pose_link= self.end_effector_link)
        self._plan_and_execute(self.robot_arm, single_plan_parameters= single_plan_request_params)

        # TODO: return bool

    def Gripper (self, joint_val:float, planner_name:str="stomp_planner"):
        """ """
        joint_values = {
                self.gripper_joint_names[0]: joint_val,
                self.gripper_joint_names[1]: -joint_val,
        }

        # Set start state to current state.
        self.gripper.set_start_state_to_current_state()
        
        # Set robot state joint positions.
        self.robot_state.joint_positions= joint_values
        
        # Set Goal state.
        self.gripper.set_goal_state(robot_state= self.robot_state)
        single_plan_request_params = PlanRequestParameters(self.robot, planner_name)
        self._plan_and_execute(self.gripper, single_plan_parameters=single_plan_request_params)

        # TODO: Return Type

    def _plan_and_execute(self,
                          planning_component,
                          single_plan_parameters=None,
                        multi_plan_parameters=None):
        """Helper function to plan and execute a motion.
           Planning Componenet: Robot or gripper.
        """
        # plan to goal
        self.logger.info("Planning trajectory")
        if multi_plan_parameters is not None:
            plan_result = planning_component.plan(
                multi_plan_parameters=multi_plan_parameters
            )
        elif single_plan_parameters is not None:
            plan_result = planning_component.plan(
                single_plan_parameters=single_plan_parameters
            )
        else:
            plan_result = planning_component.plan()

        # execute the plan
        if plan_result:
            self.logger.info("Executing plan")
            robot_trajectory = plan_result.trajectory
            self.robot.execute(robot_trajectory, controllers=[])
        else:
            self.logger.error("Planning failed")

        time.sleep(self.sleep_time) 
#!/usr/bin/env python3
# node inspired by https://docs.pal-robotics.com/sdk/24.09/actions/arm_controller-follow_joint_trajectory.html#id1 

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from rclpy.signals import SignalHandlerOptions
import math
import time


class FollowJointTrajectoryActionClient(Node):
    def __init__(self):
        super().__init__('arm_controller_follow_joint_trajectory_client')
        self._action_client = ActionClient(
            self,                                # The ROS2 node using the client
            FollowJointTrajectory,               # The action type
            '/arm_controller/follow_joint_trajectory'  # The action server topic to connect to
        )

        ## arnavs code ----

        #  Joint limits - configurable via parameters for portability
        # Default values are from OpenManipulator-X URDF
        self.declare_parameter('joint_limits.joint1.lower', -math.pi*0.9)
        self.declare_parameter('joint_limits.joint1.upper', math.pi*0.9)
        self.declare_parameter('joint_limits.joint2.lower', -math.pi*0.57)
        self.declare_parameter('joint_limits.joint2.upper', math.pi*0.5)
        self.declare_parameter('joint_limits.joint3.lower', -math.pi*0.3)
        self.declare_parameter('joint_limits.joint3.upper', math.pi*0.44)
        self.declare_parameter('joint_limits.joint4.lower', -math.pi*0.57)
        self.declare_parameter('joint_limits.joint4.upper', math.pi*0.65)


        # Load joint limits from parameters (read once at startup)
        self.joint_limits = {}
        for joint_name in ['joint1', 'joint2', 'joint3', 'joint4']:
            lower = self.get_parameter(f'joint_limits.{joint_name}.lower').get_parameter_value().double_value
            upper = self.get_parameter(f'joint_limits.{joint_name}.upper').get_parameter_value().double_value
            self.joint_limits[joint_name] = (lower, upper)

        self.arm_joints = ["joint1", "joint2", "joint3", "joint4"]

        ## arnavs code ----


    def send_goal(self, a, b):
        goal_msg = FollowJointTrajectory.Goal()

        # TODO: adapt to the action's parameters
        # check https://docs.ros2.org/latest/api/control_msgs/action/FollowJointTrajectory.html
        # for the possible goal parameters
        # goal_msg.a = a
        # goal_msg.b = b

        # ===== Goal Parameters: ======

        # joint names string[]
        goal_msg.trajectory.joint_names = self.arm_joints # add joints to goal_msg


        # joint trajectory points float64[]
        # Each 'JointTrajectoryPoint()' describes where all joints (from trajectory.joint_names) should be at a specific time (from start) aka 'time_from_start'.
        # A list of 'points' describes the journey of points the robot should aim towards at different times, hence the motion is a smooth trajectory, with parameters that change with time (time_from_start).
        # =======
        # point = JointTrajectoryPoint() # Arnavs

        #   positions - REQUIRED - list of target positions for all joints listed in trajectory.joint_names
        # =======
        # point.positions = [] # placeholder for Arnavs calcs
        #   velocities
        #   accelerations
        #   effort
        #   time from start - REQUIRED - The time all joints need to reach these positions in. Need to be defined with point.positions (simple suvat or v=d/t)
        # =======
        # point.time_from_start.sec = 1  # Arnavs
        # point.time_from_start.nanosec = 0  # Arnavs

        # Add points to goal_msg
        # A list of 'JointTrajectoryPoint()'s the robot aims for (over the specified time). 
        # Aarnav's code only has one point in as it continuously only tries to aim for the aruco marker.
        # I've modified this so that mine has multiple positions to aim for over time.
        # =======
        # goal_msg.trajectory.points = [point] # Arnavs


        # +++++++++++ my modified points code +++++++++++

        point1 = JointTrajectoryPoint()
        point1.positions = [0.0, 0.0, 0.0, 0.0]    # Start (4 joints)
        point1.time_from_start.sec = 0

        point2 = JointTrajectoryPoint()  
        # point2.positions = [, , , ]    # Move here in 2s  - TODO: weave in joint limits?? - ASK ARNAV 
        point2.time_from_start.sec = 2

        goal_msg.trajectory.points = [point1, point2] # the journey

        # +++++++++++++++


        self._action_client.wait_for_server()

        return self._action_client.send_goal_async(goal_msg)


    def calculate_inverse_kinematics(self, target_direction):
        """
        Simple inverse kinematics for pointing the end-effector in a target direction.
        target_direction: [x, y, z] unit vector in robot base frame
        """
        x, y, z = target_direction
        
        # Calculate base rotation (joint1) - yaw
        joint1 = math.atan2(y, x)
        
        # Calculate shoulder angle (joint2) to point toward target
        # For a pointing gesture, we want the arm extended horizontally
        horizontal_dist = math.sqrt(x*x + y*y)
        joint2 = math.atan2(-z, horizontal_dist)  # Negative z for upward pointing
        
        # Set elbow (joint3) to extend the arm - configurable pointing pose
        joint3 = self.elbow_angle
        
        # Set wrist (joint4) to keep end-effector level
        joint4 = -joint2 - joint3 + math.radians(self.end_effector_angle)
        
        return [joint1, joint2, joint3, joint4]


if __name__ == '__main__':
    rclpy.init(args=args)

    action_client = FollowJointTrajectoryActionClient()

    # TODO: adapt to your action's parameters
    future = action_client.send_goal(a, b)

    rclpy.spin_until_future_complete(action_client, future)

    rclpy.shutdown()



# OWN main method
# def main(args=None):
#     rclpy.init(
#         args=args,
#         signal_handler_options=SignalHandlerOptions.NO,
#     )
#     action_client = FollowJointTrajectoryActionClient()

#     try:
#         # TODO: adapt to your action's parameters
#         # future = action_client.send_goal(a, b) # returns future once action is completed
#         # rclpy.spin_until_future_complete(action_client, future) # try inseatd of rclpy.spin(action_client) - continuous
#         rclpy.spin(action_client) 
#     except KeyboardInterrupt:
#         print(
#             f"{action_client.get_name()} received a shutdown request (Ctrl+C)."
#         )
#     finally:
#         action_client.on_shutdown()
#         while not action_client.shutdown:
#             continue
#         action_client.destroy_node()
#         rclpy.shutdown()

# if __name__ == "__main__":
#     main()


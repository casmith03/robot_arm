#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from std_msgs.msg import Float32
from sensor_msgs.msg import JointState
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from rclpy.action import ActionClient

import math
import time


class MoveArm(Node):
    def __init__(self):
        super().__init__("move_arm")

        # // ARNAVS CODE - pasted
        # Parameter: fixed end-effector (wrist) angle in degrees to stabilize final DOF
        self.declare_parameter('end_effector_angle', 0.0)
        self.end_effector_angle = (
            self.get_parameter('end_effector_angle').get_parameter_value().double_value
        )

        # Joint limits - configurable via parameters for portability
        # Default values are from OpenManipulator-X URDF
        self.declare_parameter('joint_limits.joint1.lower', -math.pi*0.9)
        self.declare_parameter('joint_limits.joint1.upper', math.pi*0.9)
        self.declare_parameter('joint_limits.joint2.lower', -math.pi*0.57)
        self.declare_parameter('joint_limits.joint2.upper', math.pi*0.5)
        self.declare_parameter('joint_limits.joint3.lower', -math.pi*0.3)
        self.declare_parameter('joint_limits.joint3.upper', math.pi*0.44)
        self.declare_parameter('joint_limits.joint4.lower', -math.pi*0.57)
        self.declare_parameter('joint_limits.joint4.upper', math.pi*0.65)

        # Q - whats the point of using declare_parameter here ??

        # Load joint limits from parameters (read once at startup)
        self.joint_limits = {}
        for joint_name in ['joint1', 'joint2', 'joint3', 'joint4']:
            lower = self.get_parameter(f'joint_limits.{joint_name}.lower').get_parameter_value().double_value
            upper = self.get_parameter(f'joint_limits.{joint_name}.upper').get_parameter_value().double_value
            self.joint_limits[joint_name] = (lower, upper)


        # i should need this right?
        self.joint_state_sub = self.create_subscription(JointState, "/joint_states", self.joint_state_callback, 10)


        # why do you not use :
            # /servo_node/delta_joint_cmds
            #/servo_node/delta_twist_cmds

        
        # // ARNAVS CODE - pasted

    

def main(args=None):
    rclpy.init(
        args=args,
        signal_handler_options=SignalHandlerOptions.NO,
    )
    node = MoveArm()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print(
            f"{node.get_name()} received a shutdown request (Ctrl+C)."
        )
    finally:
        node.on_shutdown()
        while not node.shutdown:
            continue
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()


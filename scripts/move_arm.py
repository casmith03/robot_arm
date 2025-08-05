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


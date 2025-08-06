#!/usr/bin/env python3
# node inspired by https://docs.pal-robotics.com/sdk/24.09/actions/arm_controller-follow_joint_trajectory.html#id1 

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from rclpy.signals import SignalHandlerOptions


class FollowJointTrajectoryActionClient(Node):
    def __init__(self):
        super().__init__('arm_controller_follow_joint_trajectory_client')
        self._action_client = ActionClient(
            self,                                # The ROS2 node using the client
            FollowJointTrajectory,               # The action type
            '/arm_controller/follow_joint_trajectory'  # The action server topic to connect to
        )

    def send_goal(self, a, b):
        goal_msg = FollowJointTrajectory.Goal()

        # TODO: adapt to the action's parameters
        # check https://docs.ros2.org/latest/api/control_msgs/action/FollowJointTrajectory.html
        # for the possible goal parameters
        # goal_msg.a = a
        # goal_msg.b = b

        self._action_client.wait_for_server()

        return self._action_client.send_goal_async(goal_msg)



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


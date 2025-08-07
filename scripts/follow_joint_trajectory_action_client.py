#!/usr/bin/env python3
# node inspired by https://docs.pal-robotics.com/sdk/24.09/actions/arm_controller-follow_joint_trajectory.html#id1 

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from sensor_msgs.msg import JointState
from rclpy.signals import SignalHandlerOptions
import math
import time


class FollowJointTrajectoryActionClient(Node):
    def __init__(self):
        super().__init__('arm_controller_follow_joint_trajectory_client')
        
        self._action_client = ActionClient(
            self,                                      # The ROS2 node using the client
            FollowJointTrajectory,                     # The action type
            '/arm_controller/follow_joint_trajectory'  # The action server topic to connect to
        )

        # Debug flag
        self.declare_parameter('debug_mode', False)
        self.debug_mode = self.get_parameter('debug_mode').get_parameter_value().bool_value

        self.shutdown = False

        ## arnavs code ------------------

        # #  Joint limits - configurable via parameters for portability
        # # Default values are from OpenManipulator-X URDF
        # self.declare_parameter('joint_limits.joint1.lower', -math.pi*0.9)
        # self.declare_parameter('joint_limits.joint1.upper', math.pi*0.9)
        # self.declare_parameter('joint_limits.joint2.lower', -math.pi*0.57)
        # self.declare_parameter('joint_limits.joint2.upper', math.pi*0.5)
        # self.declare_parameter('joint_limits.joint3.lower', -math.pi*0.3)
        # self.declare_parameter('joint_limits.joint3.upper', math.pi*0.44)
        # self.declare_parameter('joint_limits.joint4.lower', -math.pi*0.57)
        # self.declare_parameter('joint_limits.joint4.upper', math.pi*0.65)


        # # Load joint limits from parameters (read once at startup)
        # self.joint_limits = {}
        # for joint_name in ['joint1', 'joint2', 'joint3', 'joint4']:
        #     lower = self.get_parameter(f'joint_limits.{joint_name}.lower').get_parameter_value().double_value
        #     upper = self.get_parameter(f'joint_limits.{joint_name}.upper').get_parameter_value().double_value
        #     self.joint_limits[joint_name] = (lower, upper)

        self.arm_joints = ["joint1", "joint2", "joint3", "joint4"]
        self.current_positions = [0.0] * len(self.arm_joints) # for feedback??
        self.joint_states_received = False # TODO: what does this do?

        self.current_goal_handle = None
        self.get_result_future = None
        self.goal_succeeded = False

        # --- Subscribers : ---
        # Information about the joint states
        self.joint_state_sub = self.create_subscription(
            JointState, 
            "/joint_states", 
            self.joint_state_callback,
            10)
        
        ## arnavs code -------------------

        self.get_logger().info("Follow Joint Trajectory node initialized")


    # Credit: Robotics Assignment 
    def on_shutdown(self):
        """Custom behaviour on shutdown."""
        self.get_logger().info("Stopping the robot...")
        # TODO: publish commands to stop all motion - option to reutrn to initial joint states?? or does it do this already?
        self.shutdown = True


    # Credit: Arnavs
    # TODO: what is this useful for?
    # poupulates initial joint states - only once?
    def joint_state_callback(self, msg: JointState):
        for i, name in enumerate(self.arm_joints):
            if name in msg.name:
                idx = msg.name.index(name)
                self.current_positions[i] = msg.position[idx]
        
        if not self.joint_states_received:
            self.joint_states_received = True
            # self.get_logger().info("Initial joint states received")

        # print initial joint states
        if self.debug_mode:
            self.get_logger().info(f"{self.current_positions}")


    # # Credit: Arnavs
    # # Continually sends goal to follow marker ... I dont need this in my code, but could be useful to make an auto-updating arm sequence 
    # def update_arm_position(self):
    #     if not self.joint_states_received:
    #         return
        
    #     # # Rate limiting
    #     # current_time = time.time()
    #     # if current_time - self.last_goal_time < self.min_goal_interval:
    #     #     return
            
    #     # self.send_joint_goal(self.latest_yaw, self.latest_pitch) # sends new goal - autonomous
    #     # self.last_goal_time = current_time



    #  update params to 'journey' ?? or should i make the journey multiple goals, ie one goals = one waypoint on the journey??
    """
        # joint trajectory points float64[]
        # Each 'JointTrajectoryPoint()' describes where all joints (from trajectory.joint_names) should be at a specific time (from start) aka 'time_from_start'.
        # A list of 'points' describes the journey of points the robot should aim towards at different times, hence the motion is a smooth trajectory, with parameters that change with time (time_from_start).
        # =======
        # point = JointTrajectoryPoint() # Arnavs

        #   positions - REQUIRED - list of target positions for all joints listed in trajectory.joint_names. They are joint angles in rad.
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


        # TODO: adapt to the action's parameters
        # check https://docs.ros2.org/latest/api/control_msgs/action/FollowJointTrajectory.html
        # for the possible goal parameters
        # goal_msg.a = a
        # goal_msg.b = b
    """
    def send_goal(self, joint_angles): # TODO: update params for a journey / multiple waypoints? or not ... just one waypoint but more info eg time, & what other params?

        if not self._action_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().warn("Action server not available!")
            return

        # Cancel previous goal # Arnavs
        if self.current_goal_handle is not None:
            self.current_goal_handle.cancel_goal_async()
    
        goal_msg = FollowJointTrajectory.Goal()

        # joint names string[]
        goal_msg.trajectory.joint_names = self.arm_joints # add joints to goal_msg

        point1 = JointTrajectoryPoint()
        point1.positions = [0.0, 0.0, 0.0, 0.0] # Start (4 joints) # moves to start pos first every time new goal is sent 
        point1.time_from_start.sec = 2

        point2 = JointTrajectoryPoint()  
        # point2.positions = [, , , ] # joint angles in rad - TODO: weave in joint limits?? - ASK ARNAV 
        point2.positions = joint_angles
        point2.time_from_start.sec = 3 # Move here in 2s

        goal_msg.trajectory.points = [point1, point2] # the journey

        # Sending goal to action server
        future = self._action_client.send_goal_async(goal_msg, self.feedback_callback)
        future.add_done_callback(self.goal_response_callback) # triggers callback for server response, once future completes

        return future


    # DO I NEED THIS ?
    # def calculate_inverse_kinematics(self, target_direction):
    #     """
    #     Simple inverse kinematics for pointing the end-effector in a target direction.
    #     target_direction: [x, y, z] unit vector in robot base frame
    #     """
    #     x, y, z = target_direction
        
    #     # Calculate base rotation (joint1) - yaw
    #     joint1 = math.atan2(y, x)
        
    #     # Calculate shoulder angle (joint2) to point toward target
    #     # For a pointing gesture, we want the arm extended horizontally
    #     horizontal_dist = math.sqrt(x*x + y*y)
    #     joint2 = math.atan2(-z, horizontal_dist)  # Negative z for upward pointing
        
    #     # Set elbow (joint3) to extend the arm - configurable pointing pose
    #     joint3 = self.elbow_angle
        
    #     # Set wrist (joint4) to keep end-effector level
    #     joint4 = -joint2 - joint3 + math.radians(self.end_effector_angle)
        
    #     return [joint1, joint2, joint3, joint4]


    # Feedback
    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback

        # print feedback parameters (current trajectory)
        if self.debug_mode:
            self.get_logger().info(
                f"\n\n---- FEEDBACK ----\n"
                f"Action in progress...\n"
                f"Trajectory: \n"
                f"  {feedback.actual}\n" # TODO: custom status display?? AND add rate limit ... make universal var?
                )


    # informs if server accepts the goal or not
    def goal_response_callback(self, future):
        goal_handle = future.result() # future.result() (stored in goal_handle) is the object returned after sending goal to server. Its fields reflect the fields in the action structure.
        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected by action server')
            return
        self.get_logger().info('Goal accpted by action server')
        self.current_goal_handle = goal_handle

        # get server response (wait async for response)
        self.get_result_future = self.current_goal_handle.get_result_async() 

        # process server response
        self.get_result_future.add_done_callback(self.get_result_callback)


    # Processes server reponse
    # Prints result to temrinal # TODO: make debugging mode??
    def get_result_callback(self, future):
        result = future.result().result # get result 

        # TODO: more feedback - error code, error msg, actual trajectory??
        self.get_logger().info(
            f"\n=== RESULT ===\n"
            f"Action Completed."
            # f"Trajectory: \n"
            # f"  {result.actual}\n" # TODO: custom status display??
            )

        self.goal_succeeded = True



# DEFAULT MAIN from webiste 
# if __name__ == '__main__':
#     rclpy.init(args=args)

#     action_client = FollowJointTrajectoryActionClient()

#     # TODO: adapt to your action's parameters
#     future = action_client.send_goal(a, b)

#     rclpy.spin_until_future_complete(action_client, future)

#     rclpy.shutdown()




# OWN main method
def main(args=None):
    rclpy.init(
        args=args,
        signal_handler_options=SignalHandlerOptions.NO, # works with spin_once in while loop
    )
    action_client = FollowJointTrajectoryActionClient()

    try:
        # while rclpy.ok:
        # # while not action_client.shutdown: # works the same as while clause above 
        #     rclpy.spin_once(action_client, timeout_sec=0.1) # does this control flow behave as expected?

        # TODO: adapt to your action's parameters
        joint_angles1 = [math.radians(30),0.0,0.0,0.0] # [rad] # (waypont 1)
        future = action_client.send_goal(joint_angles1)
        # rclpy.spin_until_future_complete(action_client, future) # spins until goal reponds (future), then shuts down. Error - doesnt allow any feedback as shuts node right away. try inseatd of rclpy.spin(action_client) - continuous

        rclpy.spin(action_client) # fixes feedback bug 

    except KeyboardInterrupt:
        print(
            f"{action_client.get_name()} received a shutdown request (Ctrl+C)."
        )
    finally:
        action_client.on_shutdown()
        while not action_client.shutdown:
            continue
        action_client.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()


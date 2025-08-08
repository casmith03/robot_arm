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
        self.formatted_current_positions_deg = [0.0] * len(self.arm_joints)
        self.joint_states_received = False

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

        print()
        self.get_logger().info("==================================================")
        self.get_logger().info("Follow Joint Trajectory Action Client initialized.")
        self.get_logger().info("==================================================")


    # Credit: Robotics Assignment 
    def on_shutdown(self, future):
        """Custom behaviour on shutdown."""
        self.get_logger().info("Stopping the robot...")

        # cancel goal and stop motion
        goal_handle = future.result()
        self.cancel_goal(goal_handle)

        self.shutdown = True
    
    
    # stops robot motion
    def cancel_goal(self, goal_handle):
        self.get_logger().info("Cancelling goal...")
        cancel_goal = goal_handle.cancel_goal_async() # async - doesn't pause execution
        cancel_response = goal_handle.get_result_async() 

        self.get_logger().info("Goal cancelled.")


    # Credit: Arnavs
    # prints initial joint states
    # continually updtaes current joint positions
    def joint_state_callback(self, msg: JointState):
        # continually updtaes current positions
        for i, name in enumerate(self.arm_joints):
            if name in msg.name:
                idx = msg.name.index(name)
                self.current_positions[i] = msg.position[idx] # [rad]
                self.formatted_current_positions_deg[i] = round(math.degrees(msg.position[idx]), 1) # [deg], 1 dp

        if not self.joint_states_received:
            self.joint_states_received = True
            self.get_logger().info("Initial joint states received.")
            # self.get_logger().info(f"Initial joint states (radians): {self.current_positions}")
            self.get_logger().info(f"Initial joint states (deg): {self.formatted_current_positions_deg}")


    # Feedback
    # from both action feedback and joint states
    # only prints while the goal is active
    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback

        # print feedback parameters (current trajectory)
        if self.debug_mode:
            self.get_logger().info(
                f"\n\n---- FEEDBACK ----\n"
                f"Action in progress...\n"
                f"Current joint states: {feedback.actual}\n" # TODO: custom status display?? AND add rate limit ... make universal var?
                , throttle_duration_sec=1.0
                )
        
        # print current joint states while goal is active (should be same as above feedback)
        self.get_logger().info(
            f"Current joint states (deg): {self.formatted_current_positions_deg}\n"
            , throttle_duration_sec=0.5
            ) # every x seconds # TODO: custom status display??


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



    # update params to 'journey' ?? or should i make the journey multiple goals, ie one goals = one waypoint on the journey??
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

        start = JointTrajectoryPoint()
        start.positions = [0.0, 0.0, 0.0, 0.0] # Start (4 joints)
        start.time_from_start.sec = 2 # 1s to get to start

        waypoint_1 = JointTrajectoryPoint()  
        # point2.positions = [, , , ] # joint angles in rad - TODO: weave in joint limits?? - ASK ARNAV 
        waypoint_1.positions = joint_angles
        waypoint_1.time_from_start.sec = start.time_from_start.sec + 2 # Move here in 2s

        end = JointTrajectoryPoint()
        end.positions = [0.0, 0.0, 0.0, 0.0] # Start (4 joints)
        end.time_from_start.sec = waypoint_1.time_from_start.sec + 2

        goal_msg.trajectory.points = [waypoint_1] # the journey - moves from start back to start

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


    # Informs if server accepts the goal or not
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
    # Prints result to temrinal # TODO: make debugging mojoint_angles1de??
    def get_result_callback(self, future):
        result = future.result().result # get result 
        self.goal_succeeded = True

        # # TODO: more feedback - error code, error msg, actual trajectory??
        self.get_logger().info(f"Action Completed.")
        # self.get_logger().info(f"Final joint states (radians): {self.current_positions}\n") 
        self.get_logger().info(f"Final joint states (deg): {self.formatted_current_positions_deg}\n") # TODO: custom status display??


    # ----- HELPER METHODS -------

    def print_format_goal(self, joint_angles):
        self.get_logger().info(f"Sending goal...")
        self.get_logger().info(f"--------- Goal: --------")
        n = 0
        for joint_name in self.arm_joints:
            self.get_logger().info(f"   {joint_name}: {math.degrees(joint_angles[n]):.1f} degrees")
            n += 1
        self.get_logger().info(f"------------------------")


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
        #     rclpy.spin_once(action_client, timeout_sec=0.1)

        # joint1 = # base
        # joint2 = # shoulder
        # joint3 = # elbow
        # joint4 = # wrist
        # gripper = 0


        # articulate each joint
        # [rad] # [+ clockwise, + down, + down, + down]
        joint_angles = [math.radians(-30),0.0,0.0,0.0] # (-) anticlockwise
        joint_angles = [0.0,math.radians(10),0.0,0.0] # (+) down
        joint_angles = [0.0,0.0,math.radians(20),0.0] # (+) down
        joint_angles = [0.0,0.0,0.0,math.radians(-20)] # (-) up

        # combine joint movements
        joint_angles = [math.radians(-20),math.radians(-20),math.radians(-20),math.radians(-20)] # (-) moves up and anticlockwise 
        # joint_angles = [math.radians(20),math.radians(20),math.radians(20),math.radians(20)] # (+) moves down and clockwise 

        # sending multiple goals ...
        # make square
        start = [0.0,0.0,0.0,0.0] # [rad] 
        waypoint_1 = [-math.radians(30),0.0,0.0,0.0] # [rad] 




        action_client.print_format_goal(joint_angles) # print goal 
        future = action_client.send_goal(joint_angles)
        # rclpy.spin_until_future_complete(action_client, future) # spins until goal reponds (future), then shuts down. Error - doesnt allow any feedback as shuts node right away. try inseatd of rclpy.spin(action_client) - continuous
        rclpy.spin(action_client) # fixes feedback bug - doesn't stop node prematurely like above 'spin_until_future_complete'

    except KeyboardInterrupt:
        print(
            f"{action_client.get_name()} received a shutdown request (Ctrl+C)."
        )
    finally:
        action_client.on_shutdown(future)
        while not action_client.shutdown:
            continue
        action_client.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()


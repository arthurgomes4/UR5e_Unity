#!/usr/bin/env python3

from sensor_msgs.msg import JointState
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
import rclpy

class CommandNode(Node):
    def __init__(self, command=[0,0,0,0,0,0]):
        super().__init__('command_node')
        self.publisher = self.create_publisher(JointState, 'joint_command', 1)

        self.command = command
        self.timer = self.create_timer(0.1, self.timer_callback)

    def timer_callback(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
        msg.position = [ float(x) for x in self.command ]
        self.publisher.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg)
        self.timer.cancel()

class CommandClient(Node):

    def __init__(self, command = [0,0,0,0,0,0]):

        super().__init__('command_client')
        self.command = command
        self._action_client = ActionClient(self, FollowJointTrajectory, '/joint_trajectory_controller/follow_joint_trajectory')

    def send_goal(self):

        goal_msg = FollowJointTrajectory.Goal()

        goal_msg.trajectory.joint_names = [
            'shoulder_pan_joint', 
            'shoulder_lift_joint', 
            'elbow_joint', 
            'wrist_1_joint', 
            'wrist_2_joint', 
            'wrist_3_joint'
            ]

        # trajectory to execute passed command
        trajectory_point = JointTrajectoryPoint()
        trajectory_point.positions = [float(x) for x in self.command]
        trajectory_point.velocities = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        trajectory_point.time_from_start = rclpy.time.Duration(seconds=2).to_msg()

        goal_msg.trajectory.points.append(trajectory_point)

        self.get_logger().info('waiting for server...')
        self._action_client.wait_for_server()
        self.get_logger().info('sending goal...')

        # send goal future
        return self._action_client.send_goal_async(goal_msg)

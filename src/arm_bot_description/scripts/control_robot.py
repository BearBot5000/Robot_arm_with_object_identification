#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

class RobotController(Node):

    def __init__(self):
        super().__init__('robot_controller')

        # Publisher for joint trajectory
        self.joint_trajectory_pub = self.create_publisher(JointTrajectory, '/arm_controller/joint_trajectory', 10)

        # Timer to periodically publish joint commands
        self.timer = self.create_timer(1.0, self.publish_joint_trajectory)

    def publish_joint_trajectory(self):
        # Create a JointTrajectory message
        joint_trajectory = JointTrajectory()
        joint_trajectory.joint_names = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']

        # Create a JointTrajectoryPoint message
        point = JointTrajectoryPoint()
        point.positions = [0.5, 0.5, 0.5, 0.5, 0.5, 0.5]
        point.time_from_start.sec = 1

        # Append the point to the trajectory
        joint_trajectory.points.append(point)

        # Publish the joint trajectory
        self.joint_trajectory_pub.publish(joint_trajectory)
        self.get_logger().info('Published joint trajectory')

def main(args=None):
    rclpy.init(args=args)
    robot_controller = RobotController()
    rclpy.spin(robot_controller)
    robot_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

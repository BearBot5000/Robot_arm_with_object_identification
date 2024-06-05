import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

class JointCommandPublisher(Node):
    def __init__(self):
        super().__init__('joint_command_publisher')
        self.publisher_ = self.create_publisher(JointTrajectory, '/forward_position_controller/joint_trajectory', 10)
        timer_period = 1.0  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        msg = JointTrajectory()
        msg.joint_names = ['Revolute_9', 'Revolute_10', 'Revolute_11', 'Revolute_12', 'Revolute_13', 'Slider_14', 'Slider_15']

        point = JointTrajectoryPoint()
        point.positions = [0.5, 0.5, 0.5, 0.5, 0.5, 0.02, 0.02]
        point.time_from_start.sec = 2

        msg.points.append(point)
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: {msg}')

def main(args=None):
    rclpy.init(args=args)
    joint_command_publisher = JointCommandPublisher()
    rclpy.spin(joint_command_publisher)
    joint_command_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

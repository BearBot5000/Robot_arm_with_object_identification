import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
import time

class RobotArmController(Node):
    def __init__(self):
        super().__init__('robot_arm_controller')
        
        # Create publishers for each joint controller
        self.revolute_9_pub = self.create_publisher(Float64, '/Revolute_9_position_controller/command', 10)
        self.revolute_10_pub = self.create_publisher(Float64, '/Revolute_10_position_controller/command', 10)
        self.revolute_11_pub = self.create_publisher(Float64, '/Revolute_11_position_controller/command', 10)
        self.revolute_12_pub = self.create_publisher(Float64, '/Revolute_12_position_controller/command', 10)
        self.revolute_13_pub = self.create_publisher(Float64, '/Revolute_13_position_controller/command', 10)
        self.slider_14_pub = self.create_publisher(Float64, '/Slider_14_position_controller/command', 10)
        self.slider_15_pub = self.create_publisher(Float64, '/Slider_15_position_controller/command', 10)

    def move_joint(self, publisher, position):
        msg = Float64()
        msg.data = position
        self.get_logger().info(f'Moving joint to position: {position}')
        publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    controller = RobotArmController()

    try:
        # Lift the arm up
        controller.move_joint(controller.revolute_9_pub, 1.0)  # Move joint 9 to position 1.0
        controller.move_joint(controller.revolute_10_pub, 0.5)  # Move joint 10 to position 0.5
        controller.move_joint(controller.revolute_11_pub, -0.5)  # Move joint 11 to position -0.5
        controller.move_joint(controller.revolute_12_pub, 0.0)  # Move joint 12 to position 0.0
        controller.move_joint(controller.revolute_13_pub, -1.0)  # Move joint 13 to position -1.0
        controller.move_joint(controller.slider_14_pub, 0.1)  # Move slider 14 to position 0.1
        controller.move_joint(controller.slider_15_pub, -0.1)  # Move slider 15 to position -0.1

        # Allow time for the arm to move
        time.sleep(5)

        # Put the arm back down
        controller.move_joint(controller.revolute_9_pub, 0.0)  # Move joint 9 to position 0.0
        controller.move_joint(controller.revolute_10_pub, 0.0)  # Move joint 10 to position 0.0
        controller.move_joint(controller.revolute_11_pub, 0.0)  # Move joint 11 to position 0.0
        controller.move_joint(controller.revolute_12_pub, 0.0)  # Move joint 12 to position 0.0
        controller.move_joint(controller.revolute_13_pub, 0.0)  # Move joint 13 to position 0.0
        controller.move_joint(controller.slider_14_pub, 0.0)  # Move slider 14 to position 0.0
        controller.move_joint(controller.slider_15_pub, 0.0)  # Move slider 15 to position 0.0

        # Allow time for the arm to move
        time.sleep(5)

    except KeyboardInterrupt:
        pass

    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

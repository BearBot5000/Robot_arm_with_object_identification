import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import sys
#from moveit_commander import MoveGroupCommander, PlanningSceneInterface, RobotCommander, roscpp_initialize, roscpp_shutdown
#from geometry_msgs.msg import Pose

class UserInteractionNode(Node):
    def __init__(self):
        super().__init__('user_interaction_node')

        # Subscriber for detected objects
        self.subscription = self.create_subscription(
            String,
            'detected_objects',
            self.detected_objects_callback,
            10)
        self.subscription

        self.detected_objects = []
        self.pickup_requested = False

    def prompt_user_input(self):
        self.target_color = input("Enter the color of the object to pick up: ").strip().lower()
        self.target_shape = input("Enter the shape of the object to pick up: ").strip().lower()
        self.pickup_requested = True

    def detected_objects_callback(self, msg):
        self.detected_objects = json.loads(msg.data)
        #print(f"Received JSON: {self.detected_objects}")  # Print the received JSON data
        if self.pickup_requested:
            self.check_for_object()

    def check_for_object(self):
        if self.target_color and self.target_shape:
            found = False
            for color, shape, world_x, world_y in self.detected_objects:  # Corrected unpacking order
                print(f"Checking object: {color} {shape} at coordinates ({world_x:.2f}, {world_y:.2f})")  # Debugging
                if color == self.target_color and shape == self.target_shape:
                    print(f"Object found: {color} {shape} at coordinates ({world_x:.2f}, {world_y:.2f})")
                    self.send_to_moveit(world_x, world_y)
                    found = True
                    break
            if not found:
                print("Specified object does not exist.")
                self.prompt_for_next_action()
                
    #send the x-y coordinates to moveit config
    def send_to_moveit(self, x, y):
        print(f"Sending coordinates to MoveIt: ({x:.2f}, {y:.2f})")
        # Simulate MoveIt function call
        print(f"Simulating robot arm movement to ({x:.2f}, {y:.2f}) and picking up the object.")


        #actual MoveIt code to move the robot arm
        # z = 0.2
        # pose_target = self.move_group.get_current_pose().pose
        # pose_target.position.x = x
        # pose_target.position.y = y
        # pose_target.position.z = 0.2  # Assuming a fixed height for the pick operation
        # self.move_group.set_pose_target(pose_target)
        # plan = self.move_group.go(wait=True)
        # self.move_group.stop()
        # self.move_group.clear_pose_targets()
        
        # if plan:
        #     print("MoveIt plan executed successfully")
        # else:
        #     print("Failed to execute MoveIt plan")


        self.pickup_requested = False
        self.prompt_for_next_action()

    #see if user wants to pick up another object else quit script
    def prompt_for_next_action(self):
        response = input("Do you want to pick up another object? (yes/no): ").strip().lower()
        if response == 'yes':
            self.prompt_user_input()
        else:
            print("Exiting...")
            sys.exit()

def main(args=None):
    rclpy.init(args=args)
    user_interaction_node = UserInteractionNode()
    user_interaction_node.prompt_user_input()
    rclpy.spin(user_interaction_node)
    user_interaction_node.destroy_node()
    rclpy.shutdown()
    roscpp_shutdown()

if __name__ == '__main__':
    main()

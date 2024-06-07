import rclpy
from rclpy.node import Node
from moveit_commander import MoveGroupCommander
from moveit_commander import PlanningSceneInterface
from moveit_commander import RobotCommander
from moveit_commander import roscpp_initialize, roscpp_shutdown
from geometry_msgs.msg import Pose
import sys

class MoveItInterface(Node):
    def __init__(self):
        super().__init__('moveit_interface')
        roscpp_initialize(sys.argv)
        self.robot = RobotCommander()
        self.scene = PlanningSceneInterface()
        self.group_name = "manipulator"
        self.move_group = MoveGroupCommander(self.group_name)
        self.move_group.set_planning_time(10.0)
        self.move_group.set_num_planning_attempts(10)

    def move_to(self, x, y, z):
        pose_goal = Pose()
        pose_goal.orientation.w = 1.0
        pose_goal.position.x = x
        pose_goal.position.y = y
        pose_goal.position.z = z
        self.move_group.set_pose_target(pose_goal)

        plan = self.move_group.go(wait=True)
        self.move_group.stop()
        self.move_group.clear_pose_targets()
        return plan

def main(args=None):
    rclpy.init(args=args)
    moveit_interface = MoveItInterface()

    # Example usage:
    x = float(input("Enter x coordinate: "))
    y = float(input("Enter y coordinate: "))
    z = float(input("Enter z coordinate: "))

    success = moveit_interface.move_to(x, y, z)
    if success:
        print("MoveIt plan executed successfully")
    else:
        print("Failed to execute MoveIt plan")

    moveit_interface.destroy_node()
    rclpy.shutdown()
    roscpp_shutdown()

if __name__ == '__main__':
    main()

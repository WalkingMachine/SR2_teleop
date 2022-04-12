import rclpy
from rclpy.node import Node
from .xbox_controller.controller_reader import XBoxControllerReader
from geometry_msgs.msg import Twist
from std_msgs.msg import Empty

class SR2_Teleop(Node):

    def __init__(self):
        super().__init__('sr2_teleop')

        self.controller = XBoxControllerReader(self.controller_callback)
        
        # Publisher for the wheels
        self.wheel_pub = self.create_publisher(Twist, "sara/cmd_move", 10)

        # TODO: publisher for the arm (kinova, dynamixel)

        # TODO: publisher for the gripper

        # heartbeat
        self.heartbeat_pub = self.create_publisher(Empty, "sara/teleop/heartbeat", 10)
        self.heartbeat_timer = self.create_timer(1, self.heartbeat_callback)


    def controller_callback(self, ctrl_vals):
        
        # Wheel control
        wheel_msg = Twist()
        if ctrl_vals["L_BUMPER"] == 1 and ctrl_vals["R_BUMPER"] == 1:
            wheel_msg.linear.x = ctrl_vals["JOY_RX"]
            wheel_msg.linear.y = ctrl_vals["JOY_RY"]*-1  # This is inversed for easier calculations
            wheel_msg.angular.z = ctrl_vals["JOY_LX"]
        else:
            wheel_msg.linear.x = 0.0
            wheel_msg.linear.y = 0.0
            wheel_msg.angular.z = 0.0

        self.wheel_pub.publish(wheel_msg)

        # TODO: Arm messages
        # TODO: Gripper Messages

    def heartbeat_callback(self):
        if self.controller.is_alive() and self.controller.is_running:
            msg = Empty()
            self.heartbeat_pub.publish(msg)
        else:
            self.controller.restart()

def main(args=None):
    rclpy.init(args=args)

    node = SR2_Teleop()

    node.controller.start()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()


if __name__ == "__main__":
    main()

import rclpy
from rclpy.node import Node

from sr2_interfaces.msg import Teleop, ArmControl
from geometry_msgs.msg import Twist

class TeleopControl(Node):
    
    def __init__(self):
        super().__init__('teleop_control')

        # Subscriber to Teleop
        self._teleop_sub = self.create_subscription(Teleop, 'sr2/teleop/source', self.teleop_callback, 10)

        # Publication to Arm Control
        self._arm_pub = self.create_publisher(ArmControl, 'sr2/arm/control', 10)

        # Publication to wheels
        self._wheel_pub = self.create_publisher(Twist, 'sr2/wheels/cmd_vel', 10)

        self._start_pressed = False
        self._back_pressed = False
        self._kinova_moving = False
        self._a_pressed = False
        self._gripper_state = False
    
    def teleop_callback(self, msg: Teleop):
        pub_msg = ArmControl()
        wheel_msg = Twist()

        if msg.start == 1 and not self._start_pressed:
            self._start_pressed = True
            pub_msg.kinova_joint = True
            pub_msg.change = ArmControl.JOINT_UP
        elif msg.back == 1 and not self._back_pressed:
            self._back_pressed = True
            pub_msg.kinova_joint = True
            pub_msg.change = ArmControl.JOINT_DOWN

        if msg.start == 0 and self._start_pressed:
            self._start_pressed = False

        if msg.back == 0 and self._back_pressed:
            self._back_pressed = False
        
        if msg.l_trigger > 0.0:
            pub_msg.kinova_move = True
            pub_msg.direction = ArmControl.CW
            pub_msg.speed = msg.l_trigger
        elif msg.r_trigger > 0.0:
            pub_msg.kinova_move = True
            pub_msg.direction = ArmControl.CCW
            pub_msg.speed = msg.r_trigger

        if msg.a == 1 and not self._a_pressed:
            self._a_pressed = True
            pub_msg.gripper = True
            pub_msg.state = ArmControl.GRIPPER_CLOSE if self._gripper_state else ArmControl.GRIPPER_OPEN
            self._gripper_state = not self._gripper_state

        if msg.a == 0 and self._a_pressed:
            self._a_pressed = False
        
        if msg.l_bumper == 1 and msg.r_bumper == 1:
            wheel_msg.linear.x = float(msg.joy_rx)
            wheel_msg.linear.y = -1*float(msg.joy_ry)
            wheel_msg.angular.z = float(msg.joy_lx)
        else:
            wheel_msg.linear.x = 0.0
            wheel_msg.linear.y = 0.0
            wheel_msg.angular.z = 0.0


        self._arm_pub.publish(pub_msg)
        self._wheel_pub.publish(wheel_msg)


def main(args=None):
    rclpy.init(args=args)

    node = TeleopControl()

    rclpy.spin(node)


import rclpy
from rclpy.node import Node
from sr2_teleop.xbox_controller.controller_reader import XBoxControllerReader
from geometry_msgs.msg import Twist
from sr2_interfaces.msg import Heartbeat, Teleop

class SR2_Teleop(Node):

    def __init__(self):
        super().__init__('sr2_teleop')

        self.controller = XBoxControllerReader(self.controller_callback)
        
        # Publish to the teleop controller
        self._pub = self.create_publisher(Teleop, 'sr2/teleop/source', 10)

        # heartbeat
        self.heartbeat_pub = self.create_publisher(Heartbeat, "sr2/teleop/heartbeat", 10)
        self.heartbeat_timer = self.create_timer(0.5, self.heartbeat_callback)
        self._heartbeat_error_counter = 0

        # auto update
        self.update_timer = self.create_timer(1, self.update_timer_callback)

    def controller_callback(self, ctrl_vals):
        msg = Teleop()
        msg.joy_lx = float(ctrl_vals["JOY_LX"])
        msg.joy_ly = float(ctrl_vals["JOY_LY"])
        msg.joy_rx = float(ctrl_vals["JOY_RX"])
        msg.joy_ry = float(ctrl_vals["JOY_RY"])
        msg.l_trigger = float(ctrl_vals["L_TRIGGER"])
        msg.r_trigger = float(ctrl_vals["R_TRIGGER"])
        msg.dpad_x = ctrl_vals["DPAD_X"]
        msg.dpad_y = ctrl_vals["DPAD_Y"]
        msg.x = ctrl_vals["X"]
        msg.y = ctrl_vals["Y"]
        msg.a = ctrl_vals["A"]
        msg.b = ctrl_vals["B"]
        msg.l_bumper = ctrl_vals["L_BUMPER"]
        msg.r_bumper = ctrl_vals["R_BUMPER"]
        msg.l_thumb = ctrl_vals["L_THUMB"]
        msg.r_thumb = ctrl_vals["R_THUMB"]
        msg.back = ctrl_vals["BACK"]
        msg.start = ctrl_vals["START"]
        msg.xbox = ctrl_vals["XBOX"]
        self._pub.publish(msg)
        self.update_timer.reset()

    def heartbeat_callback(self):
        if self.controller.is_alive() and self.controller.is_running:
            msg = Heartbeat()
            self.heartbeat_pub.publish(msg)
            self.get_logger().debug("Heartbeat")
        else:
            if self._heartbeat_error_counter % 10:
                self.get_logger().error("Connection to controller lost, attempting to reconnect")
                try:
                    self.controller = XBoxControllerReader(self.controller_callback)
                except IndexError as e:
                    self._heartbeat_error_counter = 0
            else:
                self._heartbeat_error_counter += 1

    def update_timer_callback(self):
        ctrl_vals = self.controller.read()
        msg = Teleop()
        msg.joy_lx = float(ctrl_vals["JOY_LX"])
        msg.joy_ly = float(ctrl_vals["JOY_LY"])
        msg.joy_rx = float(ctrl_vals["JOY_RX"])
        msg.joy_ry = float(ctrl_vals["JOY_RY"])
        msg.l_trigger = float(ctrl_vals["L_TRIGGER"])
        msg.r_trigger = float(ctrl_vals["R_TRIGGER"])
        msg.dpad_x = ctrl_vals["DPAD_X"]
        msg.dpad_y = ctrl_vals["DPAD_Y"]
        msg.x = ctrl_vals["X"]
        msg.y = ctrl_vals["Y"]
        msg.a = ctrl_vals["A"]
        msg.b = ctrl_vals["B"]
        msg.l_bumper = ctrl_vals["L_BUMPER"]
        msg.r_bumper = ctrl_vals["R_BUMPER"]
        msg.l_thumb = ctrl_vals["L_THUMB"]
        msg.r_thumb = ctrl_vals["R_THUMB"]
        msg.back = ctrl_vals["BACK"]
        msg.start = ctrl_vals["START"]
        msg.xbox = ctrl_vals["XBOX"]
        self._pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)

    node = SR2_Teleop()

    rclpy.spin(node)
    node.controller.stop()

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

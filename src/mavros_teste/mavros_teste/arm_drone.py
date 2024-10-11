import rclpy
from rclpy.node import Node
from mavros_msgs.srv import CommandBool
from mavros_msgs.srv import SetMode
from mavros_msgs.msg import State

class ArmDroneNode(Node):
    def __init__(self):
        super().__init__('arm_drone_node')
        self.state_sub = self.create_subscription(
            State,
            '/mavros/state',
            self.state_callback,
            10
        )
        self.arming_client = self.create_client(CommandBool, '/mavros/cmd/arming')
        self.set_mode_client = self.create_client(SetMode, '/mavros/set_mode')

        self.current_state = State()

        self.get_logger().info("Waiting for arming and mode services...")
        self.arming_client.wait_for_service()
        self.set_mode_client.wait_for_service()

        self.get_logger().info("Services are available.")

    def state_callback(self, msg):
        self.current_state = msg

    def arm_drone(self):
        # Set to OFFBOARD mode
        offboard_mode = SetMode.Request()
        offboard_mode.custom_mode = 'OFFBOARD'

        self.get_logger().info("Setting mode to OFFBOARD...")
        mode_response = self.set_mode_client.call(offboard_mode)

        if mode_response.mode_sent:
            self.get_logger().info("OFFBOARD mode set successfully.")
        else:
            self.get_logger().error("Failed to set OFFBOARD mode.")
            return

        # Arm the drone
        arm_cmd = CommandBool.Request()
        arm_cmd.value = True

        self.get_logger().info("Arming the drone...")
        arm_response = self.arming_client.call(arm_cmd)

        if arm_response.success:
            self.get_logger().info("Drone armed successfully.")
        else:
            self.get_logger().error("Failed to arm the drone.")

def main(args=None):
    rclpy.init(args=args)
    node = ArmDroneNode()

    # Wait for the drone to be connected and ready
    while rclpy.ok() and not node.current_state.connected:
        rclpy.spin_once(node)

    node.get_logger().info("Drone connected.")

    # Arm the drone
    node.arm_drone()

    # Keep the node alive for monitoring
    rclpy.spin(node)

    rclpy.shutdown()

if __name__ == '__main__':
    main()

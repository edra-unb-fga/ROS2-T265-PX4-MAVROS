import math
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion, Point, Vector3  # Import Point aqui
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time

class VisualInertialOdometryPublisher(Node):
    def __init__(self):
        super().__init__('VIO_pub')
        self.odometry_publisher_ = self.create_publisher(Odometry, '/mavros/odometry/in', 10)
        self.odometry_subscription_ = self.create_subscription(
            Odometry,
            '/t265/pose/sample',
            self.odometry_callback,
            qos_profile=qos_profile_sensor_data
        )
        self.timer = self.create_timer(10, self.timer_callback)  # Timer callback a cada 10 segundos
        
    def odometry_callback(self, msg):
        odometry_msg = Odometry()
        odometry_msg.header.stamp = msg.header.stamp
        odometry_msg.header.frame_id = "odom"

        # Copiar dados de posição
        position = msg.pose.pose.position
        rotated_position = Point()  # Use Point aqui
        rotated_position.x = position.x
        rotated_position.y = -position.y
        rotated_position.z = -position.z
        odometry_msg.pose.pose.position = rotated_position

        # Copiar dados de velocidade linear
        velocity = msg.twist.twist.linear
        rotated_velocity = Vector3()
        rotated_velocity.x = velocity.x
        rotated_velocity.y = -velocity.y
        rotated_velocity.z = -velocity.z
        odometry_msg.twist.twist.linear = rotated_velocity

        # Copiar dados de velocidade angular
        angular_velocity = msg.twist.twist.angular
        rotated_angular_velocity = Vector3()
        rotated_angular_velocity.x = angular_velocity.x
        rotated_angular_velocity.y = -angular_velocity.y
        rotated_angular_velocity.z = -angular_velocity.z
        odometry_msg.twist.twist.angular = rotated_angular_velocity

        # Copiar e rotacionar a orientação (quaternion)
        orientation = msg.pose.pose.orientation
        euler_angles = self.quaternion_to_euler(orientation)
        euler_angles[0] += math.pi  # Rotaciona 180 graus no eixo X
        rotated_quaternion = self.euler_to_quaternion(euler_angles)
        odometry_msg.pose.pose.orientation.x = rotated_quaternion[0]
        odometry_msg.pose.pose.orientation.y = rotated_quaternion[1]
        odometry_msg.pose.pose.orientation.z = rotated_quaternion[2]
        odometry_msg.pose.pose.orientation.w = rotated_quaternion[3]

        # Publicar a odometria modificada no tópico do MAVROS
        self.odometry_publisher_.publish(odometry_msg)

    def timer_callback(self):
        self.get_logger().info('Publicando odometria visual-inercial')

    def quaternion_to_euler(self, quaternion):
        # Converter quaternion para ângulos de Euler (roll, pitch, yaw)
        x = quaternion.x
        y = quaternion.y
        z = quaternion.z
        w = quaternion.w

        roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x**2 + y**2))
        pitch = math.asin(2 * (w * y - z * x))
        yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y**2 + z**2))

        return [roll, pitch, yaw]

    def euler_to_quaternion(self, euler_angles):
        # Converter ângulos de Euler para quaternion
        roll = euler_angles[0]
        pitch = euler_angles[1]
        yaw = euler_angles[2]

        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)

        w = cy * cp * cr + sy * sp * sr
        x = cy * cp * sr - sy * sp * cr
        y = sy * cp * sr + cy * sp * cr
        z = sy * cp * cr - cy * sp * sr

        return [x, y, z, w]

def main(args=None):
    rclpy.init(args=args)
    publisher = VisualInertialOdometryPublisher()
    rclpy.spin(publisher)
    publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

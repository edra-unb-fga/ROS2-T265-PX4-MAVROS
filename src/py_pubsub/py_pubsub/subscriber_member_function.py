import math
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion, Vector3
from rclpy.time import Time

class VisualInertialOdometryPublisher(Node):
    def __init__(self):
        super().__init__('VIO_pub')
        
        # Publica a odometria visual-inercial no tópico do MAVROS
        self.odometry_publisher_ = self.create_publisher(Odometry, '/mavros/odometry/out', 10)
        
        # Assina o tópico de odometria visual da câmera
        self.odometry_subscription_ = self.create_subscription(
            Odometry,
            '/camera/pose/sample',
            self.odometry_callback,
            10
        )

        self.timer = self.create_timer(10, self.timer_callback)  # Timer callback every 10 seconds
        
    def odometry_callback(self, msg):
        # Cria a mensagem de odometria MAVROS
        odom_msg = Odometry()
        odom_msg.header.stamp = msg.header.stamp
        odom_msg.header.frame_id = "odom"

        # Processa a posição e a rotação (ajustando para a convenção de MAVROS se necessário)
        position = msg.pose.pose.position
        rotated_position = Vector3()
        rotated_position.x = position.x
        rotated_position.y = -position.y
        rotated_position.z = -position.z

        odom_msg.pose.pose.position.x = rotated_position.x
        odom_msg.pose.pose.position.y = rotated_position.y
        odom_msg.pose.pose.position.z = rotated_position.z

        # Processa a orientação (ajuste de rotação de 180 graus)
        orientation = msg.pose.pose.orientation
        euler_angles = self.quaternion_to_euler(orientation)
        euler_angles[0] += math.pi  # Rotaciona 180 graus ao redor do eixo X
        rotated_quaternion = self.euler_to_quaternion(euler_angles)

        odom_msg.pose.pose.orientation.x = rotated_quaternion[0]
        odom_msg.pose.pose.orientation.y = rotated_quaternion[1]
        odom_msg.pose.pose.orientation.z = rotated_quaternion[2]
        odom_msg.pose.pose.orientation.w = rotated_quaternion[3]

        # Processa as velocidades lineares
        velocity = msg.twist.twist.linear
        rotated_velocity = Vector3()
        rotated_velocity.x = velocity.x
        rotated_velocity.y = -velocity.y
        rotated_velocity.z = -velocity.z

        odom_msg.twist.twist.linear.x = rotated_velocity.x
        odom_msg.twist.twist.linear.y = rotated_velocity.y
        odom_msg.twist.twist.linear.z = rotated_velocity.z

        # Processa as velocidades angulares
        angular_velocity = msg.twist.twist.angular
        rotated_angular_velocity = Vector3()
        rotated_angular_velocity.x = angular_velocity.x
        rotated_angular_velocity.y = -angular_velocity.y
        rotated_angular_velocity.z = -angular_velocity.z

        odom_msg.twist.twist.angular.x = rotated_angular_velocity.x
        odom_msg.twist.twist.angular.y = rotated_angular_velocity.y
        odom_msg.twist.twist.angular.z = rotated_angular_velocity.z

        # Publica a odometria no tópico MAVROS
        self.odometry_publisher_.publish(odom_msg)

    def timer_callback(self):
        self.get_logger().info('Publicando odometria visual-inercial no MAVROS')

    def quaternion_to_euler(self, quaternion):
        # Converte quaternion para ângulos de Euler (roll, pitch, yaw)
        x = quaternion.x
        y = quaternion.y
        z = quaternion.z
        w = quaternion.w

        roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x**2 + y**2))
        pitch = math.asin(2 * (w * y - z * x))
        yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y**2 + z**2))

        return [roll, pitch, yaw]

    def euler_to_quaternion(self, euler_angles):
        # Converte ângulos de Euler (roll, pitch, yaw) para quaternion
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

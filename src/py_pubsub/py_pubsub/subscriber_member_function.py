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
        self.odometry_publisher_ = self.create_publisher(Odometry, '/mavros/odometry/out', 10)
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
        odometry_msg.twist.twist.angular = rotated_angular_veloci

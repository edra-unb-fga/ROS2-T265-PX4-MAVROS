import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Quaternion, Vector3
from nav_msgs.msg import Odometry
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped

class VisualInertialPosePublisher(Node):
    def __init__(self):
        super().__init__('VIO_pose_pub')
        # Publicar no tópico de visão do MAVROS (PoseStamped)
        self.pose_publisher_ = self.create_publisher(PoseStamped, '/mavros/vision_pose/pose', 10)
        
        # Assinar o tópico da odometria da T265
        self.odometry_subscription_ = self.create_subscription(
            Odometry,
            '/t265/pose/sample',
            self.odometry_callback,
            qos_profile=qos_profile_sensor_data
        )

        # Criar um TransformBroadcaster para publicar as transformações
        self.br = TransformBroadcaster(self)

        # Timer callback para checar periodicamente
        self.timer = self.create_timer(10, self.timer_callback)

    def odometry_callback(self, msg):
        # Criar a mensagem de PoseStamped para o MAVROS
        pose_msg = PoseStamped()
        pose_msg.header.stamp = msg.header.stamp
        pose_msg.header.frame_id = "odom"  # Definindo o frame_id como "odom"

        # Posição
        position = msg.pose.pose.position
        rotated_position = Vector3()
        rotated_position.x = position.x
        rotated_position.y = -position.y
        rotated_position.z = -position.z

        pose_msg.pose.position.x = rotated_position.x
        pose_msg.pose.position.y = rotated_position.y
        pose_msg.pose.position.z = rotated_position.z

        # Orientação (Rotacionar 180 graus no eixo X)
        orientation = msg.pose.pose.orientation
        euler_angles = self.quaternion_to_euler(orientation)
        euler_angles[0] += math.pi
        rotated_quaternion = self.euler_to_quaternion(euler_angles)

        pose_msg.pose.orientation.x = rotated_quaternion[0]
        pose_msg.pose.orientation.y = rotated_quaternion[1]
        pose_msg.pose.orientation.z = rotated_quaternion[2]
        pose_msg.pose.orientation.w = rotated_quaternion[3]

        # Publicar a pose no tópico /mavros/vision_pose/pose
        self.pose_publisher_.publish(pose_msg)

    def timer_callback(self):
        self.get_logger().info('Publicando pose visual-inercial e transformação estática.')

        # Publicar transformação estática entre "odom_frame_ned" e "odom"
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "odom_frame_ned"
        t.child_frame_id = "odom"

        # Transformação identidade (pode ajustar conforme necessidade)
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0

        # Publicar a transformação
        self.br.sendTransform(t)

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
        # Converter ângulos de Euler (roll, pitch, yaw) para quaternion
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
    publisher = VisualInertialPosePublisher()
    rclpy.spin(publisher)
    publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Point, Quaternion
from nav_msgs.msg import Odometry
from mavros_msgs.msg import CompanionProcessStatus
from rclpy.qos import qos_profile_sensor_data

class VisualInertialPosePublisher(Node):
    def __init__(self):
        super().__init__('VIO_pose_pub')
        # Publicar no tópico correto do MAVROS usando PoseStamped
        self.pose_publisher_ = self.create_publisher(PoseStamped, '/mavros/vision_pose/pose', 10)
        self.odometry_subscription_ = self.create_subscription(
            Odometry,
            '/t265/pose/sample',  # Tópico da T265
            self.odometry_callback,
            qos_profile=qos_profile_sensor_data
        )
        
        # Publicação adicional para odometria e status do processo
        self.odometry_sub = self.create_subscription(
            Odometry, 
            '/camera/odom/sample', 
            self.odometry_callback, 
            qos_profile=qos_profile_sensor_data
        )
        self.odometry_pub = self.create_publisher(Odometry, '/mavros/odometry/out', 5)
        self.companion_computer_pub = self.create_publisher(CompanionProcessStatus, '/mavros/companion_process/status', 1)

        self.odom_output = Odometry()
        self.mav_comp_id_msg = CompanionProcessStatus()

        self.timer = self.create_timer(0.1, self.timer_callback)  # Timer callback a cada 0.1 segundo (10 Hz)
        
    def odometry_callback(self, msg):
        pose_msg = PoseStamped()
        pose_msg.header.stamp = msg.header.stamp
        pose_msg.header.frame_id = "odom"

        # Copiar dados de posição
        position = msg.pose.pose.position
        rotated_position = Point()
        rotated_position.x = position.x
        rotated_position.y = -position.y
        rotated_position.z = -position.z
        pose_msg.pose.position = rotated_position

        # Copiar e rotacionar a orientação (quaternion)
        orientation = msg.pose.pose.orientation
        euler_angles = self.quaternion_to_euler(orientation)
        euler_angles[0] += math.pi  # Rotaciona 180 graus no eixo X (verifique se essa rotação é realmente necessária)
        rotated_quaternion = self.euler_to_quaternion(euler_angles)
        pose_msg.pose.orientation.x = rotated_quaternion[0]
        pose_msg.pose.orientation.y = rotated_quaternion[1]
        pose_msg.pose.orientation.z = rotated_quaternion[2]
        pose_msg.pose.orientation.w = rotated_quaternion[3]

        # Publicar a pose modificada no tópico do MAVROS
        self.pose_publisher_.publish(pose_msg)

        # Publicar dados de odometria no tópico mavros/odometry/out
        self.odom_output = msg  # Ajustar ou modificar a mensagem conforme necessário
        self.odometry_pub.publish(self.odom_output)

        # Publicar status do processo de companion
        self.mav_comp_id_msg.component = 1  # ID do componente MAVROS
        self.mav_comp_id_msg.state = CompanionProcessStatus.MAV_STATE_ACTIVE
        self.companion_computer_pub.publish(self.mav_comp_id_msg)

    def timer_callback(self):
        self.get_logger().info('Publicando pose visual-inercial e odometria')

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
    publisher = VisualInertialPosePublisher()
    rclpy.spin(publisher)
    publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

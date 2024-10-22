#!/usr/bin/env python3

import rclpy
from rclpy import qos
from rclpy.node import Node
from rclpy.clock import Clock
from nav_msgs.msg import Odometry
from mavros_msgs.msg import CompanionProcessStatus
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped

class RealsenseBridge(Node):

    def __init__(self):
        super().__init__('realsense_bridge')
        # Subscrição para o tópico de odometria da câmera
        self.odometry_sub = self.create_subscription(Odometry, '/camera/odom/sample', self.odometry_callback, qos.qos_profile_sensor_data)
        
        # Publicação de odometria e status do MAVROS
        self.odometry_pub = self.create_publisher(Odometry, '/mavros/odometry/out', 5)
        self.companion_computer_pub = self.create_publisher(CompanionProcessStatus, '/mavros/companion_process/status', 1)
        
        # Inicializar mensagens de saída
        self.odom_output = Odometry()
        self.mav_comp_id_msg = CompanionProcessStatus()

        # Inicializar um transform broadcaster para frames ausentes
        self.br = TransformBroadcaster(self)

        # Timer para enviar transformações de frames fixos
        self.timer = self.create_timer(0.1, self.broadcast_transform)

    def odometry_callback(self, data):
        # Copiar dados da mensagem de odometria
        self.odom_output = data
        self.odom_output.header.frame_id = data.header.frame_id
        self.odom_output.child_frame_id = data.child_frame_id
        self.odometry_pub.publish(self.odom_output)

        # Log para indicar que a odometria está sendo publicada
        self.get_logger().info('Odometria publicada no tópico /mavros/odometry/out')

        # Publicar o status do Companion Computer
        self.mav_comp_id_msg.header.stamp = Clock().now().to_msg() 
        self.mav_comp_id_msg.state = 4  # MAV_STATE_ACTIVE
        self.mav_comp_id_msg.component = 197  # MAV_COMP_ID_VISUAL_INERTIAL_ODOMETRY
        self.companion_computer_pub.publish(self.mav_comp_id_msg)

        # Log para indicar que o status do Companion Computer está sendo publicado
        self.get_logger().info('Status do Companion Computer publicado no tópico /mavros/companion_process/status')

    def broadcast_transform(self):
        # Publicar uma transformação estática entre "odom_frame_ned" e "odom" (ou outro frame que o MAVROS espera)
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "odom_frame_ned"
        t.child_frame_id = "odom"  # Altere para o frame que você deseja associar

        # Definir a transformação (aqui é uma identidade, ajuste conforme necessário)
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0

        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0

        # Publicar a transformação
        self.br.sendTransform(t)

        # Log para indicar que a transformação está sendo publicada
        self.get_logger().info('Transformação entre odom_frame_ned e odom publicada')

def main(args=None):
    rclpy.init(args=args)
    rs_b = RealsenseBridge()
    rclpy.spin(rs_b)
    rs_b.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

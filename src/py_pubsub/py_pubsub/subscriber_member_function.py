from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped

class VisualInertialPosePublisher(Node):
    def __init__(self):
        super().__init__('VIO_pose_pub')
        self.pose_publisher_ = self.create_publisher(PoseStamped, '/mavros/vision_pose/pose', 10)
        self.odometry_subscription_ = self.create_subscription(
            Odometry, '/t265/pose/sample', self.odometry_callback, qos_profile=qos_profile_sensor_data)
        
        self.br = TransformBroadcaster(self)
        self.timer = self.create_timer(10, self.timer_callback)

    def odometry_callback(self, msg):
        pose_msg = PoseStamped()
        pose_msg.header.stamp = msg.header.stamp
        pose_msg.header.frame_id = "odom"

        # Copiar e rotacionar os dados da pose
        position = msg.pose.pose.position
        rotated_position = Vector3()
        rotated_position.x = position.x
        rotated_position.y = -position.y
        rotated_position.z = -position.z
        pose_msg.pose.position = rotated_position

        # Copiar e rotacionar a orientação
        orientation = msg.pose.pose.orientation
        euler_angles = self.quaternion_to_euler(orientation)
        euler_angles[0] += math.pi  # Rotacionar 180 graus no eixo X
        rotated_quaternion = self.euler_to_quaternion(euler_angles)
        pose_msg.pose.orientation.x = rotated_quaternion[0]
        pose_msg.pose.orientation.y = rotated_quaternion[1]
        pose_msg.pose.orientation.z = rotated_quaternion[2]
        pose_msg.pose.orientation.w = rotated_quaternion[3]

        # Publicar a pose
        self.pose_publisher_.publish(pose_msg)

    def timer_callback(self):
        self.get_logger().info('Publicando pose visual-inercial')

        # Publicar transformação estática entre "odom_frame_ned" e "odom"
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "odom_frame_ned"
        t.child_frame_id = "odom"
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0

        self.br.sendTransform(t)


#!/usr/bin/env python

import rospy
import tf2_ros
import tf2_geometry_msgs
import tf
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from mavros_msgs.msg import LandingTarget
import math

def main():
    rospy.init_node('vision_to_mavros')

    # Variáveis para navegação precisa
    camera_pose_publisher = rospy.Publisher('vision_pose', PoseStamped, queue_size=10)
    body_path_publisher = rospy.Publisher('body_frame/path', Path, queue_size=1)

    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)

    msg_body_pose = PoseStamped()
    body_path = Path()

    # Parâmetros do launch file ou valores padrões
    target_frame_id = rospy.get_param('~target_frame_id', '/camera_odom_frame')
    source_frame_id = rospy.get_param('~source_frame_id', '/camera_link')
    output_rate = rospy.get_param('~output_rate', 20.0)
    roll_cam = rospy.get_param('~roll_cam', 0.0)
    pitch_cam = rospy.get_param('~pitch_cam', 0.0)
    yaw_cam = rospy.get_param('~yaw_cam', 1.5707963)
    gamma_world = rospy.get_param('~gamma_world', -1.5707963)

    # Variáveis para pouso preciso (opcional)
    enable_precland = rospy.get_param('~enable_precland', False)
    precland_target_frame_id = rospy.get_param('~precland_target_frame_id', '/landing_target')
    precland_camera_frame_id = rospy.get_param('~precland_camera_frame_id', '/camera_fisheye2_optical_frame')

    precland_msg_publisher = None
    if enable_precland:
        precland_msg_publisher = rospy.Publisher('landing_raw', LandingTarget, queue_size=10)

    rate = rospy.Rate(output_rate)

    while not rospy.is_shutdown():
        try:
            now = rospy.Time.now()
            transform = tf_buffer.lookup_transform(target_frame_id, source_frame_id, rospy.Time(0))

            position_orig = transform.transform.translation
            quat_cam = transform.transform.rotation

            # 1) Rotação do frame original do mundo para o frame com y para frente.
            position_body_x = math.cos(gamma_world) * position_orig.x + math.sin(gamma_world) * position_orig.y
            position_body_y = -math.sin(gamma_world) * position_orig.x + math.cos(gamma_world) * position_orig.y
            position_body_z = position_orig.z

            # 2) Rotação do frame da câmera para o frame do corpo
            quat_cam_to_body_x = tf.transformations.quaternion_from_euler(roll_cam, 0, 0)
            quat_cam_to_body_y = tf.transformations.quaternion_from_euler(0, pitch_cam, 0)
            quat_cam_to_body_z = tf.transformations.quaternion_from_euler(0, 0, yaw_cam)
            quat_rot_z = tf.transformations.quaternion_from_euler(0, 0, -gamma_world)

            quat_body = tf.transformations.quaternion_multiply(
                quat_rot_z,
                tf.transformations.quaternion_multiply(quat_cam, tf.transformations.quaternion_multiply(quat_cam_to_body_x, tf.transformations.quaternion_multiply(quat_cam_to_body_y, quat_cam_to_body_z)))
            )

            # Normaliza o quaternion
            quat_body = tf.transformations.unit_vector(quat_body)

            # Prepara a mensagem PoseStamped
            msg_body_pose.header.stamp = rospy.Time.now()
            msg_body_pose.header.frame_id = target_frame_id
            msg_body_pose.pose.position.x = position_body_x
            msg_body_pose.pose.position.y = position_body_y
            msg_body_pose.pose.position.z = position_body_z
            msg_body_pose.pose.orientation.x = quat_body[0]
            msg_body_pose.pose.orientation.y = quat_body[1]
            msg_body_pose.pose.orientation.z = quat_body[2]
            msg_body_pose.pose.orientation.w = quat_body[3]

            # Publica a pose da câmera no frame do corpo
            camera_pose_publisher.publish(msg_body_pose)

            # Publica a trajetória para visualização
            body_path.header.stamp = msg_body_pose.header.stamp
            body_path.header.frame_id = msg_body_pose.header.frame_id
            body_path.poses.append(msg_body_pose)
            body_path_publisher.publish(body_path)

        except tf2_ros.LookupException as ex:
            rospy.logwarn(ex)
            rospy.sleep(1.0)

        if enable_precland:
            try:
                if tf_buffer.can_transform(precland_camera_frame_id, precland_target_frame_id, rospy.Time(0)):
                    transform = tf_buffer.lookup_transform(precland_camera_frame_id, precland_target_frame_id, rospy.Time(0))

                    msg_landing_target = LandingTarget()
                    msg_landing_target.header.frame_id = transform.header.frame_id
                    msg_landing_target.header.stamp = transform.header.stamp
                    msg_landing_target.target_num = 0
                    msg_landing_target.frame = LandingTarget.LOCAL_NED
                    msg_landing_target.type = LandingTarget.VISION_FIDUCIAL

                    msg_landing_target.angle[0] = math.atan2(transform.transform.translation.x, transform.transform.translation.z)
                    msg_landing_target.angle[1] = math.atan2(transform.transform.translation.y, transform.transform.translation.z)
                    msg_landing_target.distance = math.sqrt(
                        transform.transform.translation.x**2 + transform.transform.translation.y**2 + transform.transform.translation.z**2
                    )

                    # Publica a mensagem de pouso preciso
                    precland_msg_publisher.publish(msg_landing_target)
                    rospy.loginfo("Landing target detected")

            except tf2_ros.LookupException as ex:
                rospy.logwarn(ex)
                rospy.sleep(1.0)

        rate.sleep()

if __name__ == '__main__':
    main()

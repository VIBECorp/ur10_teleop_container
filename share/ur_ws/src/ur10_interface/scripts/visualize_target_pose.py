#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, TransformStamped
from std_msgs.msg import Bool
import tf2_ros


class TargetPoseMarker(Node):
    def __init__(self):
        super().__init__('target_pose_marker_node')
        
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # PoseStamped Subscriber 생성
        self.pose_sub = self.create_subscription(
            PoseStamped,
            '/target_pose',
            self.pose_callback,
            10
        )
        
        self.create_subscription(Bool, "/shutdown", self.shutdown_callback, 10)
        
    def shutdown_callback(self, msg):
        if msg.data:
            rclpy.shutdown()

    def pose_callback(self, msg):
        # self.get_logger().info(f"Received Target Pose: x={msg.pose.position.x}, y={msg.pose.position.y}, z={msg.pose.position.z}")

        # Transform 메시지 생성
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = msg.header.frame_id  # 기준 프레임 (예: "base_link")
        transform.child_frame_id = "target_pose_frame"  # 새로 생성될 TF 프레임

        # 위치 설정
        transform.transform.translation.x = msg.pose.position.x
        transform.transform.translation.y = msg.pose.position.y
        transform.transform.translation.z = msg.pose.position.z

        # 회전 설정 (쿼터니언 그대로 사용)
        transform.transform.rotation = msg.pose.orientation

        # TF 브로드캐스트
        self.tf_broadcaster.sendTransform(transform)

def main(args=None):
    rclpy.init(args=args)

    target_pose_marker_publisher = TargetPoseMarker()
    rclpy.spin(target_pose_marker_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    target_pose_marker_publisher.destroy_node()
    rclpy.shutdown()
        
if __name__ == '__main__':
    main()
"""ROS 2 발행 (선택). rclpy는 이 파일에서만 쓴다.

- objects 토픽 (geometry_msgs/PoseArray): 인식된 물체 전체의 윗면 중심. 방향은 단위 쿼터니언(의미 없음)
- target 토픽 (geometry_msgs/PointStamped): 클릭으로 선택한 물체의 윗면 중심
이름은 config/frames.yaml에서 받는다.
⚠️ Humble·Jazzy 모두 같은 API. 실행 전 `source /opt/ros/<humble|jazzy>/setup.bash` 필요.
"""
from __future__ import annotations

import rclpy
from geometry_msgs.msg import Point, PointStamped, Pose, PoseArray


class ObjectPublisher:
    def __init__(self, frames: dict):
        if not rclpy.ok():
            rclpy.init()
        self.frame_id = frames["base_frame"]
        self.node = rclpy.create_node(frames["node_name"])
        self.pub_objects = self.node.create_publisher(PoseArray, frames["topics"]["objects"], 10)
        self.pub_target = self.node.create_publisher(PointStamped, frames["topics"]["target"], 10)

    def _stamp(self):
        return self.node.get_clock().now().to_msg()

    def publish_objects(self, xyz_list):
        msg = PoseArray()
        msg.header.frame_id = self.frame_id
        msg.header.stamp = self._stamp()
        for x, y, z in xyz_list:
            pose = Pose()
            pose.position = Point(x=float(x), y=float(y), z=float(z))
            pose.orientation.w = 1.0
            msg.poses.append(pose)
        self.pub_objects.publish(msg)

    def publish_target(self, xyz):
        msg = PointStamped()
        msg.header.frame_id = self.frame_id
        msg.header.stamp = self._stamp()
        msg.point = Point(x=float(xyz[0]), y=float(xyz[1]), z=float(xyz[2]))
        self.pub_target.publish(msg)

    def close(self):
        self.node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

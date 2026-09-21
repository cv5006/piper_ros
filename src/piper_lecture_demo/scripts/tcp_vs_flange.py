#!/usr/bin/env python3
"""MoveIt 이 맞추는 것은 link6 이지 손끝이 아니다. (§12.3)

rviz 에서 볼 수 없는 것이 이것 하나다. MotionPlanning 의 목표 마커는 link6 에 붙어
있는데 화면에는 그렇게 안 쓰여 있고, 손끝은 그보다 더 나가 있다. 첫 픽앤플레이스가
빗나가는 자리다.

이 노드는 두 점을 찍고 그 사이 거리를 숫자로 낸다.

    빨강  link6 원점   — MoveIt 이 목표 자세에 맞추는 자리
    파랑  손끝(TCP)    — 실제로 물건에 닿는 자리

읽는 것은 TF 뿐이다. MoveIt 도 URDF 파싱도 필요 없어서 MoveIt 스택 위에서도,
URDF 스택 위에서도 그대로 돈다.

    ros2 run piper_lecture_demo tcp_vs_flange.py

손끝 자리는 link7(손가락) 프레임에서 가져온다. 없으면 tip_link 에서 z 로
tcp_offset 만큼 나간 점을 쓴다 — 기본값 0.1358 m 는 URDF 와 그리퍼 메시에서 구한
값이고 마침 link7 원점과 같다.
"""

import numpy as np
import rclpy
from geometry_msgs.msg import Point
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from tf2_ros import Buffer, TransformListener
from visualization_msgs.msg import Marker, MarkerArray


class TcpOffset(Node):

    def __init__(self):
        super().__init__("tcp_vs_flange")
        self.declare_parameter("base_link", "base_link")
        self.declare_parameter("tip_link", "link6")      # MoveIt 그룹의 끝. 목표가 붙는 자리
        self.declare_parameter("finger_link", "link7")   # 손가락. 없으면 오프셋으로 대신한다
        self.declare_parameter("tcp_offset", 0.1358)     # tip_link 의 z 로 잰 손끝까지 [m]

        self.base = self.get_parameter("base_link").value
        self.tip = self.get_parameter("tip_link").value
        self.finger = self.get_parameter("finger_link").value
        self.offset = float(self.get_parameter("tcp_offset").value)

        self.buf = Buffer()
        self.listener = TransformListener(self.buf, self)
        qos = QoSProfile(depth=1,
                         reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pub = self.create_publisher(MarkerArray, "/lecture_markers", qos)
        self.create_timer(0.1, self.tick)
        self.reported = False

        self.get_logger().info(
            f"marking {self.tip} (what MoveIt aims at) and the TCP. waiting for TF")

    def lookup(self, child):
        try:
            t = self.buf.lookup_transform(self.base, child, rclpy.time.Time()).transform
        except Exception:
            return None, None
        p = np.array([t.translation.x, t.translation.y, t.translation.z])
        q = np.array([t.rotation.x, t.rotation.y, t.rotation.z, t.rotation.w])
        return p, q

    @staticmethod
    def z_axis(q):
        """쿼터니언에서 z 축 방향만 꺼낸다."""
        x, y, z, w = q
        return np.array([2 * (x * z + w * y), 2 * (y * z - w * x), 1 - 2 * (x * x + y * y)])

    def tick(self):
        p6, q6 = self.lookup(self.tip)
        if p6 is None:
            return
        p_tip, _ = self.lookup(self.finger)
        source = self.finger
        if p_tip is None:                       # 손가락 프레임이 없으면 오프셋으로
            p_tip = p6 + self.z_axis(q6) * self.offset
            source = f"{self.tip} + z {self.offset:.4f} m"

        gap = float(np.linalg.norm(p_tip - p6))
        if not self.reported:
            self.get_logger().info(f"  {self.tip} origin = {np.round(p6, 3).tolist()}")
            self.get_logger().info(f"  TCP ({source}) = {np.round(p_tip, 3).tolist()}")
            self.get_logger().info(f"  gap {gap:.4f} m - aiming at the goal pose as-is misses by this much")
            self.reported = True

        now = self.get_clock().now().to_msg()
        arr = MarkerArray()

        def base(mid, mtype):
            m = Marker()
            m.header.frame_id = self.base
            m.header.stamp = now
            m.ns = "tcp_vs_flange"
            m.id = mid
            m.type = mtype
            m.action = Marker.ADD
            m.pose.orientation.w = 1.0
            return m

        for mid, p, rgb in ((0, p6, (0.95, 0.35, 0.20)), (1, p_tip, (0.25, 0.55, 0.95))):
            m = base(mid, Marker.SPHERE)
            m.pose.position.x, m.pose.position.y, m.pose.position.z = p
            m.scale.x = m.scale.y = m.scale.z = 0.022
            m.color.r, m.color.g, m.color.b = rgb
            m.color.a = 0.95
            arr.markers.append(m)

        line = base(2, Marker.LINE_LIST)
        line.points = [Point(x=float(p6[0]), y=float(p6[1]), z=float(p6[2])),
                       Point(x=float(p_tip[0]), y=float(p_tip[1]), z=float(p_tip[2]))]
        line.scale.x = 0.005
        line.color.r, line.color.g, line.color.b, line.color.a = 1.0, 0.95, 0.3, 0.95
        arr.markers.append(line)

        text = base(3, Marker.TEXT_VIEW_FACING)
        mid_p = 0.5 * (p6 + p_tip)
        text.pose.position.x, text.pose.position.y = mid_p[0], mid_p[1]
        text.pose.position.z = mid_p[2] + 0.06
        text.scale.z = 0.030
        text.color.r = text.color.g = text.color.b = text.color.a = 1.0
        text.text = f"goal {self.tip}  ->  fingertip   {gap:.4f} m"
        arr.markers.append(text)

        self.pub.publish(arr)


def main():
    rclpy.init()
    node = TcpOffset()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()

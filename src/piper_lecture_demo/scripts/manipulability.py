#!/usr/bin/env python3
"""manipulability 타원체 + sigma_min. (M1 · M3-3 · M3-4)

    ros2 launch piper_lecture_demo urdf_demos.launch.py [preset:=elbow] [readout:=marker]

지표를 둘 낸다. 하나만 보면 놓친다.

    선속도 (J 위 3행)  -> 타원체
    전체   (J 6xN)     -> sigma_min

joint5 = 0 에서 sigma_min 은 0 인데 타원체는 둥글다.

숫자는 2D 숫자판(/manipulability_readout, Image). rviz2 에 HUD 가 없다.
Jacobian 은 kinematics.py 것이다. 기본은 감추기 — 버튼으로 드러낸다.
"""

import cv2
import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Point, TransformStamped
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import Image, JointState
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool
from std_srvs.srv import SetBool
from tf2_ros import StaticTransformBroadcaster
from visualization_msgs.msg import Marker, MarkerArray

from piper_lecture_demo.kinematics import Chain, matrix_to_quaternion, metrics

# 타원체 주축 색. 1강 F3-2 와 같은 어법(x=빨강 y=초록 z=파랑)을 sigma1 sigma2 sigma3 에 준다.
AXIS_COLORS = ((0.90, 0.25, 0.25), (0.30, 0.75, 0.35), (0.30, 0.45, 0.90))
AXIS_COLORS_255 = tuple(tuple(int(round(v * 255)) for v in c) for c in AXIS_COLORS)


class Manipulability(Node):

    def __init__(self):
        super().__init__("manipulability")

        default_urdf = (get_package_share_directory("piper_description")
                        + "/urdf/piper_description.urdf")
        self.declare_parameter("urdf", default_urdf)
        self.declare_parameter("base_link", "base_link")
        self.declare_parameter("tip_link", "link6")
        self.declare_parameter("frame_id", "base_link")
        self.declare_parameter("scale", 0.30)         # 타원체 크기 배율
        self.declare_parameter("report_period", 1.0)  # 터미널 출력 주기 [s], 0 이면 끔
        # 기준점(TCP) — tip_link 프레임 오프셋 [m]. [0,0,0] 이면 플랜지 기준.
        # 0.1358 은 URDF·그리퍼 메시에서 구한 파지 중심이고 link7 원점과 같다.
        self.declare_parameter("tcp_offset", [0.0, 0.0, 0.1358])
        # 숫자 표시 — image(2D 숫자판, 기본) / marker(3D 텍스트) / both / none
        self.declare_parameter("readout", "image")
        # **기본은 감추기다.** 화면을 비운 채로 시작해 버튼으로 드러내는 것이
        # 이 패키지의 연출이다 (ik_solutions 의 해와 같다).
        self.declare_parameter("show", False)

        urdf = self.get_parameter("urdf").value
        self.tcp = np.array(self.get_parameter("tcp_offset").value, dtype=float)
        self.tip_link = self.get_parameter("tip_link").value
        self.chain = Chain(urdf,
                           self.get_parameter("base_link").value,
                           self.tip_link,
                           tool=self.tcp)
        self.frame_id = self.get_parameter("frame_id").value
        self.scale = float(self.get_parameter("scale").value)
        self.readout = str(self.get_parameter("readout").value)
        if self.readout not in ("image", "marker", "both", "none"):
            self.get_logger().warn(
                f"readout='{self.readout}' is not a valid value. falling back to image")
            self.readout = "image"

        self.q = np.zeros(self.chain.dof)
        self.have_q = False
        self.show = bool(self.get_parameter("show").value)
        self.cleared = False        # 감춘 뒤 지우기 마커를 이미 보냈나

        self.pub = self.create_publisher(MarkerArray, "manipulability", 1)
        self.pub_img = self.create_publisher(Image, "manipulability_readout", 1)
        self.create_subscription(JointState, "/joint_states", self.on_joints, 10)
        self.srv_show = self.create_service(SetBool, "~/show", self.on_show)
        # 지금 켜져 있나를 래치해서 알린다. 패널이 나중에 떠도 상태를 맞출 수 있고,
        # 터미널로 껐다 켜도 버튼 라벨이 따라온다.
        self.pub_shown = self.create_publisher(
            Bool, "~/shown",
            QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                       durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.pub_shown.publish(Bool(data=self.show))
        self.create_timer(0.05, self.publish_markers)
        self.create_timer(0.1, self.publish_readout)

        period = float(self.get_parameter("report_period").value)
        if period > 0.0:
            self.create_timer(period, self.report)

        # /joint_states 에 발행자가 둘 이상이면 서로 덮어쓴다. rviz 의 로봇은 한쪽을,
        # 이 노드의 타원체는 다른 쪽을 따라가게 되어 「타원체가 로봇을 안 따라간다」로 보인다.
        # 원인은 대개 MoveIt 스택(joint_state_broadcaster)이나 앞서 띄운 데모가 살아 있는 것이다.
        self.warned_publishers = -1
        self.create_timer(2.0, self.check_publishers)

        # TCP 를 프레임으로도 내보낸다. rviz 에서 눈으로 확인할 수 있고
        # 다른 노드가 lookup_transform 으로 가져다 쓸 수도 있다.
        self.tf_static = StaticTransformBroadcaster(self)
        tf = TransformStamped()
        tf.header.stamp = self.get_clock().now().to_msg()
        tf.header.frame_id = self.tip_link
        tf.child_frame_id = "tcp"
        tf.transform.translation.x = float(self.tcp[0])
        tf.transform.translation.y = float(self.tcp[1])
        tf.transform.translation.z = float(self.tcp[2])
        tf.transform.rotation.w = 1.0
        self.tf_static.sendTransform(tf)

        self.get_logger().info(f"URDF={urdf}")
        self.get_logger().info(
            f"  chain {self.chain.base} -> {self.chain.tip}, "
            f"{self.chain.dof} axes {self.chain.names}")
        if self.chain.has_tool:
            self.get_logger().info(
                f"  reference = TCP: {np.round(self.tcp, 4).tolist()} m from {self.tip_link} "
                f"(frame name 'tcp')")
        else:
            self.get_logger().info(f"  reference = flange {self.tip_link} (no TCP offset)")

    def check_publishers(self):
        n = self.count_publishers("/joint_states")
        if n == self.warned_publishers:
            return
        self.warned_publishers = n
        if n > 1:
            self.get_logger().warn(
                f"/joint_states has {n} publishers. they overwrite each other, so the robot "
                "model and the ellipsoid drift apart.")
            self.get_logger().warn(
                "  check whether another demo or the MoveIt stack "
                "(joint_state_broadcaster) is still alive:")
            self.get_logger().warn(
                "    ros2 topic info /joint_states --verbose")
        elif n == 0:
            self.get_logger().warn("nothing publishes /joint_states. check that the slider GUI or the "
                                   "preset driver came up")

    def on_joints(self, msg):
        index = {n: i for i, n in enumerate(msg.name)}
        for k, name in enumerate(self.chain.names):
            if name in index:
                self.q[k] = msg.position[index[name]]
        self.have_q = True

    def state(self):
        """현재 자세의 손끝 위치 · 타원체 자세 · 지표 두 벌."""
        T = self.chain.fk(self.q)
        J = self.chain.jacobian(self.q)
        U, sigma, w, cond = metrics(J[:3, :])       # 선속도 — 타원체가 되는 쪽
        sigma6 = np.linalg.svd(J, compute_uv=False)  # 전체 6자유도
        return T[:3, 3], U, sigma, w, cond, sigma6

    def on_show(self, req, res):
        """타원체를 낼지 말지. ik_solutions 의 reveal 과 같은 방식이다 — 화면이 붐빌 때
        (해 여덟 벌이 겹쳐 있을 때) 타원체를 잠깐 치우려고 둔 것이다."""
        self.show = bool(req.data)
        self.cleared = False
        self.pub_shown.publish(Bool(data=self.show))
        res.success = True
        res.message = "showing the ellipsoid" if self.show else "hiding the ellipsoid"
        return res

    def clear_markers(self):
        """지우기 마커 하나. ns 는 비운다 — 채우면 rviz 가 디스플레이를 오류로
        표시하면서도 그리기는 정상으로 해서 원인을 찾기 어렵다."""
        m = Marker()
        m.header.frame_id = self.frame_id
        m.header.stamp = self.get_clock().now().to_msg()
        m.action = Marker.DELETEALL
        m.pose.orientation.w = 1.0
        arr = MarkerArray()
        arr.markers.append(m)
        self.pub.publish(arr)

    def publish_markers(self):
        if not self.have_q:
            return
        if not self.show:
            # 20 Hz 로 지우기를 계속 보낼 이유가 없다. 한 번만 보내고 쉰다.
            if not self.cleared:
                self.clear_markers()
                self.cleared = True
            return
        p, U, sigma, w, cond, sigma6 = self.state()
        quat = matrix_to_quaternion(U)
        now = self.get_clock().now().to_msg()
        arr = MarkerArray()

        def base(mid, mtype):
            m = Marker()
            m.header.frame_id = self.frame_id
            m.header.stamp = now
            m.ns = "manipulability"
            m.id = mid
            m.type = mtype
            m.action = Marker.ADD
            m.pose.orientation.w = 1.0
            return m

        # (1) 타원체 — 반축 길이가 sigma 다. 어느 방향으로 얼마나 빨리 움직일 수 있나.
        e = base(0, Marker.SPHERE)
        e.pose.position.x, e.pose.position.y, e.pose.position.z = p
        e.pose.orientation.x, e.pose.orientation.y, e.pose.orientation.z, e.pose.orientation.w = quat
        e.scale.x, e.scale.y, e.scale.z = (2.0 * self.scale * float(s) for s in sigma)
        e.color.r, e.color.g, e.color.b, e.color.a = 0.25, 0.55, 0.95, 0.45
        arr.markers.append(e)

        # (2) 주축 세 개 — 어느 축이 먼저 죽는지가 눈에 보인다.
        for i in range(3):
            a = base(1 + i, Marker.ARROW)
            tip = p + U[:, i] * self.scale * float(sigma[i])
            a.points = [_pt(p), _pt(tip)]
            a.scale.x, a.scale.y, a.scale.z = 0.004, 0.010, 0.012
            a.color.r, a.color.g, a.color.b = AXIS_COLORS[i]
            a.color.a = 0.95
            arr.markers.append(a)

        # (3) 숫자 — 타원체가 멀쩡해 보여도 6자유도 sigma_min 은 죽어 있을 수 있다.
        #     기본 표시는 Image 패널(readout) 쪽이다. 3D 텍스트는 readout=marker|both 일 때만.
        if self.readout in ("marker", "both"):
            t = base(4, Marker.TEXT_VIEW_FACING)
            t.pose.position.x, t.pose.position.y = p[0], p[1]
            t.pose.position.z = p[2] + 0.18
            t.scale.z = 0.032
            t.color.r = t.color.g = t.color.b = t.color.a = 1.0
            # 줄마다 이름을 먼저 둔다. rviz 가 줄을 접거나 겹쳐 그려도 무슨 숫자인지 읽힌다.
            cond6 = sigma6[0] / sigma6[-1] if sigma6[-1] > 1e-12 else np.inf
            t.text = chr(10).join([
                f"ref {'tcp' if self.chain.has_tool else 'flange'}",
                f"sigma_v {sigma[0]:.3f} {sigma[1]:.3f} {sigma[2]:.3f}",
                f"cond_v {_fmt(cond)}",
                f"w_v {w:.5f}",
                f"sigma_min_6d {sigma6[-1]:.5f}",
                f"cond_6d {_fmt(cond6)}",
            ])
            arr.markers.append(t)

        # (4) 기준점 자체. 타원체가 어디에 붙어 있는지가 한눈에 보이게 한다.
        d = base(5, Marker.SPHERE)
        d.pose.position.x, d.pose.position.y, d.pose.position.z = p
        d.scale.x = d.scale.y = d.scale.z = 0.012
        d.color.r, d.color.g, d.color.b, d.color.a = 1.0, 0.95, 0.3, 1.0
        arr.markers.append(d)

        self.pub.publish(arr)

    # ------------------------------------------------------------ 숫자판 (2D)
    # rviz2 에 화면 고정 오버레이가 없다. 이미지로 그려 보내면 기본 Image 디스플레이가
    # 받고, 카메라를 움직여도 크기·위치가 안 변한다. numpy + opencv 만, cv_bridge 없이.

    def publish_readout(self):
        if not self.have_q or self.readout not in ("image", "both"):
            return
        _, _, sigma, w, cond, sigma6 = self.state()
        img = self.render_readout(sigma, w, cond, sigma6)

        msg = Image()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.height, msg.width = img.shape[:2]
        msg.encoding = "rgb8"
        msg.is_bigendian = 0
        msg.step = msg.width * 3
        msg.data = img.tobytes()
        self.pub_img.publish(msg)

    def render_readout(self, sigma, w, cond, sigma6):
        W, H = 520, 336
        img = np.full((H, W, 3), (28, 28, 32), dtype=np.uint8)
        font = cv2.FONT_HERSHEY_SIMPLEX
        cond6 = sigma6[0] / sigma6[-1] if sigma6[-1] > 1e-12 else np.inf

        def text(s, x, y, size, color, thick=1):
            cv2.putText(img, s, (x, y), font, size, color, thick, cv2.LINE_AA)

        ref = "TCP" if self.chain.has_tool else "FLANGE"
        text(f"manipulability   ref = {ref}", 16, 34, 0.62, (235, 235, 240), 1)
        cv2.line(img, (16, 46), (W - 16, 46), (80, 80, 90), 1)

        # 선속도 타원체의 반축 — 막대 길이를 고정 기준(1.0)에 맞춰 자세끼리 비교되게 한다
        text("velocity ellipsoid  (m/s per rad/s)", 16, 74, 0.5, (150, 150, 160), 1)
        bar_x, bar_w, full = 130, W - 130 - 110, 1.0
        for i, sv in enumerate(sigma):
            y = 104 + i * 34
            text(f"sigma{i + 1}", 16, y + 6, 0.55, AXIS_COLORS_255[i], 1)
            cv2.rectangle(img, (bar_x, y - 10), (bar_x + bar_w, y + 8), (50, 50, 58), -1)
            fill = int(bar_w * min(1.0, float(sv) / full))
            cv2.rectangle(img, (bar_x, y - 10), (bar_x + fill, y + 8), AXIS_COLORS_255[i], -1)
            text(f"{sv:.3f}", bar_x + bar_w + 12, y + 6, 0.55, (225, 225, 230), 1)

        cv2.line(img, (16, 218), (W - 16, 218), (80, 80, 90), 1)
        text(f"cond {_fmt(cond)}", 16, 246, 0.58, (225, 225, 230), 1)
        text(f"w {w:.5f}", 190, 246, 0.58, (225, 225, 230), 1)

        # 6자유도 쪽은 타원체에 안 보이는 값이라 따로 준다. 둘 다 항상 보여준다 —
        # 어느 한쪽만 보면 놓치는 자세가 이 로봇에 실제로 있다 (joint5 = 0).
        text(f"6-DOF  sigma_min {sigma6[-1]:.5f}   cond {_fmt(cond6)}",
             16, 280, 0.55, (200, 200, 210), 1)

        # 판정 기준은 이 로봇에서 실측한 값이다 (NOTES.md 「특이점 판정 기준」):
        #   cond_6d 400 자세 표본에서 KDL IK 성공률 —
        #   < 100 : 100 %   /   100 ~ 1000 : 98 %   /   > 1000 : 67 %
        if cond6 >= 1000.0:
            band, color = "SINGULAR    IK fails ~1 in 3", (235, 90, 80)
        elif cond6 >= 100.0:
            band, color = "CAUTION     IK occasionally fails", (235, 190, 80)
        else:
            band, color = "OK", (140, 210, 140)
        cv2.rectangle(img, (16, 296), (W - 16, 324), (44, 44, 50), -1)
        text(band, 26, 316, 0.56, color, 2 if cond6 >= 100.0 else 1)
        return img

    def report(self):
        if not self.have_q:
            return
        _, _, sigma, w, cond, sigma6 = self.state()
        cond6 = sigma6[0] / sigma6[-1] if sigma6[-1] > 1e-12 else np.inf
        self.get_logger().info(
            f"linear sigma=[{sigma[0]:.4f} {sigma[1]:.4f} {sigma[2]:.4f}] "
            f"w={w:.6f} cond={_fmt(cond)}   |   "
            f"6-DOF sigma_min={sigma6[-1]:.6f} cond={_fmt(cond6)}")


def _fmt(v):
    return f"{v:.1f}" if np.isfinite(v) else "inf"


def _pt(v):
    return Point(x=float(v[0]), y=float(v[1]), z=float(v[2]))


def main():
    rclpy.init()
    node = Manipulability()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()

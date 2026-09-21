#!/usr/bin/env python3
"""작업영역 점구름 + 조건수 컬러맵. (M3-2)

    관절 한계 안 샘플링 -> FK -> J -> 조건수 -> 색 -> PointCloud2
    ros2 launch piper_lecture_demo urdf_demos.launch.py workspace:=true

구름을 둘로 낸다. 이 차이가 설치 위치를 정할 때의 핵심이다.

    ~/workspace/reachable   닿기만 하면 되는 영역
    ~/workspace/oriented    정해둔 접근 방향으로 닿는 영역

판정식은 TCP 프레임 z 축과 월드 -Z 의 각 (기본 허용 25도). joint6 회전은 자유.

⚠ 교과서의 dexterous workspace 가 아니다. 상세는 NOTES.md.
읽는 것은 URDF 뿐. /joint_states 를 안 쓰므로 URDF 스택에 얹힌다. 계산 16 초.
"""

import struct

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2, PointField

from piper_lecture_demo.kinematics import Chain

# 파랑(잘 움직인다) -> 청록 -> 초록 -> 노랑 -> 빨강(특이점에 가깝다)
COLORMAP = np.array([
    [0.19, 0.39, 0.85],
    [0.20, 0.75, 0.80],
    [0.35, 0.78, 0.35],
    [0.95, 0.80, 0.20],
    [0.85, 0.20, 0.18],
])


def colorize(t):
    """t in [0, 1] -> (r, g, b). 구간 선형 보간."""
    t = float(np.clip(t, 0.0, 1.0)) * (len(COLORMAP) - 1)
    i = int(np.floor(t))
    if i >= len(COLORMAP) - 1:
        return COLORMAP[-1]
    f = t - i
    return COLORMAP[i] * (1.0 - f) + COLORMAP[i + 1] * f


def pack_rgb(rgb):
    r, g, b = (int(round(float(c) * 255.0)) for c in rgb)
    return struct.unpack("f", struct.pack("I", (r << 16) | (g << 8) | b))[0]


class Workspace(Node):

    def __init__(self):
        super().__init__("workspace")

        default_urdf = (get_package_share_directory("piper_description")
                        + "/urdf/piper_description.urdf")
        self.declare_parameter("urdf", default_urdf)
        self.declare_parameter("base_link", "base_link")
        self.declare_parameter("tip_link", "link6")
        self.declare_parameter("frame_id", "base_link")
        # 관절별 샘플 수. 1 = 그 관절을 0 으로 고정. joint1 을 1 로 두면 수직 단면.
        # ⚠ joint4 고정 금지 — 접근 방향이 아래로 쏠려 「자세까지 맞는 영역」이 3배 부푼다.
        # ⚠ joint6 은 1 로 둬도 된다 — 자기 축 회전이라 TCP 위치·접근 방향을 안 바꾼다.
        self.declare_parameter("samples", [11, 13, 13, 5, 5, 1])
        # 색 범위: 조건수를 log10 으로 본다. 1 -> 0, 100 -> 2
        self.declare_parameter("cond_log_min", 0.0)
        self.declare_parameter("cond_log_max", 2.0)
        # 「원하는 자세」의 정의 — 손끝 z 축이 월드 -Z(바닥) 과 이루는 각 허용치 [deg]
        self.declare_parameter("approach_tol_deg", 25.0)
        # 부피 비율을 낼 때 쓰는 복셀 한 변 [m]
        self.declare_parameter("voxel", 0.02)
        # 기준점(TCP). manipulability 와 같은 값이다 — 「닿는다」는 것은 플랜지가 아니라
        # 손끝이 닿는다는 뜻이므로 작업영역도 이 점으로 그린다.
        self.declare_parameter("tcp_offset", [0.0, 0.0, 0.1358])

        urdf = self.get_parameter("urdf").value
        self.tcp = np.array(self.get_parameter("tcp_offset").value, dtype=float)
        self.chain = Chain(urdf,
                           self.get_parameter("base_link").value,
                           self.get_parameter("tip_link").value,
                           tool=self.tcp)
        self.frame_id = self.get_parameter("frame_id").value

        qos = QoSProfile(depth=1,
                         reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pub_reach = self.create_publisher(PointCloud2, "workspace/reachable", qos)
        self.pub_oriented = self.create_publisher(PointCloud2, "workspace/oriented", qos)

        self.compute_and_publish()

    # ------------------------------------------------------------------ 계산

    def grid(self):
        """관절 한계 안에서 격자 샘플을 만든다."""
        counts = list(self.get_parameter("samples").value)
        counts += [1] * (self.chain.dof - len(counts))
        axes = []
        for k in range(self.chain.dof):
            n = max(1, int(counts[k]))
            if n == 1:
                axes.append(np.array([0.0]))
            else:
                axes.append(np.linspace(self.chain.lower[k], self.chain.upper[k], n))
        return axes, counts

    def compute_and_publish(self):
        axes, counts = self.grid()
        total = int(np.prod([len(a) for a in axes]))
        self.get_logger().info(
            f"sampling grid {[len(a) for a in axes]} = {total} poses. computing...")
        self.get_logger().info(
            f"  reference = {'TCP ' + str(np.round(self.tcp, 4).tolist()) + ' m' if self.chain.has_tool else 'flange'}")

        lo = float(self.get_parameter("cond_log_min").value)
        hi = float(self.get_parameter("cond_log_max").value)
        cos_tol = np.cos(np.deg2rad(float(self.get_parameter("approach_tol_deg").value)))

        # FK 와 Jacobian 은 자세마다 돌리고, SVD 는 한 번에 몰아서 한다.
        # SVD 를 자세마다 부르면 이 계산의 대부분이 거기서 간다.
        pos, down, Jv_all = [], [], []
        for q in _iterate(axes):
            T = self.chain.fk(q)
            pos.append(T[:3, 3])
            # 손끝(link6) 의 z 축이 아래를 보는가. 이것이 「원하는 자세」의 정의다.
            down.append(float(T[:3, 2] @ np.array([0.0, 0.0, -1.0])) >= cos_tol)
            Jv_all.append(self.chain.jacobian(q)[:3, :])

        pos = np.array(pos)
        down = np.array(down)
        sigma = np.linalg.svd(np.array(Jv_all), compute_uv=False)   # (N, 3)
        conds = np.where(sigma[:, -1] > 1e-12, sigma[:, 0] / np.maximum(sigma[:, -1], 1e-12),
                         np.inf)

        t = np.clip((np.log10(np.where(np.isfinite(conds), conds, 10.0 ** hi)) - lo) / (hi - lo),
                    0.0, 1.0)
        rgb = np.array([pack_rgb(colorize(v)) for v in t])

        reach = [(pos[i, 0], pos[i, 1], pos[i, 2], rgb[i]) for i in range(len(pos))]
        oriented = [reach[i] for i in range(len(pos)) if down[i]]

        self.pub_reach.publish(self.to_cloud(reach))
        self.pub_oriented.publish(self.to_cloud(oriented))

        conds = conds[np.isfinite(conds)]
        pts = pos
        radius = np.linalg.norm(pts, axis=1)              # base_link 원점에서 잰 3차원 거리
        horiz = np.hypot(pts[:, 0], pts[:, 1])            # 수평 반경 — 설치 위치를 정할 때 쓰는 값

        # ⚠ 비율은 「자세 개수」가 아니라 「부피」로 낸다. 같은 점을 여러 자세가 찍으므로
        #    자세를 세면 도달 자세가 많은 자리가 과대 반영된다. 강의에서 말하려는 것은
        #    영역의 크기지 자세의 개수가 아니다.
        vox = float(self.get_parameter("voxel").value)
        keys_r = set(map(tuple, np.floor(pts / vox).astype(int)))
        keys_o = set(map(tuple, np.floor(pts[down] / vox).astype(int)))
        self.get_logger().info(
            "done - {1} of {0} reachable poses also meet the approach direction".format(
                len(reach), len(oriented)))
        self.get_logger().info(
            "  by volume ({0:.0f} cm voxels): {1} / {2} voxels = {3:.1f} %".format(
                vox * 100, len(keys_o), len(keys_r), 100.0 * len(keys_o) / max(1, len(keys_r))))
        self.get_logger().info(
            "  NOTE this ratio is a lower bound; it rises as the wrist "
            "(joint4/joint5) is sampled more finely")
        self.get_logger().info(
            "  distance from the base_link origin  min {0:.3f} m / max {1:.3f} m".format(
                radius.min(), radius.max()))
        self.get_logger().info(
            "  horizontal radius                  max {0:.3f} m / height {1:.3f} ~ {2:.3f} m".format(
                horiz.max(), pts[:, 2].min(), pts[:, 2].max()))
        self.get_logger().info(
            "  condition number  median {0:.1f} / 90th pct {1:.1f} / max {2:.1f}".format(
                np.median(conds), np.percentile(conds, 90), conds.max()))
        self.get_logger().info("  the clouds are latched, so rviz still receives them if started later.")

    # ------------------------------------------------------------------ 메시지

    def to_cloud(self, rows):
        msg = PointCloud2()
        msg.header.frame_id = self.frame_id
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.height = 1
        msg.width = len(rows)
        msg.fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name="rgb", offset=12, datatype=PointField.FLOAT32, count=1),
        ]
        msg.is_bigendian = False
        msg.point_step = 16
        msg.row_step = msg.point_step * msg.width
        msg.is_dense = True
        msg.data = np.array(rows, dtype=np.float32).tobytes() if rows else b""
        return msg


def _iterate(axes):
    """격자를 하나씩 돌려준다. itertools.product 를 쓰지 않은 것은 배열로 받기 위해서다."""
    idx = np.zeros(len(axes), dtype=int)
    sizes = [len(a) for a in axes]
    while True:
        yield np.array([axes[k][idx[k]] for k in range(len(axes))])
        k = len(axes) - 1
        while k >= 0:
            idx[k] += 1
            if idx[k] < sizes[k]:
                break
            idx[k] = 0
            k -= 1
        if k < 0:
            return


def main():
    rclpy.init()
    node = Workspace()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()

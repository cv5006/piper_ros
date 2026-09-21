#!/usr/bin/env python3
"""같은 손끝 자세를 만드는 해가 여럿이다. 그런데 쓸 수 있는 것은 하나다. (M2 · M3-2)

팔이 있는 자리의 손끝 자세를 목표로 잡고, 그 자세를 만드는 관절값을 전부 찾는다.
찾는 방법은 강의에서 다룬 그대로다 — q <- q + J⁺(x_target - f(q)) 를 무작위 초기값에서
여러 번 돌리고, 수렴한 것들을 모아 중복을 지운다 (DLS 감쇠).

상주하면서 부를 때마다 다시 푼다. 팔을 옮기고 또 누르면 그 자리에서 다시 찾는다.

    ros2 run piper_lecture_demo ik_solutions.py          # 뜨면 한 번 풀고 대기
    ros2 service call /ik_solutions/solve std_srvs/srv/Trigger
    (또는 강의 패널의 「Solve IK」 버튼)

두 토픽으로 낸다. rviz 의 **MarkerArray** 디스플레이 둘이 각각 받는다.
해 하나가 로봇 한 벌이고, 링크마다 메시 Marker 를 제자리에 놓는다.

    /ik_solutions/valid     관절 한계 안에 드는 해   (초록)
    /ik_solutions/invalid   한계 밖으로 밀려난 해     (빨강)

**MoveIt 을 쓰지 않는다.** 한때 MoveIt 의 Trajectory 디스플레이로 그렸는데 그것이
SRDF 를 요구해서 URDF 스택(robot_description 만 있는 스택)에서는 뜨지 못했다 —
`Unable to parse SRDF` 가 나고 디스플레이에 빨간 오류 표시만 남는다.
메시를 직접 놓으면 rviz 기본 플러그인만으로 되고, **두 스택 어디서든 같게 보인다.**
덤으로 애니메이션이 없어 화면이 깜박이지 않는다.

**빨강은 풀자마자 내지 않는다.** 디스플레이는 rviz 에서 켜져 있지만 이 노드가
빈 메시지를 내고 있어서 화면에는 아무것도 없다. 한 번 더 부르면 그때 낸다.

    ros2 service call /ik_solutions/reveal std_srvs/srv/SetBool "{data: true}"
    (또는 강의 패널의 「Show remaining solutions」 버튼)

초록도 같은 식으로 끈다. 아무것도 없는 화면에서 시작해 초록 하나를 보이고, 그 다음
빨강 일곱을 드러내는 진행을 만들 수 있다.

    ros2 service call /ik_solutions/show_valid std_srvs/srv/SetBool "{data: false}"
    (또는 강의 패널의 「Hide usable solutions」 버튼)

*"해가 몇 개일까?"* 를 묻고 초록 하나를 보인 뒤 나머지 일곱을 한꺼번에 드러내는
것이 이 데모의 장치다. 다시 풀지 않고 이미 찾아둔 것을 내보내므로 즉시 뜬다.

⚠ 그런데 **초록 하나는 지금 로봇이 있는 그 자세**다 (아래 참조). 진짜 로봇과
정확히 겹쳐 그려지므로 화면상으로는 변화가 없다 — 눈에 보이는 것은 터미널의
개수와, 이어서 드러낼 빨강 쪽이다.

⚠ 이 팔에서 보게 될 것: **해는 8개인데 쓸 수 있는 것은 거의 항상 1개다.**

  무작위 목표 40개로 확인한 결과 — 관절 한계를 무시하면 8해가 지배적(30건)인데,
  한계 안에 드는 해는 29건이 정확히 1개, 4건이 2개, 7건이 0개였다.

  막는 것은 손목이다. 손목을 뒤집으려면 joint4 · joint6 을 180도 돌려야 하는데
  한계가 각각 ±100도 · ±120도 라 닿지 않고, joint5 는 ±70도 뿐이다.
  팔꿈치 뒤집기는 joint3(-170도~0), 어깨 뒤집기는 joint2(0~180도)가 막는다.

  그래서 이 팔에서는 rviz 마커를 끌어도 elbow up/down 이 튀는 것을 볼 수 없다.
  「해가 여럿」은 기구학의 성질이고, 「쓸 수 있는 해가 하나」는 이 로봇의 성질이다.

  덧붙여 이 팔은 구형 손목이 아니다 — joint4 · joint5 축은 한 점에서 만나지만
  joint6 축이 0.091 m 떨어져 있다. 해석해 공식이 깔끔하게 떨어지지 않는 구조이고,
  레포가 KDL 수치해를 쓰는 이유다.
"""

import time

import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool
from std_srvs.srv import SetBool, Trigger
from visualization_msgs.msg import Marker, MarkerArray

from piper_lecture_demo.kinematics import Chain, Visuals, matrix_to_quaternion

ARM = [f"joint{i}" for i in range(1, 7)]
VALID_RGB = (60, 200, 90)
INVALID_RGB = (220, 70, 60)


def pose_error(T, Td):
    """위치 오차 3 + 자세 오차 3. 자세는 회전행렬 차이의 축각으로 잰다."""
    ep = Td[:3, 3] - T[:3, 3]
    R = Td[:3, :3] @ T[:3, :3].T
    ang = np.arccos(np.clip((np.trace(R) - 1.0) / 2.0, -1.0, 1.0))
    if ang < 1e-9:
        eo = np.zeros(3)
    else:
        eo = ang / (2.0 * np.sin(ang)) * np.array(
            [R[2, 1] - R[1, 2], R[0, 2] - R[2, 0], R[1, 0] - R[0, 1]])
    return np.concatenate([ep, eo])


def solve_ik(chain, Td, q0, iters=250, lam=0.05,
             tol=1e-9, stall_at=60, stall_tol=1e-3):
    """강의에서 다룬 반복식 그대로. 관절 한계는 여기서 걸지 않는다 —
    한계 때문에 못 쓰는 해가 몇 개인지를 보는 것이 이 데모의 요지라서다.

    두 군데서 일찍 끊는다.

    ① 수렴하면 tol 에서 멈춘다. 예전 기준은 1e-11 이었는데 채택 기준이 1e-6 이라
       사실상 걸리지 않았고, 그래서 이미 수렴한 시드도 250 회를 다 돌았다.
    ② stall_at 회까지 stall_tol 밖에 있으면 접는다. 무작위 시드의 3 할이 끝내
       수렴하지 못하는데 그것들이 시간의 절반을 먹고 있었다.

    무작위 6 자세에서 대조했다 — 이렇게 끊고 시드를 40 개만 써도 끊지 않고
    400 개를 쓴 것과 **해집합이 같다.** 시간은 4.9 s -> 0.4 s (자세 5 는 41.6 s -> 3.8 s)."""
    q = q0.copy()
    for k in range(iters):
        e = pose_error(chain.fk(q), Td)
        err = np.linalg.norm(e)
        if err < tol:
            break
        if k == stall_at and err > stall_tol:
            return q, float(err)          # 가망이 없다
        J = chain.jacobian(q)
        dq = J.T @ np.linalg.solve(J @ J.T + lam ** 2 * np.eye(6), e)
        q = q + np.clip(dq, -0.2, 0.2)
    return q, float(np.linalg.norm(pose_error(chain.fk(q), Td)))


def wrap(q):
    return (q + np.pi) % (2.0 * np.pi) - np.pi


class IkBranches(Node):

    def __init__(self):
        super().__init__("ik_solutions")
        default_urdf = (get_package_share_directory("piper_description")
                        + "/urdf/piper_description.urdf")
        self.declare_parameter("urdf", default_urdf)
        self.declare_parameter("seeds", 80)
        self.declare_parameter("time_budget", 3.0)   # 넘으면 시드를 더 쓰지 않는다
        self.declare_parameter("frame", "base_link")   # 마커를 놓을 좌표계
        self.declare_parameter("alpha", 0.55)          # 유령 로봇의 불투명도
        # **기본은 감추기다.** 아무것도 없는 화면에서 시작해 초록 -> 빨강 순으로
        # 드러내는 진행이 이 데모의 장치다.
        self.declare_parameter("show_valid", False)
        self.declare_parameter("solve_on_start", True)
        # 목표를 어디서 잡나. 6개가 아니면 /joint_states 의 지금 자세를 쓴다.
        # (빈 리스트를 기본값으로 두면 rclpy 가 BYTE_ARRAY 로 추론해 버린다)
        self.declare_parameter("q", [0.0])

        self.chain = Chain(self.get_parameter("urdf").value)
        self.visuals = Visuals(self.get_parameter("urdf").value)
        self.latest = None
        self.q_ref = None
        self.solved_once = False
        self.valid = []
        self.invalid = []
        self.reveal = False      # 한계 밖 해를 화면에 낼지. 기본은 감춰둔다
        self.show_valid = bool(self.get_parameter("show_valid").value)

        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pub_valid = self.create_publisher(MarkerArray, "/ik_solutions/valid", qos)
        self.pub_invalid = self.create_publisher(MarkerArray, "/ik_solutions/invalid", qos)
        self.create_subscription(JointState, "/joint_states", self.on_joints, 10)
        self.srv = self.create_service(Trigger, "~/solve", self.on_solve)
        self.srv_reveal = self.create_service(SetBool, "~/reveal", self.on_reveal)
        self.srv_valid = self.create_service(SetBool, "~/show_valid", self.on_show_valid)

        # 지금 무엇이 보이는지를 래치해 알린다. 패널이 나중에 떠도, 터미널로 껐다
        # 켜도 버튼 라벨이 실제 상태를 따라온다.
        self.pub_shown_valid = self.create_publisher(Bool, "~/shown_valid", qos)
        self.pub_revealed = self.create_publisher(Bool, "~/revealed", qos)
        self.announce()

        q_param = list(self.get_parameter("q").value)
        self.fixed_q = np.array(q_param, dtype=float) if len(q_param) == 6 else None

        self.get_logger().info("staying up. to solve again:")
        self.get_logger().info("  ros2 service call /ik_solutions/solve std_srvs/srv/Trigger")
        self.get_logger().info('  (or the "Solve IK" button on the lecture panel)')

    # ------------------------------------------------------------------ 입력

    def on_joints(self, msg):
        index = {n: i for i, n in enumerate(msg.name)}
        if not all(n in index for n in ARM):
            return
        self.latest = np.array([msg.position[index[n]] for n in ARM])
        if not self.solved_once and bool(self.get_parameter("solve_on_start").value):
            self.solved_once = True
            self.run(self.target())

    def target(self):
        if self.fixed_q is not None:
            return self.fixed_q
        return self.latest

    def on_solve(self, _req, res):
        q = self.target()
        if q is None:
            res.success = False
            res.message = "no /joint_states received yet"
            return res
        n_all, n_valid = self.run(q)
        res.success = True
        res.message = f"{n_all} solutions / {n_valid} within joint limits"
        return res

    def announce(self):
        self.pub_shown_valid.publish(Bool(data=self.show_valid))
        self.pub_revealed.publish(Bool(data=self.reveal))

    def on_show_valid(self, req, res):
        """초록을 낼지 말지. 다시 풀지 않는다."""
        self.show_valid = bool(req.data)
        self.announce()
        res.success = True
        if self.q_ref is None:
            res.message = 'nothing solved yet. press "Solve IK" first'
            return res
        self.pub_valid.publish(
            self.to_markers(self.valid if self.show_valid else [], "ik_valid", VALID_RGB))
        res.message = ("showing usable solutions" if self.show_valid
                       else "hiding usable solutions")
        return res

    def on_reveal(self, req, res):
        """한계 밖 해를 화면에 낼지 말지. **다시 풀지 않는다** — 이미 찾아둔 것을
        다시 낼 뿐이라 즉시 끝난다."""
        self.reveal = bool(req.data)
        self.announce()
        res.success = True
        if self.q_ref is None:
            res.message = 'nothing solved yet. press "Solve IK" first'
            return res
        self.pub_invalid.publish(
            self.to_markers(self.invalid if self.reveal else [], "ik_invalid", INVALID_RGB))
        if not self.invalid:
            res.message = "there are no out-of-limit solutions"
        elif self.reveal:
            res.message = f"showing {len(self.invalid)} out-of-limit solutions"
        else:
            res.message = "hiding out-of-limit solutions"
        return res

    # ------------------------------------------------------------------ 본체

    def run(self, q_ref):
        # 새로 풀면 빨강은 다시 감춘다 — 「해가 몇 개일까」를 묻고 드러내는 것이
        # 이 데모의 장치라, 자세를 옮길 때마다 그 장치가 되감겨야 한다.
        # 초록은 건드리지 않는다. 그쪽은 연출이 아니라 보기 설정이다.
        self.reveal = False
        self.q_ref = wrap(np.asarray(q_ref, dtype=float).copy())
        Td = self.chain.fk(q_ref)
        self.get_logger().info("")
        self.get_logger().info(
            f"target: the TCP of the current pose. position {np.round(Td[:3, 3], 4).tolist()}")

        # 목표가 특이점이면 해가 낱개가 아니라 연속체가 된다 (손목 특이점에서는
        # joint4 와 joint6 의 합만 정해지고 각각은 자유롭다). 그때는 「몇 개」가 뜻이 없다.
        s6 = np.linalg.svd(self.chain.jacobian(q_ref), compute_uv=False)
        if s6[-1] < 1e-3:
            self.get_logger().warn(
                f"this pose is singular (6-DOF sigma_min = {s6[-1]:.2e}). "
                "the solutions form a continuum rather than discrete branches, "
                "so counting them is meaningless")
            self.get_logger().warn(
                "  move the arm off the singularity and run again - the MoveIt stack's "
                "default pose (all joints zero) IS the wrist singularity")

        seeds = int(self.get_parameter("seeds").value)
        budget = float(self.get_parameter("time_budget").value)
        rng = np.random.default_rng(0)
        # 기준 자세 자체가 해다 — 목표를 그 자세의 FK 로 잡았으니 당연하다.
        # 수치 탐색은 특이점이나 관절 한계 근처에서 이것을 놓치므로 먼저 넣어둔다.
        sols = [self.q_ref.copy()]
        t0 = time.monotonic()
        used = 0
        for _ in range(seeds):
            # 특이점 근처에서는 시드 하나가 250 회를 다 쓰기도 한다. 강의 중에
            # 버튼이 몇 십 초씩 물려 있지 않도록 시간으로도 끊는다.
            if budget > 0.0 and time.monotonic() - t0 > budget:
                break
            used += 1
            q0 = -np.pi + 2.0 * np.pi * rng.random(6)
            q, err = solve_ik(self.chain, Td, q0)
            if err > 1e-6:
                continue
            qn = wrap(q)
            if not any(np.max(np.abs(qn - t)) < 0.05 for t in sols):
                sols.append(qn)

        lo, hi = self.chain.lower, self.chain.upper
        valid, invalid = [], []
        for s in sols:
            (valid if np.all(s >= lo - 1e-6) and np.all(s <= hi + 1e-6) else invalid).append(s)

        self.valid, self.invalid = valid, invalid
        self.report(valid, invalid, lo, hi)
        self.get_logger().info(
            f"seeds {used}/{seeds} · {time.monotonic() - t0:.1f} s")
        if invalid and not self.reveal:
            self.get_logger().info(
                'out-of-limit solutions are not on screen yet - '
                'use "Show remaining solutions" on the panel')

        self.announce()
        self.pub_valid.publish(
            self.to_markers(valid if self.show_valid else [], "ik_valid", VALID_RGB))
        # 빨강 디스플레이는 rviz 에서 켜져 있다. 감추는 것은 빈 메시지로 한다 —
        # 그래야 체크박스를 마우스로 찾지 않고 버튼 하나로 연출이 된다.
        self.pub_invalid.publish(
            self.to_markers(invalid if self.reveal else [], "ik_invalid", INVALID_RGB))
        return len(sols), len(valid)

    def report(self, valid, invalid, lo, hi):
        self.get_logger().info(
            f"found {len(valid) + len(invalid)} solutions - "
            f"{len(valid)} within joint limits / {len(invalid)} outside")
        for s in valid:
            mark = "  <- current pose" if np.max(np.abs(s - self.q_ref)) < 1e-6 else ""
            self.get_logger().info(f"  [usable]       {np.round(s, 3).tolist()}{mark}")
        for s in invalid:
            bad = [f"joint{k + 1}={np.degrees(s[k]):+.0f}deg"
                   f"(limit {np.degrees(lo[k]):+.0f}~{np.degrees(hi[k]):+.0f})"
                   for k in range(6) if s[k] < lo[k] - 1e-6 or s[k] > hi[k] + 1e-6]
            self.get_logger().info(f"  [out of limit] {np.round(s, 3).tolist()}")
            self.get_logger().info(f"              {' · '.join(bad)}")

    def to_markers(self, sols, ns, rgb):
        """해 목록 -> 유령 로봇 한 벌씩. 링크마다 메시 Marker 하나다.

        맨 앞의 DELETEALL 이 지난번 것을 지운다. 그래서 해가 줄어도 잔상이 남지
        않고, **빈 목록을 주면 그대로 화면에서 사라진다** (감추기가 이것이다).
        """
        frame = self.get_parameter("frame").value

        arr = MarkerArray()
        clear = Marker()
        clear.header.frame_id = frame
        clear.action = Marker.DELETEALL
        # ns 를 **비워 둔다.** DELETEALL 은 원래 네임스페이스를 가리지 않고 다 지우는
        # 것이라, ns 를 채우면 rviz 가 디스플레이에 빨간 오류 표시를 낸다 (그리기는
        # 되는데 아이콘만 빨개져서 원인을 찾기 어렵다). 토픽이 갈려 있으므로
        # 디스플레이 하나에는 우리 마커뿐이고, 다 지워도 문제가 없다.
        clear.pose.orientation.w = 1.0
        arr.markers.append(clear)
        alpha = float(self.get_parameter("alpha").value)
        stamp = self.get_clock().now().to_msg()

        for i, sol in enumerate(sols):
            poses = self.visuals.link_poses(dict(zip(ARM, sol)))
            for k, link in enumerate(self.visuals.links):
                uri, T_visual, scale = self.visuals.mesh[link]
                T = poses[link] @ T_visual
                q = matrix_to_quaternion(T[:3, :3])

                m = Marker()
                m.header.frame_id = frame
                m.header.stamp = stamp
                m.ns = ns
                m.id = i * 100 + k
                m.type = Marker.MESH_RESOURCE
                m.action = Marker.ADD
                m.mesh_resource = uri
                m.pose.position.x, m.pose.position.y, m.pose.position.z = T[:3, 3]
                (m.pose.orientation.x, m.pose.orientation.y,
                 m.pose.orientation.z, m.pose.orientation.w) = q
                m.scale.x, m.scale.y, m.scale.z = scale
                m.color.r, m.color.g, m.color.b = (c / 255.0 for c in rgb)
                m.color.a = alpha
                arr.markers.append(m)
        return arr


def main():
    rclpy.init()
    node = IkBranches()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()

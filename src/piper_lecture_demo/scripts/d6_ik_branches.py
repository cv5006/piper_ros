#!/usr/bin/env python3
"""D6 — 같은 손끝 자세를 만드는 해가 여럿이다. 그런데 쓸 수 있는 것은 하나다. (M2 · M3-2)

팔이 있는 자리의 손끝 자세를 목표로 잡고, 그 자세를 만드는 관절값을 전부 찾는다.
찾는 방법은 강의에서 다룬 그대로다 — q <- q + J⁺(x_target - f(q)) 를 무작위 초기값에서
여러 번 돌리고, 수렴한 것들을 모아 중복을 지운다 (DLS 감쇠).

상주하면서 부를 때마다 다시 푼다. 팔을 옮기고 또 누르면 그 자리에서 다시 찾는다.

    ros2 run piper_lecture_demo d6_ik_branches.py          # 뜨면 한 번 풀고 대기
    ros2 service call /d6_ik_branches/solve std_srvs/srv/Trigger
    (또는 강의 패널의 「IK 해 찾기」 버튼)

두 토픽으로 낸다. rviz 의 Trajectory 디스플레이 둘이 각각 받는다.

    /ik_branches/valid     관절 한계 안에 드는 해   (초록)
    /ik_branches/invalid   한계 밖으로 밀려난 해     (빨강)

Show Trail 을 켜면 여러 자세가 한 화면에 겹쳐 보인다.

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

import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from moveit_msgs.msg import DisplayTrajectory, RobotState, RobotTrajectory
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger
from trajectory_msgs.msg import JointTrajectoryPoint

from piper_lecture_demo.kinematics import Chain

ARM = [f"joint{i}" for i in range(1, 7)]
ALL = [f"joint{i}" for i in range(1, 9)]


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


def solve_ik(chain, Td, q0, iters=250, lam=0.05):
    """강의에서 다룬 반복식 그대로. 관절 한계는 여기서 걸지 않는다 —
    한계 때문에 못 쓰는 해가 몇 개인지를 보는 것이 이 데모의 요지라서다."""
    q = q0.copy()
    for _ in range(iters):
        e = pose_error(chain.fk(q), Td)
        if np.linalg.norm(e) < 1e-11:
            break
        J = chain.jacobian(q)
        dq = J.T @ np.linalg.solve(J @ J.T + lam ** 2 * np.eye(6), e)
        q = q + np.clip(dq, -0.2, 0.2)
    return q, float(np.linalg.norm(pose_error(chain.fk(q), Td)))


def wrap(q):
    return (q + np.pi) % (2.0 * np.pi) - np.pi


class IkBranches(Node):

    def __init__(self):
        super().__init__("d6_ik_branches")
        default_urdf = (get_package_share_directory("piper_description")
                        + "/urdf/piper_description.urdf")
        self.declare_parameter("urdf", default_urdf)
        self.declare_parameter("seeds", 400)
        self.declare_parameter("state_seconds", 1.0)   # 자세 하나를 보여주는 시간
        self.declare_parameter("solve_on_start", True)
        # 목표를 어디서 잡나. 6개가 아니면 /joint_states 의 지금 자세를 쓴다.
        # (빈 리스트를 기본값으로 두면 rclpy 가 BYTE_ARRAY 로 추론해 버린다)
        self.declare_parameter("q", [0.0])

        self.chain = Chain(self.get_parameter("urdf").value)
        self.latest = None
        self.q_ref = None
        self.solved_once = False

        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pub_valid = self.create_publisher(DisplayTrajectory, "/ik_branches/valid", qos)
        self.pub_invalid = self.create_publisher(DisplayTrajectory, "/ik_branches/invalid", qos)
        self.create_subscription(JointState, "/joint_states", self.on_joints, 10)
        self.srv = self.create_service(Trigger, "~/solve", self.on_solve)

        q_param = list(self.get_parameter("q").value)
        self.fixed_q = np.array(q_param, dtype=float) if len(q_param) == 6 else None

        self.get_logger().info("상주한다. 다시 풀려면:")
        self.get_logger().info("  ros2 service call /d6_ik_branches/solve std_srvs/srv/Trigger")
        self.get_logger().info("  (또는 강의 패널의 「IK 해 찾기」 버튼)")

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
            res.message = "/joint_states 를 아직 못 받았다"
            return res
        n_all, n_valid = self.run(q)
        res.success = True
        res.message = f"해 {n_all} 개 / 관절 한계 안 {n_valid} 개"
        return res

    # ------------------------------------------------------------------ 본체

    def run(self, q_ref):
        self.q_ref = wrap(np.asarray(q_ref, dtype=float).copy())
        Td = self.chain.fk(q_ref)
        self.get_logger().info("")
        self.get_logger().info(
            f"목표: 지금 자세의 손끝. 위치 {np.round(Td[:3, 3], 4).tolist()}")

        # 목표가 특이점이면 해가 낱개가 아니라 연속체가 된다 (손목 특이점에서는
        # joint4 와 joint6 의 합만 정해지고 각각은 자유롭다). 그때는 「몇 개」가 뜻이 없다.
        s6 = np.linalg.svd(self.chain.jacobian(q_ref), compute_uv=False)
        if s6[-1] < 1e-3:
            self.get_logger().warn(
                f"이 자세는 특이점이다 (6자유도 sigma_min = {s6[-1]:.2e}). "
                "해가 낱개가 아니라 연속체라 개수는 뜻이 없다")
            self.get_logger().warn(
                "  팔을 특이점에서 떼어놓고 다시 실행할 것 — MoveIt 스택의 기본 자세(영자세)가 "
                "바로 손목 특이점이다")

        seeds = int(self.get_parameter("seeds").value)
        rng = np.random.default_rng(0)
        # 기준 자세 자체가 해다 — 목표를 그 자세의 FK 로 잡았으니 당연하다.
        # 수치 탐색은 특이점이나 관절 한계 근처에서 이것을 놓치므로 먼저 넣어둔다.
        sols = [self.q_ref.copy()]
        for _ in range(seeds):
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

        self.report(valid, invalid, lo, hi)
        self.pub_valid.publish(self.to_display(valid, q_ref))
        self.pub_invalid.publish(self.to_display(invalid, q_ref))
        return len(sols), len(valid)

    def report(self, valid, invalid, lo, hi):
        self.get_logger().info(
            f"찾은 해 {len(valid) + len(invalid)} 개 — "
            f"관절 한계 안 {len(valid)} 개 / 밖 {len(invalid)} 개")
        for s in valid:
            mark = "  <- 지금 자세" if np.max(np.abs(s - self.q_ref)) < 1e-6 else ""
            self.get_logger().info(f"  [쓸 수 있다] {np.round(s, 3).tolist()}{mark}")
        for s in invalid:
            bad = [f"joint{k + 1}={np.degrees(s[k]):+.0f}deg"
                   f"(한계 {np.degrees(lo[k]):+.0f}~{np.degrees(hi[k]):+.0f})"
                   for k in range(6) if s[k] < lo[k] - 1e-6 or s[k] > hi[k] + 1e-6]
            self.get_logger().info(f"  [한계 밖]   {np.round(s, 3).tolist()}")
            self.get_logger().info(f"              {' · '.join(bad)}")

    def to_display(self, sols, q_ref):
        msg = DisplayTrajectory()
        msg.model_id = "piper"
        start = RobotState()
        start.joint_state.name = list(ALL)
        start.joint_state.position = [float(v) for v in q_ref] + [0.0, 0.0]
        msg.trajectory_start = start
        if not sols:
            return msg

        step = float(self.get_parameter("state_seconds").value)
        traj = RobotTrajectory()
        traj.joint_trajectory.joint_names = list(ARM)
        for i, s in enumerate(sols):
            pt = JointTrajectoryPoint()
            pt.positions = [float(v) for v in s]
            t = (i + 1) * step
            pt.time_from_start.sec = int(t)
            pt.time_from_start.nanosec = int((t % 1.0) * 1e9)
            traj.joint_trajectory.points.append(pt)
        msg.trajectory.append(traj)
        return msg


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

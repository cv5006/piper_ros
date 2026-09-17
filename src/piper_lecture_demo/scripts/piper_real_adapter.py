#!/usr/bin/env python3
"""MoveIt 스택을 실물 팔에 물린다 — mock_components 자리를 대신하는 어댑터.

    ros2 launch piper_lecture_demo moveit_demo.launch.py real:=true

MoveIt 이 기대하는 것과 piper 드라이버가 가진 것 사이가 비어 있다. 이 노드가 그 사이다.

  MoveIt 이 부르는 것                        piper 드라이버가 가진 것
  ─────────────────────────────────────      ─────────────────────────────────
  /arm_controller/follow_joint_trajectory    joint_ctrl_single  (JointState 하나 = 목표 자세)
  /gripper_controller/follow_joint_trajectory
  /joint_states (상태)                       joint_states_feedback (JointState)

하는 일은 셋이다.

  ① 상태 다리 — joint_states_feedback -> /joint_states
     드라이버는 일곱째 관절을 **'gripper'** 라고 부르는데 로봇 모델은 joint7 이다.
     이름을 바꾸고 joint8 도 채운다 (URDF 에 mimic 이 없어 move_group 이
     「Missing joint8」을 1 초에 한 번씩 내던 것도 여기서 없어진다).

  ② 명령 다리 — joint_ctrl_single 로 목표 자세를 쏜다
     **velocity[6] 에 속도(%)를 반드시 채운다.** 비워 두면 드라이버가
     MotionCtrl_2(..., 100) 으로 전속을 건다.

  ③ 궤적 실행 — FollowJointTrajectory 액션 서버 둘
     드라이버의 JointCtrl 은 메시지 하나당 목표 하나이고 보간이 없다. 그래서
     경유점 사이를 여기서 보간해 일정 주기로 쏘는 것이 곧 실행이다.

⚠ 드라이버를 **remap 없이** 띄울 것. piper 의 start_single_piper.launch.py 는
  joint_ctrl_single 을 /joint_states 로 remap 하는데, 그 상태로 이 노드를 켜면
  ①이 내보낸 상태가 그대로 ②의 명령으로 되돌아가 루프가 된다. real:=true 런치는
  드라이버 노드를 remap 없이 직접 띄운다.

⚠ 실물이 붙어 있으면 이 노드는 진짜 팔을 움직인다. 처음 켤 때 파라미터를 확인할 것.

    speed_percent   드라이버에 넘기는 속도 (1~100). 기본 20
    max_step_rad    첫 경유점이 지금 자세에서 이만큼 넘게 떨어져 있으면 goal 을
                    거절한다. 계단 명령을 막는 장치다. 기본 0.35 rad (20도)
"""

import math
import threading

import rclpy
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import JointState

from piper_msgs.srv import Enable

ARM = [f"joint{i}" for i in range(1, 7)]
GRIPPER = "joint7"
# 드라이버가 쓰는 이름. 일곱째만 다르다.
DRIVER_NAMES = ARM + ["gripper"]


def lerp(a, b, t):
    return a + (b - a) * t


class PiperAdapter(Node):

    def __init__(self):
        super().__init__("piper_real_adapter")

        self.declare_parameter("speed_percent", 20)
        self.declare_parameter("max_step_rad", 0.35)
        self.declare_parameter("command_rate", 100.0)
        self.declare_parameter("gripper_effort", 1.0)
        # 드라이버가 상태를 주기 전에는 아무것도 하지 않는다
        self.declare_parameter("state_timeout", 5.0)
        self.declare_parameter("auto_enable", True)

        self.speed = int(self.get_parameter("speed_percent").value)
        self.max_step = float(self.get_parameter("max_step_rad").value)
        self.rate = float(self.get_parameter("command_rate").value)

        self.q = None            # 팔의 실측 자세 (6)
        self.grip = 0.0          # 그리퍼 실측 열림 [m]
        self.lock = threading.Lock()

        group = ReentrantCallbackGroup()

        # ① 상태 다리
        self.pub_state = self.create_publisher(JointState, "/joint_states", 10)
        self.create_subscription(JointState, "joint_states_feedback",
                                 self.on_feedback, 10, callback_group=group)

        # ② 명령 다리
        self.pub_cmd = self.create_publisher(JointState, "joint_ctrl_single", 10)

        # 드라이버 enable
        self.enable_cli = self.create_client(Enable, "enable_srv",
                                             callback_group=group)

        # ③ 궤적 실행
        self.srv_arm = ActionServer(
            self, FollowJointTrajectory, "/arm_controller/follow_joint_trajectory",
            execute_callback=self.execute_arm, goal_callback=self.accept,
            cancel_callback=self.accept_cancel, callback_group=group)
        self.srv_grip = ActionServer(
            self, FollowJointTrajectory, "/gripper_controller/follow_joint_trajectory",
            execute_callback=self.execute_gripper, goal_callback=self.accept,
            cancel_callback=self.accept_cancel, callback_group=group)

        self.get_logger().info(
            f"실물 어댑터. 속도 {self.speed} % · 명령 {self.rate:.0f} Hz · "
            f"계단 한계 {math.degrees(self.max_step):.0f} 도")
        self.get_logger().warn("실물이 붙어 있으면 이 노드는 진짜 팔을 움직인다.")

    # ------------------------------------------------------------------ ①

    def on_feedback(self, msg):
        index = {n: i for i, n in enumerate(msg.name)}
        if not all(n in index for n in DRIVER_NAMES[:6]):
            return
        with self.lock:
            self.q = [msg.position[index[n]] for n in ARM]
            if "gripper" in index:
                self.grip = msg.position[index["gripper"]]
            q, grip = list(self.q), self.grip

        out = JointState()
        out.header.stamp = msg.header.stamp
        # joint8 은 반대쪽 손가락이다. URDF 에 mimic 이 없어 드라이버도 내지 않으므로
        # 여기서 대칭으로 채운다. 그래야 로봇 모델의 여덟 관절이 다 찬다.
        out.name = ARM + [GRIPPER, "joint8"]
        out.position = q + [grip, -grip]
        if len(msg.velocity) >= 6:
            out.velocity = list(msg.velocity[:6]) + [0.0, 0.0]
        if len(msg.effort) >= 7:
            out.effort = list(msg.effort[:6]) + [msg.effort[6], msg.effort[6]]
        self.pub_state.publish(out)

    def state(self):
        with self.lock:
            return (list(self.q) if self.q is not None else None), self.grip

    # ------------------------------------------------------------------ ②

    def send(self, q, grip):
        """드라이버가 알아듣는 형태로 목표 하나를 쏜다.

        velocity[6] 을 비우면 드라이버가 전속(100 %)을 건다. 반드시 채운다."""
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = DRIVER_NAMES
        msg.position = list(q) + [float(grip)]
        msg.velocity = [0.0] * 6 + [float(self.speed)]
        msg.effort = [0.0] * 6 + [float(self.get_parameter("gripper_effort").value)]
        self.pub_cmd.publish(msg)

    def ensure_enabled(self):
        if not bool(self.get_parameter("auto_enable").value):
            return True
        if not self.enable_cli.service_is_ready():
            self.get_logger().warn("enable_srv 가 없다. 드라이버가 떠 있나?")
            return False
        fut = self.enable_cli.call_async(Enable.Request(enable_request=True))
        end = self.get_clock().now().nanoseconds + 3e9
        while not fut.done() and self.get_clock().now().nanoseconds < end:
            pass
        res = fut.result()
        if res is None:
            self.get_logger().warn("enable_srv 응답이 없다")
            return False
        return True

    # ------------------------------------------------------------------ ③

    def accept(self, _goal):
        return GoalResponse.ACCEPT

    def accept_cancel(self, _goal):
        return CancelResponse.ACCEPT

    def reject(self, handle, why):
        self.get_logger().warn(why)
        handle.abort()
        res = FollowJointTrajectory.Result()
        res.error_code = FollowJointTrajectory.Result.INVALID_GOAL
        res.error_string = why
        return res

    def execute_arm(self, handle):
        traj = handle.request.trajectory
        q_now, _ = self.state()
        if q_now is None:
            return self.reject(handle, "드라이버 상태를 아직 못 받았다")
        if not traj.points:
            handle.succeed()
            return FollowJointTrajectory.Result()

        try:
            order = [traj.joint_names.index(n) for n in ARM]
        except ValueError:
            return self.reject(handle, f"궤적에 팔 관절이 다 없다: {traj.joint_names}")

        first = [traj.points[0].positions[i] for i in order]
        step = max(abs(a - b) for a, b in zip(first, q_now))
        if step > self.max_step:
            return self.reject(
                handle,
                f"첫 경유점이 지금 자세에서 {math.degrees(step):.0f} 도 떨어져 있다 "
                f"(한계 {math.degrees(self.max_step):.0f} 도). 계단 명령을 막는다")

        if not self.ensure_enabled():
            return self.reject(handle, "드라이버를 enable 하지 못했다")

        return self.stream(handle, traj, order, gripper=False)

    def execute_gripper(self, handle):
        traj = handle.request.trajectory
        if GRIPPER not in traj.joint_names:
            return self.reject(handle, f"그리퍼 궤적이 아니다: {traj.joint_names}")
        if not traj.points:
            handle.succeed()
            return FollowJointTrajectory.Result()
        if not self.ensure_enabled():
            return self.reject(handle, "드라이버를 enable 하지 못했다")
        return self.stream(handle, traj, [traj.joint_names.index(GRIPPER)], gripper=True)

    def stream(self, handle, traj, order, gripper):
        """경유점 사이를 보간해 일정 주기로 쏜다. 이것이 곧 실행이다."""
        def at(point):
            return [point.positions[i] for i in order]

        def secs(point):
            return point.time_from_start.sec + point.time_from_start.nanosec * 1e-9

        pts = traj.points
        total = secs(pts[-1])
        period = 1.0 / self.rate
        feedback = FollowJointTrajectory.Feedback()
        feedback.joint_names = list(ARM) if not gripper else [GRIPPER]

        t = 0.0
        k = 0
        while t <= total + 1e-9:
            if not handle.is_active:
                return FollowJointTrajectory.Result()
            if handle.is_cancel_requested:
                q_now, grip_now = self.state()
                if q_now is not None and not gripper:
                    self.send(q_now, grip_now)          # 그 자리에 세운다
                handle.canceled()
                self.get_logger().info("궤적 취소 — 그 자리에 세웠다")
                return FollowJointTrajectory.Result()

            while k + 1 < len(pts) and secs(pts[k + 1]) < t:
                k += 1
            if k + 1 < len(pts):
                t0, t1 = secs(pts[k]), secs(pts[k + 1])
                a = 0.0 if t1 <= t0 else (t - t0) / (t1 - t0)
                target = [lerp(x, y, a) for x, y in zip(at(pts[k]), at(pts[k + 1]))]
            else:
                target = at(pts[-1])

            q_now, grip_now = self.state()
            if gripper:
                self.send(q_now if q_now else [0.0] * 6, target[0])
            else:
                self.send(target, grip_now)

            feedback.desired.positions = list(target)
            if q_now is not None:
                feedback.actual.positions = [grip_now] if gripper else list(q_now)
            handle.publish_feedback(feedback)

            self.get_clock().sleep_for(rclpy.duration.Duration(seconds=period))
            t += period

        handle.succeed()
        res = FollowJointTrajectory.Result()
        res.error_code = FollowJointTrajectory.Result.SUCCESSFUL
        return res


def main():
    rclpy.init()
    node = PiperAdapter()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()

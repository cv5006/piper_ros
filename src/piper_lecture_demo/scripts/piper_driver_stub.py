#!/usr/bin/env python3
"""piper 드라이버의 **토픽 인터페이스만** 흉내 낸다. 실물 없이 어댑터를 시험한다.

    ros2 run piper_lecture_demo piper_driver_stub.py

    구독   joint_ctrl_single      목표 자세 하나
    발행   joint_states_feedback  joint1..joint6 + 'gripper'
    서비스 enable_srv

⚠ 확인되는 것 — 궤적 분해 · 속도 필드 · 이름 변환 · 되먹임 루프 부재 · Plan & Execute
⚠ 확인 안 되는 것 — CAN 타이밍 · 추종 오차 · 관절 한계 거동 · 그리퍼 힘

팔은 1차 지연으로 따라가는 시늉만 한다.
"""

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import JointState

from piper_msgs.srv import Enable

NAMES = [f"joint{i}" for i in range(1, 7)] + ["gripper"]
START = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]


class Stub(Node):

    def __init__(self):
        super().__init__("piper_driver_stub")
        self.declare_parameter("tau", 0.08)        # 따라가는 시정수 [s]
        self.declare_parameter("rate", 200.0)      # 진짜 드라이버와 같은 200 Hz
        self.declare_parameter("start", START)

        self.q = list(self.get_parameter("start").value)
        self.target = list(self.q)
        self.enabled = False
        self.speed = None
        self.n_cmd = 0

        self.pub = self.create_publisher(JointState, "joint_states_feedback", 10)
        self.create_subscription(JointState, "joint_ctrl_single", self.on_cmd, 10)
        self.create_service(Enable, "enable_srv", self.on_enable)

        rate = float(self.get_parameter("rate").value)
        self.dt = 1.0 / rate
        self.create_timer(self.dt, self.tick)
        self.create_timer(2.0, self.report)

        self.get_logger().info("piper driver stub. this is NOT the real hardware.")

    def on_enable(self, req, res):
        self.enabled = bool(req.enable_request)
        res.enable_response = self.enabled
        self.get_logger().info(f"enable -> {self.enabled}")
        return res

    def on_cmd(self, msg):
        index = {n: i for i, n in enumerate(msg.name)}
        for k, name in enumerate(NAMES):
            if name in index:
                self.target[k] = msg.position[index[name]]
        self.n_cmd += 1
        # 진짜 드라이버는 velocity 가 비었거나 전부 0 이면 전속(100 %)을 건다.
        # 어댑터가 그 자리를 채우는지 여기서 본다.
        self.speed = msg.velocity[6] if len(msg.velocity) >= 7 else None

    def tick(self):
        a = min(1.0, self.dt / max(1e-3, float(self.get_parameter("tau").value)))
        self.q = [q + (t - q) * a for q, t in zip(self.q, self.target)]

        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(NAMES)
        msg.position = list(self.q)
        msg.velocity = [0.0] * 6
        msg.effort = [0.0] * 7
        self.pub.publish(msg)

    def report(self):
        if self.n_cmd:
            speed = ("none (the real driver would run at full speed)"
                     if self.speed is None else f"{self.speed:.0f} %")
            self.get_logger().info(
                f"{self.n_cmd} commands received · speed field {speed} · "
                f"enable={self.enabled}")
            self.n_cmd = 0


def main():
    rclpy.init()
    node = Stub()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()

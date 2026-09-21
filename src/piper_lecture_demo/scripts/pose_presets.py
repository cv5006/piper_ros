#!/usr/bin/env python3
"""자세 드라이버 — 프리셋을 /joint_states 로 내보낸다. (M3-3 · M3-4)

**/joint_states 의 유일한 발행자다.** 슬라이더 GUI 는 /gui_joint_states 로 비켜
발행하고 이 노드가 받아 넘긴다. 발행자가 둘이면 서로 덮어쓴다 (명령 토픽이다).

    슬라이더 -> gui 모드 / goto_pose -> preset 모드 (보간)

GUI 에 source_list 로 /joint_states 를 물려 두어 프리셋 뒤에 슬라이더를 잡아도 안 튄다.

근거는 이 레포 URDF 로 계산한 값이다 (기준점 TCP).

    preset    cond_v  sigma_min_6d
    good         2.8      0.104     잘 움직인다
    stretch     10.5      0.0075    팔을 편다 — 원반
    elbow       77.4      0.000050  joint3=-2.85 + joint5=0 — 선분
    wrist        3.2      0.000057  ⚠ 둥근데 6자유도가 죽었다
    home         7.8      0.000061  ⚠ 영자세가 곧 손목 특이점
    shoulder     5.1      0.000076  손목 중심이 joint1 축 위
    slow         6.5      0.0238    ⚠ elbow 와 w 가 같은데 특이점이 아니다

⚠ 실물을 연결하지 않은 상태로 쓸 것.
"""

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String

NAMES = [f"joint{i}" for i in range(1, 9)]

# 잘 움직이는 자세. 기동 시 여기서 시작한다.
GOOD = [0.0, 1.70, -1.38, 0.0, 0.50, 0.0]

PRESETS = {
    "good": GOOD,
    "stretch": [0.0, 2.90, -2.90, 0.0, 0.50, 0.0],
    "elbow": [0.0, 2.85, -2.85, 0.0, 0.0, 0.0],
    "wrist": [0.0, 1.20, -1.00, 0.30, 0.0, 0.0],
    "home": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    "shoulder": [1.566, 0.507, -0.768, -0.925, -1.021, -0.565],
    # elbow 와 「부피 w」가 1% 이내로 같은데 모양은 전혀 다른 자세. 둘을 붙여 보이면
    # 「w 가 작다 != 특이점에 가깝다」가 한 화면에서 끝난다.
    "slow": [1.183, 0.613, -0.167, -1.049, 1.040, 0.508],
}

GOTO_TOPIC = "~/goto"       # 사설 이름 — /pose_presets/goto 가 된다
GUI_TOPIC = "/gui_joint_states"   # 슬라이더 GUI 가 비켜서 발행하는 자리
GUI_MOVED = 0.02            # GUI 값이 이만큼(rad) 「변하면」 사람이 슬라이더를 만진 것


class PoseDriver(Node):

    def __init__(self):
        super().__init__("pose_presets")
        self.declare_parameter("preset", "good")
        self.declare_parameter("duration", 4.0)
        self.declare_parameter("loop", False)

        self.duration = max(0.1, float(self.get_parameter("duration").value))
        self.loop = bool(self.get_parameter("loop").value)

        self.q = np.array(GOOD, dtype=float)      # 지금 내보내는 값
        self.start = self.q.copy()
        self.target = self.q.copy()
        self.t = self.duration                    # 기동 직후는 이동 없음
        self.dt = 0.02
        self.mode = "gui"                         # gui | preset
        self.gui_q = None                         # 슬라이더에서 마지막으로 받은 값
        self.motion_done = True                   # 프리셋 보간이 끝났나

        self.pub = self.create_publisher(JointState, "/joint_states", 10)
        self.create_subscription(String, GOTO_TOPIC, self.on_goto, 10)
        self.create_subscription(JointState, GUI_TOPIC, self.on_gui, 10)
        self.create_timer(self.dt, self.tick)
        self.add_on_set_parameters_callback(self.on_param)

        self.get_logger().info("pose driver up. publishing /joint_states")
        self.get_logger().info(f"  presets: {' '.join(sorted(PRESETS))}")
        self.get_logger().info("  change pose: ros2 run piper_lecture_demo goto_pose.py <name>")
        first = str(self.get_parameter("preset").value)
        if first != "good":
            self.goto(first)

    # ------------------------------------------------------------ 명령 받기

    def on_goto(self, msg):
        self.goto(msg.data.strip())

    def on_gui(self, msg):
        index = {n: i for i, n in enumerate(msg.name)}
        q = self.gui_q.copy() if self.gui_q is not None else self.q.copy()
        for k in range(6):
            name = f"joint{k + 1}"
            if name in index:
                q[k] = msg.position[index[name]]

        # 「현재 자세와 다르다」가 아니라 「GUI 값이 변했다」로 판정한다.
        # 전자로 하면 GUI 가 /joint_states 를 못 따라올 때 프리셋이 곧바로 되돌려진다.
        # 후자는 GUI 가 가만히 있으면 아무 일도 일어나지 않아 안전하다.
        if (self.mode == "preset" and self.motion_done and self.gui_q is not None
                and np.max(np.abs(q - self.gui_q)) > GUI_MOVED):
            self.mode = "gui"
            self.get_logger().info("slider moved -> gui mode")
        self.gui_q = q

    def on_param(self, params):
        from rcl_interfaces.msg import SetParametersResult
        for p in params:
            if p.name == "preset":
                self.goto(str(p.value))
            elif p.name == "duration":
                self.duration = max(0.1, float(p.value))
            elif p.name == "loop":
                self.loop = bool(p.value)
        return SetParametersResult(successful=True)

    def goto(self, name):
        if name not in PRESETS:
            self.get_logger().error(
                f"'{name}' is not a known preset. available: {' '.join(sorted(PRESETS))}")
            return
        self.start = self.q.copy()
        self.target = np.array(PRESETS[name], dtype=float)
        self.t = 0.0
        self.mode = "preset"
        self.get_logger().info(
            f"-> {name}  {np.round(self.target, 3).tolist()}  (over {self.duration:.0f} s)")

    # ------------------------------------------------------------ 내보내기

    def tick(self):
        if self.mode == "preset":
            phase = min(self.t / self.duration, 1.0)
            a = 0.5 * (1.0 - np.cos(np.pi * phase))   # 양끝에서 부드럽게
            self.q = self.start + (self.target - self.start) * a
            self.t += self.dt

            if phase >= 1.0 and self.loop:
                # 왕복 — 끝에 닿으면 출발점과 목표를 맞바꾼다
                self.start, self.target = self.target.copy(), self.start.copy()
                self.t = 0.0
            # 슬라이더 감지는 보간이 끝난 뒤에만 한다. 움직이는 동안은 GUI 가
            # 뒤따라오느라 값이 계속 바뀌므로 사람이 만진 것과 구분되지 않는다.
            self.motion_done = phase >= 1.0
        elif self.gui_q is not None:
            self.q = self.gui_q.copy()

        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = NAMES
        msg.position = self.q.tolist() + [0.0, 0.0]
        self.pub.publish(msg)


def main():
    rclpy.init()
    node = PoseDriver()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()

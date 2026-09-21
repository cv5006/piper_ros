#!/usr/bin/env python3
"""강의 패널 — 데모를 버튼으로 돌린다. 터미널을 치지 않기 위한 것이다.

    ros2 run piper_lecture_demo lecture_panel.py

묶음 셋. 순서가 강의 진행 순서다.

    Manipulability & poses   타원체 · 자세 프리셋   URDF 스택
    IK solutions             해 전수 탐색           두 스택
    Obstacles                planning scene         MoveIt 스택

**떠 있지 않은 스택의 묶음은 회색으로 잠긴다** (1초마다 서비스 확인).
**일은 항상 딴 스레드에서.** 콜백에서 기다리면 창이 얼어붙는다.
"""

import signal
import sys
import threading
import time

import rclpy
from PyQt5.QtCore import Qt, QTimer, pyqtSignal
from PyQt5.QtWidgets import (QApplication, QGridLayout, QGroupBox, QLabel,
                             QPushButton, QVBoxLayout, QWidget)
from geometry_msgs.msg import Pose
from moveit_msgs.msg import CollisionObject, PlanningScene
from moveit_msgs.srv import ApplyPlanningScene
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from shape_msgs.msg import SolidPrimitive
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, String
from std_srvs.srv import SetBool, Trigger

PRESETS = ["good", "stretch", "elbow", "wrist", "slow", "home", "shoulder"]
# 버튼에 이름만 있으면 무엇을 보여주는 자세인지 알 수 없다. 툴팁으로 붙인다
# (goto_pose.py 의 설명과 같은 내용).
PRESET_HINT = {
    "good": "well-conditioned pose. the ellipsoid is round",
    "stretch": "arm extended. the ellipsoid becomes a disc",
    "elbow": "elbow + wrist singularity. the ellipsoid becomes a line segment",
    "wrist": "wrist singularity. the ellipsoid stays round but the 6-DOF rank dies",
    "slow": "uniformly slow pose. same w as elbow, yet not a singularity",
    "home": "all joints zero. this robot's zero pose IS the wrist singularity",
    "shoulder": "shoulder singularity. the wrist centre sits on the joint1 axis",
}
OBSTACLE_ID = "lecture_post"
OBSTACLE_FRAME = "world"
OBSTACLE_XYZ = (0.39, 0.0, 0.25)
OBSTACLE_SIZE = (0.06, 0.06, 0.50)


class Backend(Node):
    """버튼이 시키는 일을 실제로 하는 쪽. GUI 와 분리해 둔다."""

    def __init__(self):
        super().__init__("lecture_panel")
        self.goto_pub = self.create_publisher(String, "/pose_presets/goto", 10)
        self.ik_cli = self.create_client(Trigger, "/ik_solutions/solve")
        self.reveal_cli = self.create_client(SetBool, "/ik_solutions/reveal")
        self.scene_cli = self.create_client(ApplyPlanningScene, "/apply_planning_scene")
        self.ellipse_cli = self.create_client(SetBool, "/manipulability/show")
        self.valid_cli = self.create_client(SetBool, "/ik_solutions/show_valid")
        # 시각화는 전부 감추기가 기본이다. 실제 상태는 아래 래치 토픽으로 맞춘다.
        self.revealed = False
        self.valid_on = False
        self.ellipse_on = False
        self.on_state = None          # GUI 가 라벨을 다시 그리도록 걸어 두는 자리
        # d1 이 래치해 두는 상태. 터미널로 껐거나 ellipsoid:=false 로 떴어도 맞춘다.
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(Bool, "/manipulability/shown", self.on_shown, latched)
        self.create_subscription(Bool, "/ik_solutions/shown_valid",
                                 self.on_shown_valid, latched)
        self.create_subscription(Bool, "/ik_solutions/revealed",
                                 self.on_revealed, latched)

    def wait(self, fut, timeout):
        """executor 가 딴 스레드에서 돌고 있으므로 여기서는 기다리기만 한다.

        spin_until_future_complete 를 쓰면 이 노드를 두 스레드가 같이 돌리게 된다."""
        end = time.monotonic() + timeout
        while not fut.done() and time.monotonic() < end:
            time.sleep(0.02)
        return fut.result()

    # ------------------------------------------------------- 자세 프리셋

    def goto(self, name):
        if self.goto_pub.get_subscription_count() == 0:
            return f"no pose driver ({name} ignored). is the URDF demo stack up?"
        self.goto_pub.publish(String(data=name))
        return f"pose -> {name}"

    # -------------------------------------------------------- ik_solutions

    def solve_ik(self):
        if not self.ik_cli.service_is_ready():
            return "ik_solutions is missing. bring that node up first"
        # 새로 풀면 빨강이 다시 감춰지는데, 그것은 노드가 알려준다 (~/revealed).
        # 실측 약 1.5 초다. 60 초를 주면 노드가 응답하지 않을 때 창이 그만큼
        # 먹통으로 보인다 (버튼이 전부 잠긴다).
        res = self.wait(self.ik_cli.call_async(Trigger.Request()), 20.0)
        return f"IK: {res.message}" if res else "no response from IK"

    def reveal(self, on):
        if not self.reveal_cli.service_is_ready():
            return "ik_solutions is missing. bring that node up first"
        res = self.wait(self.reveal_cli.call_async(SetBool.Request(data=on)), 10.0)
        if res is None:
            return "no response"
        self.revealed = on
        return res.message

    def show_valid(self, on):
        if not self.valid_cli.service_is_ready():
            return "ik_solutions is missing. bring that node up first"
        res = self.wait(self.valid_cli.call_async(SetBool.Request(data=on)), 10.0)
        if res is None:
            return "no response"
        self.valid_on = on
        return res.message

    def on_shown(self, msg):
        self.ellipse_on = bool(msg.data)
        self.notify()

    def on_shown_valid(self, msg):
        self.valid_on = bool(msg.data)
        self.notify()

    def on_revealed(self, msg):
        self.revealed = bool(msg.data)
        self.notify()

    def notify(self):
        if self.on_state:
            self.on_state()

    # ------------------------------------------------ manipulability 타원체

    def ellipsoid(self, on):
        if not self.ellipse_cli.service_is_ready():
            return "manipulability is missing. is the URDF demo stack up?"
        res = self.wait(self.ellipse_cli.call_async(SetBool.Request(data=on)), 10.0)
        if res is None:
            return "no response"
        self.ellipse_on = on
        return res.message

    # ------------------------------------------------------------ obstacle

    def obstacle(self, add):
        if not self.scene_cli.service_is_ready():
            return "move_group is missing. is the MoveIt stack up?"
        obj = CollisionObject()
        obj.header.frame_id = OBSTACLE_FRAME
        obj.id = OBSTACLE_ID
        obj.operation = CollisionObject.ADD if add else CollisionObject.REMOVE
        if add:
            prim = SolidPrimitive()
            prim.type = SolidPrimitive.BOX
            prim.dimensions = list(OBSTACLE_SIZE)
            pose = Pose()
            pose.position.x, pose.position.y, pose.position.z = OBSTACLE_XYZ
            pose.orientation.w = 1.0
            obj.primitives.append(prim)
            obj.primitive_poses.append(pose)
        req = ApplyPlanningScene.Request()
        req.scene = PlanningScene()
        req.scene.is_diff = True
        req.scene.world.collision_objects.append(obj)
        res = self.wait(self.scene_cli.call_async(req), 10.0)
        if res is None or not res.success:
            return "failed to change the obstacle"
        return "obstacle added" if add else "obstacle removed"


class Panel(QWidget):

    finished = pyqtSignal(str)
    state_changed = pyqtSignal()

    def __init__(self, backend):
        super().__init__()
        self.backend = backend
        self.buttons = []
        self.busy = False
        self.setWindowTitle("Robotics Lecture - Demo Panel")
        self.setWindowFlags(self.windowFlags() | Qt.WindowStaysOnTopHint)

        root = QVBoxLayout(self)

        # --- ik_solutions (스택을 가리지 않는다. URDF 만 읽으므로 두 스택 어디서나 된다)
        box = QGroupBox("IK solutions  (no MoveIt needed)")
        grid = QGridLayout(box)
        self.add(grid, 0, 0, "Solve IK", self.backend.solve_ik, wide=True)
        self.valid_btn = self.button("Show usable solutions", self.toggle_valid)
        grid.addWidget(self.valid_btn, 1, 0, 1, 2)
        self.reveal_btn = self.button("Show remaining solutions", self.toggle_reveal)
        grid.addWidget(self.reveal_btn, 2, 0, 1, 2)

        # --- MoveIt 스택에서만 되는 것
        box1 = QGroupBox("Obstacles  (needs MoveIt)")
        grid1 = QGridLayout(box1)
        self.add(grid1, 0, 0, "Add obstacle", lambda: self.backend.obstacle(True))
        self.add(grid1, 0, 1, "Remove obstacle", lambda: self.backend.obstacle(False))

        # --- manipulability 타원체와 자세 프리셋 (URDF 스택)
        box2 = QGroupBox("Manipulability & poses  (no MoveIt needed)")
        grid2 = QGridLayout(box2)
        self.ellipse_btn = self.button("Show ellipsoid", self.toggle_ellipsoid)
        grid2.addWidget(self.ellipse_btn, 0, 0, 1, 3)
        for i, name in enumerate(PRESETS):
            b = self.button(name, lambda _=False, n=name: self.backend.goto(n))
            b.setToolTip(PRESET_HINT.get(name, ""))
            grid2.addWidget(b, 1 + i // 3, i % 3)

        # 위에서 아래로 manipulability -> IK -> MoveIt 것들. 강의 진행이 이 순서다:
        # manipulability 로 자세와 지표를 보이고, ik_solutions 로 해를 세고,
        # 그 다음이 MoveIt 을 얹는 obstacle 이다.
        for group in (box2, box, box1):
            root.addWidget(group)

        # 지금 떠 있지 않은 스택의 묶음은 잠가 둔다. 예전에는 눌러야 상태줄에
        # 「대상이 없다」가 떠서, 학생이 먼저 헛클릭을 하고 알았다.
        self.boxes = [
            (box2, lambda: (self.backend.ellipse_cli.service_is_ready()
                            or self.backend.goto_pub.get_subscription_count() > 0)),
            (box, lambda: self.backend.ik_cli.service_is_ready()),
            (box1, lambda: self.backend.scene_cli.service_is_ready()),
        ]

        self.status = QLabel("Ready")
        self.status.setWordWrap(True)
        self.status.setStyleSheet("color: #444; padding: 4px;")
        root.addWidget(self.status)

        self.finished.connect(self.on_finished)
        # ROS 콜백은 딴 스레드다. 시그널로 넘겨 GUI 스레드에서 라벨을 고친다.
        self.state_changed.connect(self.refresh_labels)
        self.backend.on_state = self.state_changed.emit
        # 래치된 상태가 이 창이 생기기 전에 도착했을 수 있다. 한 번 맞춰 둔다.
        self.refresh_labels()
        self.refresh_availability()
        # 스택은 패널보다 늦게 뜰 수도, 중간에 내려갈 수도 있다. 계속 확인한다.
        self.avail_timer = QTimer(self)
        self.avail_timer.timeout.connect(self.refresh_availability)
        self.avail_timer.start(1000)
        self.resize(440, 620)

    # ------------------------------------------------------------------ 배치

    def button(self, text, fn):
        b = QPushButton(text)
        b.setMinimumHeight(34)
        b.clicked.connect(lambda: self.report(fn))
        self.buttons.append(b)
        return b

    def add(self, grid, row, col, text, fn, wide=False):
        b = self.button(text, fn)
        if wide:
            b.setMinimumHeight(46)
            grid.addWidget(b, row, col, 1, 2)
        else:
            grid.addWidget(b, row, col)

    # ------------------------------------------------------------------ 실행

    def toggle_reveal(self):
        return self.backend.reveal(not self.backend.revealed)

    def toggle_valid(self):
        return self.backend.show_valid(not self.backend.valid_on)

    def toggle_ellipsoid(self):
        return self.backend.ellipsoid(not self.backend.ellipse_on)

    def report(self, fn):
        """일은 딴 스레드에서 한다. 그동안 버튼을 잠가 두 번 눌리지 않게 한다."""
        if self.busy:
            return
        self.busy = True
        for b in self.buttons:
            b.setEnabled(False)
        self.status.setText("…")

        def work():
            try:
                msg = str(fn())
            except Exception as exc:               # 강의 중에 창이 죽지 않게
                msg = f"error: {exc}"
            self.finished.emit(msg)

        threading.Thread(target=work, daemon=True).start()

    def refresh_availability(self):
        """떠 있는 스택의 묶음만 켠다. 작업 중(busy)에는 건드리지 않는다."""
        if self.busy:
            return
        for box, ready in self.boxes:
            box.setEnabled(bool(ready()))

    def refresh_labels(self):
        self.valid_btn.setText("Hide usable solutions" if self.backend.valid_on
                               else "Show usable solutions")
        self.reveal_btn.setText("Hide remaining solutions" if self.backend.revealed
                                else "Show remaining solutions")
        self.ellipse_btn.setText("Hide ellipsoid" if self.backend.ellipse_on
                                 else "Show ellipsoid")

    def on_finished(self, msg):
        self.status.setText(msg)
        self.refresh_labels()
        for b in self.buttons:
            b.setEnabled(True)
        self.busy = False
        # 버튼을 전부 켠 뒤라 잠금을 다시 씌워야 한다
        self.refresh_availability()


def main():
    rclpy.init()
    backend = Backend()

    # rclpy 는 통째로 딴 스레드에서 돌린다. 버튼 쪽은 future 가 끝나기만 기다린다.
    executor = SingleThreadedExecutor()
    executor.add_node(backend)

    def spin():
        try:
            executor.spin()
        except Exception:        # rclpy 가 내려가면 여기로 나온다. 조용히 끝낸다
            pass

    threading.Thread(target=spin, daemon=True).start()

    app = QApplication(sys.argv)
    panel = Panel(backend)
    panel.show()

    # ros2 launch 는 내릴 때 SIGINT · SIGTERM 을 보낸다. 그런데 Qt 루프 안에서는
    # 파이썬 시그널 핸들러가 돌 틈이 없어서 창이 살아남고 런치 종료가 여기서 물린다
    # (rclpy 가 시그널을 먼저 채가므로 기본 동작도 기대할 수 없다).
    # 빈 타이머로 파이썬에 틈을 주고, 시그널을 받으면 Qt 루프를 끝낸다.
    for sig in (signal.SIGINT, signal.SIGTERM):
        signal.signal(sig, lambda *_: app.quit())
    idle = QTimer()
    idle.timeout.connect(lambda: None)
    idle.start(200)

    try:
        code = app.exec_()
    finally:
        executor.shutdown()
        backend.destroy_node()
        rclpy.try_shutdown()
    return code


if __name__ == "__main__":
    sys.exit(main())

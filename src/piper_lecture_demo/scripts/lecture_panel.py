#!/usr/bin/env python3
"""강의 패널 — 데모를 버튼으로 돌린다.

강의 중에 터미널에 명령을 치지 않기 위한 것이다. 창 하나에 버튼만 있고,
누르면 이미 떠 있는 노드에게 서비스를 부르거나 토픽을 쏜다.

    ros2 run piper_lecture_demo lecture_panel.py

버튼은 세 묶음이다.

    IK 해 (D6)      두 스택 어디서나 된다 — D6 는 URDF 만 읽는다.
                    초록과 빨강을 따로 껐다 켠다
    MoveIt 스택      장애물. move_group 이 있어야 한다
    D1 스택          자세 프리셋과 타원체. d1_preset · d1_manipulability 가 있어야 한다

두 스택은 같이 띄우지 않으므로, 지금 떠 있지 않은 쪽 버튼을 누르면 「대상이 없다」고
상태줄에 뜰 뿐 아무 일도 없다.

**일은 항상 딴 스레드에서 한다.** 버튼 콜백에서 그대로 기다리면 그동안 창이 얼어붙는데,
D6 는 몇 초가 걸리므로 강의 중에 창이 죽은 것처럼 보였다. 지금은 눌린 동안 버튼이
잠기고 상태줄이 「…」로 바뀌며, 끝나면 신호로 돌아와 결과를 적는다.

PyQt5 와 rclpy 만 쓴다. rqt 플러그인이 아니라 그냥 창이라 rviz 옆에 띄워두면 된다.
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
OBSTACLE_ID = "lecture_post"
OBSTACLE_FRAME = "world"
OBSTACLE_XYZ = (0.39, 0.0, 0.25)
OBSTACLE_SIZE = (0.06, 0.06, 0.50)


class Backend(Node):
    """버튼이 시키는 일을 실제로 하는 쪽. GUI 와 분리해 둔다."""

    def __init__(self):
        super().__init__("lecture_panel")
        self.goto_pub = self.create_publisher(String, "/d1_preset/goto", 10)
        self.ik_cli = self.create_client(Trigger, "/d6_ik_branches/solve")
        self.reveal_cli = self.create_client(SetBool, "/d6_ik_branches/reveal")
        self.scene_cli = self.create_client(ApplyPlanningScene, "/apply_planning_scene")
        self.ellipse_cli = self.create_client(SetBool, "/d1_manipulability/show")
        self.valid_cli = self.create_client(SetBool, "/d6_ik_branches/show_valid")
        self.revealed = False
        self.valid_on = True
        self.ellipse_on = True
        self.on_state = None          # GUI 가 라벨을 다시 그리도록 걸어 두는 자리
        # d1 이 래치해 두는 상태. 터미널로 껐거나 ellipsoid:=false 로 떴어도 맞춘다.
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(Bool, "/d1_manipulability/shown", self.on_shown, latched)
        self.create_subscription(Bool, "/d6_ik_branches/shown_valid",
                                 self.on_shown_valid, latched)
        self.create_subscription(Bool, "/d6_ik_branches/revealed",
                                 self.on_revealed, latched)

    def wait(self, fut, timeout):
        """executor 가 딴 스레드에서 돌고 있으므로 여기서는 기다리기만 한다.

        spin_until_future_complete 를 쓰면 이 노드를 두 스레드가 같이 돌리게 된다."""
        end = time.monotonic() + timeout
        while not fut.done() and time.monotonic() < end:
            time.sleep(0.02)
        return fut.result()

    # ---------------------------------------------------------------- D1

    def goto(self, name):
        if self.goto_pub.get_subscription_count() == 0:
            return f"자세 드라이버가 없다 ({name} 무시). D1 스택이 떠 있나?"
        self.goto_pub.publish(String(data=name))
        return f"자세 -> {name}"

    # ---------------------------------------------------------------- D6

    def solve_ik(self):
        if not self.ik_cli.service_is_ready():
            return "d6_ik_branches 가 없다. 그쪽 노드를 먼저 띄울 것"
        # 새로 풀면 빨강이 다시 감춰지는데, 그것은 노드가 알려준다 (~/revealed).
        res = self.wait(self.ik_cli.call_async(Trigger.Request()), 60.0)
        return f"IK: {res.message}" if res else "IK 응답 없음"

    def reveal(self, on):
        if not self.reveal_cli.service_is_ready():
            return "d6_ik_branches 가 없다. 그쪽 노드를 먼저 띄울 것"
        res = self.wait(self.reveal_cli.call_async(SetBool.Request(data=on)), 10.0)
        if res is None:
            return "응답 없음"
        self.revealed = on
        return res.message

    def show_valid(self, on):
        if not self.valid_cli.service_is_ready():
            return "d6_ik_branches 가 없다. 그쪽 노드를 먼저 띄울 것"
        res = self.wait(self.valid_cli.call_async(SetBool.Request(data=on)), 10.0)
        if res is None:
            return "응답 없음"
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

    # ---------------------------------------------------------------- D1 타원체

    def ellipsoid(self, on):
        if not self.ellipse_cli.service_is_ready():
            return "d1_manipulability 가 없다. D1 스택이 떠 있나?"
        res = self.wait(self.ellipse_cli.call_async(SetBool.Request(data=on)), 10.0)
        if res is None:
            return "응답 없음"
        self.ellipse_on = on
        return res.message

    # ---------------------------------------------------------------- D3

    def obstacle(self, add):
        if not self.scene_cli.service_is_ready():
            return "move_group 이 없다. MoveIt 스택이 떠 있나?"
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
            return "장애물 조작 실패"
        return "장애물을 세웠다" if add else "장애물을 치웠다"


class Panel(QWidget):

    finished = pyqtSignal(str)
    state_changed = pyqtSignal()

    def __init__(self, backend):
        super().__init__()
        self.backend = backend
        self.buttons = []
        self.busy = False
        self.setWindowTitle("로보틱스 특강 — 데모 패널")
        self.setWindowFlags(self.windowFlags() | Qt.WindowStaysOnTopHint)

        root = QVBoxLayout(self)

        # --- D6 (스택을 가리지 않는다)
        box = QGroupBox("IK 해 — D6")
        grid = QGridLayout(box)
        self.add(grid, 0, 0, "IK 해 찾기  (D6)", self.backend.solve_ik, wide=True)
        self.valid_btn = self.button("쓸 수 있는 해 감추기", self.toggle_valid)
        grid.addWidget(self.valid_btn, 1, 0, 1, 2)
        self.reveal_btn = self.button("나머지 해 보이기", self.toggle_reveal)
        grid.addWidget(self.reveal_btn, 2, 0, 1, 2)
        root.addWidget(box)

        # --- MoveIt 스택에서만 되는 것
        box1 = QGroupBox("MoveIt 스택 — 장애물 (D3)")
        grid1 = QGridLayout(box1)
        self.add(grid1, 0, 0, "장애물 세우기", lambda: self.backend.obstacle(True))
        self.add(grid1, 0, 1, "장애물 치우기", lambda: self.backend.obstacle(False))
        root.addWidget(box1)

        # --- D1 스택 (자세 프리셋)
        box2 = QGroupBox("D1 스택 — 자세 프리셋")
        grid2 = QGridLayout(box2)
        self.ellipse_btn = self.button("타원체 감추기", self.toggle_ellipsoid)
        grid2.addWidget(self.ellipse_btn, 0, 0, 1, 3)
        for i, name in enumerate(PRESETS):
            b = self.button(name, lambda _=False, n=name: self.backend.goto(n))
            grid2.addWidget(b, 1 + i // 3, i % 3)
        root.addWidget(box2)

        self.status = QLabel("준비됨")
        self.status.setWordWrap(True)
        self.status.setStyleSheet("color: #444; padding: 4px;")
        root.addWidget(self.status)

        self.finished.connect(self.on_finished)
        # ROS 콜백은 딴 스레드다. 시그널로 넘겨 GUI 스레드에서 라벨을 고친다.
        self.state_changed.connect(self.refresh_labels)
        self.backend.on_state = self.state_changed.emit
        # 래치된 상태가 이 창이 생기기 전에 도착했을 수 있다. 한 번 맞춰 둔다.
        self.refresh_labels()
        self.resize(420, 440)

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
                msg = f"오류: {exc}"
            self.finished.emit(msg)

        threading.Thread(target=work, daemon=True).start()

    def refresh_labels(self):
        self.valid_btn.setText("쓸 수 있는 해 감추기" if self.backend.valid_on
                               else "쓸 수 있는 해 보이기")
        self.reveal_btn.setText("나머지 해 감추기" if self.backend.revealed
                                else "나머지 해 보이기")
        self.ellipse_btn.setText("타원체 감추기" if self.backend.ellipse_on
                                 else "타원체 보이기")

    def on_finished(self, msg):
        self.status.setText(msg)
        self.refresh_labels()
        for b in self.buttons:
            b.setEnabled(True)
        self.busy = False


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

#!/usr/bin/env python3
"""강의 패널 — 데모를 버튼으로 돌린다.

강의 중에 터미널에 명령을 치지 않기 위한 것이다. 창 하나에 버튼만 있고,
누르면 이미 떠 있는 노드에게 서비스를 부르거나 토픽을 쏜다.

    ros2 run piper_lecture_demo lecture_panel.py

MoveIt 스택 쪽 버튼과 D1 스택 쪽 버튼이 한 창에 있지만, 두 스택은 같이 띄우지 않는다.
지금 떠 있지 않은 쪽 버튼을 누르면 「대상이 없다」고 상태줄에 뜰 뿐 아무 일도 없다.

PyQt5 와 rclpy 만 쓴다. rqt 플러그인이 아니라 그냥 창이라 rviz 옆에 띄워두면 된다.
"""

import sys
import threading

import rclpy
from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtWidgets import (QApplication, QGridLayout, QGroupBox, QHBoxLayout,
                             QLabel, QPushButton, QVBoxLayout, QWidget)
from geometry_msgs.msg import Pose
from moveit_msgs.msg import CollisionObject, PlanningScene
from moveit_msgs.srv import ApplyPlanningScene
from rclpy.node import Node
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import String
from std_srvs.srv import Trigger

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
        self.scene_cli = self.create_client(ApplyPlanningScene, "/apply_planning_scene")

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
        fut = self.ik_cli.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(self, fut, timeout_sec=60.0)
        res = fut.result()
        return f"IK: {res.message}" if res else "IK 응답 없음"

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
        fut = self.scene_cli.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=10.0)
        res = fut.result()
        if res is None or not res.success:
            return "장애물 조작 실패"
        return "장애물을 세웠다" if add else "장애물을 치웠다"


class Panel(QWidget):

    def __init__(self, backend):
        super().__init__()
        self.backend = backend
        self.setWindowTitle("로보틱스 특강 — 데모 패널")
        self.setWindowFlags(self.windowFlags() | Qt.WindowStaysOnTopHint)

        root = QVBoxLayout(self)

        # --- MoveIt 스택 (D3 · D4 · D5 · D6)
        box = QGroupBox("MoveIt 스택")
        grid = QGridLayout(box)
        self.add(grid, 0, 0, "IK 해 찾기  (D6)", self.backend.solve_ik, wide=True)
        self.add(grid, 1, 0, "장애물 세우기", lambda: self.backend.obstacle(True))
        self.add(grid, 1, 1, "장애물 치우기", lambda: self.backend.obstacle(False))
        root.addWidget(box)

        # --- D1 스택 (자세 프리셋)
        box2 = QGroupBox("D1 스택 — 자세 프리셋")
        grid2 = QGridLayout(box2)
        for i, name in enumerate(PRESETS):
            grid2.addWidget(self.button(name, lambda _=False, n=name: self.backend.goto(n)),
                            i // 3, i % 3)
        root.addWidget(box2)

        self.status = QLabel("준비됨")
        self.status.setWordWrap(True)
        self.status.setStyleSheet("color: #444; padding: 4px;")
        root.addWidget(self.status)

        self.resize(420, 300)

    def button(self, text, fn):
        b = QPushButton(text)
        b.setMinimumHeight(34)
        b.clicked.connect(lambda: self.report(fn))
        return b

    def add(self, grid, row, col, text, fn, wide=False):
        b = self.button(text, fn)
        if wide:
            b.setMinimumHeight(46)
            grid.addWidget(b, row, col, 1, 2)
        else:
            grid.addWidget(b, row, col)

    def report(self, fn):
        self.status.setText("...")
        QApplication.processEvents()
        try:
            self.status.setText(str(fn()))
        except Exception as exc:                       # 강의 중에 창이 죽지 않게
            self.status.setText(f"오류: {exc}")


def main():
    rclpy.init()
    backend = Backend()

    app = QApplication(sys.argv)
    panel = Panel(backend)
    panel.show()

    # rclpy 를 조금씩 돌려 서비스 탐색이 되게 한다. 서비스 호출 자체는 버튼 쪽에서
    # spin_until_future_complete 로 처리하므로 여기서는 발견만 해주면 된다.
    timer = QTimer()
    timer.timeout.connect(lambda: rclpy.spin_once(backend, timeout_sec=0.0))
    timer.start(50)

    try:
        code = app.exec_()
    finally:
        backend.destroy_node()
        rclpy.try_shutdown()
    return code


if __name__ == "__main__":
    sys.exit(main())

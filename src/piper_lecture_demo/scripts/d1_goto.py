#!/usr/bin/env python3
"""D1 자세 전환 — 떠 있는 자세 드라이버에게 프리셋 하나를 지시하고 끝난다.

    ros2 run piper_lecture_demo d1_goto.py elbow
    ros2 run piper_lecture_demo d1_goto.py slow
    ros2 run piper_lecture_demo d1_goto.py            <- 목록만 보여준다

rviz 도 드라이버도 내리지 않는다. 강의 중에 자세만 갈아끼우기 위한 것이다.
드라이버(d1_preset)가 떠 있어야 하고, 런치를 source:=preset 으로 띄우면 같이 뜬다.
"""

import sys
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

TOPIC = "/d1_preset/goto"

# 드라이버(d1_preset.py)와 같은 목록. 여기서는 이름만 알면 된다.
NAMES = ["good", "stretch", "elbow", "wrist", "home", "shoulder", "slow"]

HINT = {
    "good": "잘 움직이는 자세. 타원체가 둥글다",
    "stretch": "팔을 편다. 타원체가 원반이 된다",
    "elbow": "팔꿈치+손목 특이점. 타원체가 선분이 된다",
    "wrist": "손목 특이점. 타원체는 둥근데 6자유도는 죽어 있다",
    "home": "모든 관절 0. 이 로봇의 영자세가 곧 손목 특이점이다",
    "shoulder": "어깨 특이점. 손목 중심이 joint1 축 위",
    "slow": "전체가 느린 자세. elbow 와 w 는 같은데 특이점이 아니다",
}


def usage():
    print("사용법: ros2 run piper_lecture_demo d1_goto.py <프리셋>")
    print()
    for n in NAMES:
        print(f"  {n:9s} {HINT[n]}")
    print()
    print("강의 순서 권장:  good -> stretch -> elbow -> slow")


def main():
    args = [a for a in sys.argv[1:] if not a.startswith("--")]
    if not args:
        usage()
        return 0
    name = args[0].strip()
    if name not in NAMES:
        print(f"'{name}' 은 없는 프리셋이다.")
        print()
        usage()
        return 1

    rclpy.init()
    node = Node("d1_goto")
    pub = node.create_publisher(String, TOPIC, 10)

    # 드라이버가 구독을 붙일 때까지 기다린다. 바로 쏘면 유실된다.
    # ⚠ spin_once 로 기다리면 안 된다 — 이 노드에는 구독도 타이머도 없어서
    #   wait set 이 비어 있고 spin_once 가 즉시 반환한다. 실제로 안 기다린다.
    deadline = time.monotonic() + 3.0
    while time.monotonic() < deadline and pub.get_subscription_count() == 0:
        time.sleep(0.05)

    if pub.get_subscription_count() == 0:
        node.get_logger().error(
            f"{TOPIC} 을 듣는 노드가 없다. 자세 드라이버가 떠 있는지 확인할 것:")
        node.get_logger().error(
            "  ros2 launch piper_lecture_demo d1_manipulability.launch.py")
        node.destroy_node()
        rclpy.try_shutdown()
        return 1

    pub.publish(String(data=name))
    time.sleep(0.2)                     # 나가기 전에 실제로 나가도록 잠깐 둔다
    print(f"-> {name}  ({HINT[name]})")
    node.destroy_node()
    rclpy.try_shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())

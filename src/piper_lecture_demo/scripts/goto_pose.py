#!/usr/bin/env python3
"""자세 전환 — 떠 있는 자세 드라이버에게 프리셋 하나를 지시하고 끝난다.

    ros2 run piper_lecture_demo goto_pose.py elbow
    ros2 run piper_lecture_demo goto_pose.py slow
    ros2 run piper_lecture_demo goto_pose.py            <- 목록만 보여준다

rviz 도 드라이버도 내리지 않는다. 강의 중에 자세만 갈아끼우기 위한 것이다.
드라이버(pose_presets)는 URDF 런치가 항상 같이 띄운다.
"""

import sys
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

TOPIC = "/pose_presets/goto"

# 드라이버(pose_presets.py)와 같은 목록. 여기서는 이름만 알면 된다.
NAMES = ["good", "stretch", "elbow", "wrist", "home", "shoulder", "slow"]

HINT = {
    "good": "well-conditioned pose. the ellipsoid is round",
    "stretch": "arm extended. the ellipsoid becomes a disc",
    "elbow": "elbow + wrist singularity. the ellipsoid becomes a line segment",
    "wrist": "wrist singularity. the ellipsoid stays round but the 6-DOF rank dies",
    "home": "all joints zero. this robot's zero pose IS the wrist singularity",
    "shoulder": "shoulder singularity. the wrist centre sits on the joint1 axis",
    "slow": "uniformly slow pose. same w as elbow, yet not a singularity",
}


def usage():
    print("usage: ros2 run piper_lecture_demo goto_pose.py <preset>")
    print()
    for n in NAMES:
        print(f"  {n:9s} {HINT[n]}")
    print()
    print("suggested lecture order:  good -> stretch -> elbow -> slow")


def main():
    args = [a for a in sys.argv[1:] if not a.startswith("--")]
    if not args:
        usage()
        return 0
    name = args[0].strip()
    if name not in NAMES:
        print(f"'{name}' is not a known preset.")
        print()
        usage()
        return 1

    rclpy.init()
    node = Node("goto_pose")
    pub = node.create_publisher(String, TOPIC, 10)

    # 드라이버가 구독을 붙일 때까지 기다린다. 바로 쏘면 유실된다.
    # ⚠ spin_once 로 기다리면 안 된다 — 이 노드에는 구독도 타이머도 없어서
    #   wait set 이 비어 있고 spin_once 가 즉시 반환한다. 실제로 안 기다린다.
    deadline = time.monotonic() + 3.0
    while time.monotonic() < deadline and pub.get_subscription_count() == 0:
        time.sleep(0.05)

    if pub.get_subscription_count() == 0:
        node.get_logger().error(
            f"nothing is listening on {TOPIC}. check that the pose driver is up:")
        node.get_logger().error(
            "  ros2 launch piper_lecture_demo urdf_demos.launch.py")
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

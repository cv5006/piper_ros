#!/usr/bin/env python3
"""D3 — 충돌 물체를 planning scene 에 세우거나 치운다. (M4)

플래너에게 주는 것은 셋뿐이다: 시작 · 목표 · 충돌 환경. 앞의 둘은 rviz 의
MotionPlanning 패널에서 직접 주고, 남은 하나를 이 노드가 준다.

    ros2 run piper_lecture_demo d3_obstacle.py spawn         # 기본 기둥을 세운다
    ros2 run piper_lecture_demo d3_obstacle.py despawn       # 그것을 치운다
    ros2 run piper_lecture_demo d3_obstacle.py despawn --all # lecture_* 를 전부 치운다
    ros2 run piper_lecture_demo d3_obstacle.py list          # 지금 뭐가 있나

자리와 크기를 바꾸려면 코드를 고칠 필요 없다.

    d3_obstacle.py spawn --id wall --shape box --xyz 0.35 0 0.30 --size 0.02 0.40 0.30
    d3_obstacle.py spawn --id can --shape cylinder --xyz 0.30 0.20 0.10 --size 0.05 0.20
    d3_obstacle.py spawn --id ball --shape sphere --xyz 0.30 0 0.40 --size 0.06

  box       --size 는 x y z 세 변 [m]
  cylinder  --size 는 반지름, 높이
  sphere    --size 는 반지름

rviz 로도 만들 수 있다 — MotionPlanning 패널의 Scene Objects 탭에서 도형을 더하고
마우스로 옮긴 뒤 Publish 하면 된다. 그렇게 만든 것을 .scene 파일로 내보내고
다시 가져올 수도 있다 (같은 탭의 Export/Import Scene Geometry). 강의처럼 매번 같은
자리에 같은 것을 세워야 할 때는 이 노드가 편하고, 즉흥적으로 만들 때는 rviz 가 편하다.

기본 기둥 자리는 계산해서 골랐다. 팔을 반경 0.39 m 로 뻗은 채 joint1 으로 좌우로
스윙하면 그 한가운데를 지나므로, 관절 공간에서 곧게 가면 반드시 통과하는 자리다.

⚠ 물체는 move_group 이 살아 있는 한 planning scene 에 남는다. 다음 데모의 목표 자세가
  「이미 충돌」이 되어 IK 부터 실패할 수 있으므로, 쓰고 나면 치울 것.
"""

import argparse
import sys

import rclpy
from geometry_msgs.msg import Pose
from moveit_msgs.msg import CollisionObject, PlanningScene, PlanningSceneComponents
from moveit_msgs.srv import ApplyPlanningScene, GetPlanningScene
from rclpy.node import Node
from shape_msgs.msg import SolidPrimitive

PREFIX = "lecture_"
DEFAULT_ID = "lecture_post"
DEFAULT_FRAME = "world"               # move_group 의 planning frame
DEFAULT_XYZ = [0.39, 0.0, 0.25]       # 두 자세의 한가운데, 팔이 뻗는 반경 위
DEFAULT_SIZE = [0.06, 0.06, 0.50]

SHAPES = {
    "box": (SolidPrimitive.BOX, 3, "x y z 세 변"),
    "cylinder": (SolidPrimitive.CYLINDER, 2, "높이, 반지름"),
    "sphere": (SolidPrimitive.SPHERE, 1, "반지름"),
}


def parse_args(argv):
    ap = argparse.ArgumentParser(
        prog="d3_obstacle.py", add_help=True,
        description="planning scene 에 충돌 물체를 세우거나 치운다")
    ap.add_argument("command", choices=["spawn", "despawn", "list"])
    ap.add_argument("--id", default=DEFAULT_ID, help=f"물체 이름 (기본 {DEFAULT_ID})")
    ap.add_argument("--all", action="store_true",
                    help=f"despawn 에서 {PREFIX}* 를 전부 치운다")
    ap.add_argument("--shape", default="box", choices=sorted(SHAPES))
    ap.add_argument("--xyz", nargs=3, type=float, default=DEFAULT_XYZ,
                    metavar=("X", "Y", "Z"))
    ap.add_argument("--size", nargs="+", type=float, default=DEFAULT_SIZE,
                    help="도형별 치수. box 는 세 변, cylinder 는 높이와 반지름, sphere 는 반지름")
    ap.add_argument("--frame", default=DEFAULT_FRAME, help=f"기준 프레임 (기본 {DEFAULT_FRAME})")
    return ap.parse_args(argv)


def make_object(args, operation):
    obj = CollisionObject()
    obj.header.frame_id = args.frame
    obj.id = args.id
    obj.operation = operation
    if operation == CollisionObject.ADD:
        kind, n, what = SHAPES[args.shape]
        if len(args.size) != n:
            raise SystemExit(f"{args.shape} 는 --size 에 {n} 개가 필요하다 ({what})")
        prim = SolidPrimitive()
        prim.type = kind
        prim.dimensions = list(args.size)
        pose = Pose()
        pose.position.x, pose.position.y, pose.position.z = args.xyz
        pose.orientation.w = 1.0
        obj.primitives.append(prim)
        obj.primitive_poses.append(pose)
    return obj


def fetch_objects(node):
    cli = node.create_client(GetPlanningScene, "/get_planning_scene")
    if not cli.wait_for_service(timeout_sec=5.0):
        node.get_logger().error(
            "/get_planning_scene 이 없다. move_group 이 떠 있는지 확인할 것:")
        node.get_logger().error("  ros2 launch piper_with_gripper_moveit demo.launch.py")
        return None
    req = GetPlanningScene.Request()
    req.components.components = (PlanningSceneComponents.WORLD_OBJECT_NAMES
                                 | PlanningSceneComponents.WORLD_OBJECT_GEOMETRY)
    fut = cli.call_async(req)
    rclpy.spin_until_future_complete(node, fut, timeout_sec=10.0)
    res = fut.result()
    return None if res is None else res.scene.world.collision_objects


def apply_objects(node, objects, label):
    cli = node.create_client(ApplyPlanningScene, "/apply_planning_scene")
    if not cli.wait_for_service(timeout_sec=5.0):
        node.get_logger().error(
            "/apply_planning_scene 이 없다. move_group 이 떠 있는지 확인할 것")
        return False
    req = ApplyPlanningScene.Request()
    scene = PlanningScene()
    scene.is_diff = True              # 통째로 갈아끼우지 않고 이 물체들만 더한다/뺀다
    scene.world.collision_objects.extend(objects)
    req.scene = scene
    fut = cli.call_async(req)
    rclpy.spin_until_future_complete(node, fut, timeout_sec=10.0)
    res = fut.result()
    if res is None or not res.success:
        node.get_logger().error(f"{label} 실패")
        return False
    print(f"-> {label}")
    return True


def main():
    args = parse_args([a for a in sys.argv[1:] if a != "--ros-args"])
    rclpy.init()
    node = Node("d3_obstacle")
    ok = False
    try:
        if args.command == "list":
            objs = fetch_objects(node)
            if objs is not None:
                print(f"planning scene 안의 충돌 물체: {len(objs)}")
                for o in objs:
                    dims = list(o.primitives[0].dimensions) if o.primitives else []
                    print(f"  - {o.id}  {[round(d, 3) for d in dims]}")
                if not objs:
                    print("  (비어 있다)")
                ok = True

        elif args.command == "spawn":
            ok = apply_objects(node, [make_object(args, CollisionObject.ADD)],
                               f"{args.id} ({args.shape}) 를 {args.xyz} 에 세웠다")

        else:                          # despawn
            if args.all:
                objs = fetch_objects(node)
                if objs is None:
                    return 1
                names = [o.id for o in objs if o.id.startswith(PREFIX)]
                if not names:
                    print(f"치울 {PREFIX}* 물체가 없다")
                    return 0
                removals = []
                for name in names:
                    obj = CollisionObject()
                    obj.header.frame_id = args.frame
                    obj.id = name
                    obj.operation = CollisionObject.REMOVE
                    removals.append(obj)
                ok = apply_objects(node, removals, f"{len(names)} 개를 치웠다: {' '.join(names)}")
            else:
                ok = apply_objects(node, [make_object(args, CollisionObject.REMOVE)],
                                   f"{args.id} 를 치웠다")
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())

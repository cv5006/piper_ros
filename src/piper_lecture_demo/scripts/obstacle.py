#!/usr/bin/env python3
"""충돌 물체를 planning scene 에 세우거나 치운다. (M4)

플래너에 주는 것은 셋 — 시작 · 목표 · 충돌 환경. 앞의 둘은 rviz, 나머지가 이 노드다.

    obstacle.py spawn              기본 기둥
    obstacle.py despawn [--all]    그것만 / lecture_* 전부
    obstacle.py list

    --shape box       --size = x y z 세 변 [m]
    --shape cylinder  --size = 높이, 반지름 (shape_msgs/SolidPrimitive 순서)
    --shape sphere    --size = 반지름

기본 기둥 자리는 계산해서 골랐다 — 반경 0.39 m 로 뻗어 joint1 으로 스윙하면 그
한가운데를 지나므로 관절 공간에서 곧게 가면 반드시 통과한다.

⚠ 물체는 move_group 이 사는 동안 남는다. 다음 데모가 「이미 충돌」로 IK 부터
  실패할 수 있다. 쓰고 나면 치울 것.
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
    "box": (SolidPrimitive.BOX, 3, "x y z edge lengths"),
    "cylinder": (SolidPrimitive.CYLINDER, 2, "height, radius"),
    "sphere": (SolidPrimitive.SPHERE, 1, "radius"),
}


def parse_args(argv):
    ap = argparse.ArgumentParser(
        prog="obstacle.py", add_help=True,
        description="add or remove collision objects in the planning scene")
    ap.add_argument("command", choices=["spawn", "despawn", "list"])
    ap.add_argument("--id", default=DEFAULT_ID, help=f"object name (default {DEFAULT_ID})")
    ap.add_argument("--all", action="store_true",
                    help=f"with despawn, remove every {PREFIX}* object")
    ap.add_argument("--shape", default="box", choices=sorted(SHAPES))
    ap.add_argument("--xyz", nargs=3, type=float, default=DEFAULT_XYZ,
                    metavar=("X", "Y", "Z"))
    ap.add_argument("--size", nargs="+", type=float, default=DEFAULT_SIZE,
                    help="dimensions per shape: box = three edges, "
                         "cylinder = height then radius, sphere = radius")
    ap.add_argument("--frame", default=DEFAULT_FRAME, help=f"reference frame (default {DEFAULT_FRAME})")
    return ap.parse_args(argv)


def make_object(args, operation):
    obj = CollisionObject()
    obj.header.frame_id = args.frame
    obj.id = args.id
    obj.operation = operation
    if operation == CollisionObject.ADD:
        kind, n, what = SHAPES[args.shape]
        if len(args.size) != n:
            raise SystemExit(f"{args.shape} needs {n} value(s) in --size ({what})")
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
            "/get_planning_scene is missing. check that move_group is up:")
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
            "/apply_planning_scene is missing. check that move_group is up")
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
        node.get_logger().error(f"{label} failed")
        return False
    print(f"-> {label}")
    return True


def main():
    args = parse_args([a for a in sys.argv[1:] if a != "--ros-args"])
    rclpy.init()
    node = Node("obstacle")
    ok = False
    try:
        if args.command == "list":
            objs = fetch_objects(node)
            if objs is not None:
                print(f"collision objects in the planning scene: {len(objs)}")
                for o in objs:
                    dims = list(o.primitives[0].dimensions) if o.primitives else []
                    print(f"  - {o.id}  {[round(d, 3) for d in dims]}")
                if not objs:
                    print("  (empty)")
                ok = True

        elif args.command == "spawn":
            ok = apply_objects(node, [make_object(args, CollisionObject.ADD)],
                               f"added {args.id} ({args.shape}) at {args.xyz}")

        else:                          # despawn
            if args.all:
                objs = fetch_objects(node)
                if objs is None:
                    return 1
                names = [o.id for o in objs if o.id.startswith(PREFIX)]
                if not names:
                    print(f"no {PREFIX}* objects to remove")
                    return 0
                removals = []
                for name in names:
                    obj = CollisionObject()
                    obj.header.frame_id = args.frame
                    obj.id = name
                    obj.operation = CollisionObject.REMOVE
                    removals.append(obj)
                ok = apply_objects(node, removals, f"removed {len(names)}: {' '.join(names)}")
            else:
                ok = apply_objects(node, [make_object(args, CollisionObject.REMOVE)],
                                   f"removed {args.id}")
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())

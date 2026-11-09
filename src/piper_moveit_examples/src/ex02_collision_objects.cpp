// ex02 — 충돌 물체 (Planning Scene 의 world)
//
// 순서
// 1. 정의   CollisionObject = id + 기준 프레임 + 모양(primitive/mesh) + 자세
// 2. 추가   ADD
// 3. 조회   이름 · 자세
// 4. 이동   MOVE (모양은 그대로, 자세만)
// 5. 부착   AttachedCollisionObject — 그리퍼에 붙임 → 팔과 함께 움직이는 물체
// 6. 분리   부착 해제 → 다시 world 물체
// 7. 제거   REMOVE
//
// 요점
// - 플래너는 Planning Scene 에 있는 것만 피함 (카메라가 본 것도 여기 넣어야 의미 있음)
// - apply*()  : 반영 완료까지 기다림 (동기) → 예제는 이것만 사용
// - add*()    : 토픽으로 보내고 바로 반환 (비동기) → 직후 계획하면 반영 전일 수 있음
//
// 실행
// - ros2 launch piper_moveit_examples stack.launch.py
// - ros2 launch piper_moveit_examples example.launch.py ex:=ex2
// - rviz: MotionPlanning > Scene Objects 또는 화면에서 물체 확인
//
// 팔을 움직이지 않음

#include <chrono>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/attached_collision_object.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>

using moveit::planning_interface::PlanningSceneInterface;

namespace
{
// 상자 하나 정의. 자세는 frame_id 기준, 상자 중심
moveit_msgs::msg::CollisionObject makeBox(const std::string& id, double sx, double sy, double sz,
                                          double x, double y, double z)
{
  moveit_msgs::msg::CollisionObject obj;
  obj.id = id;
  obj.header.frame_id = "base_link";

  shape_msgs::msg::SolidPrimitive box;
  box.type = shape_msgs::msg::SolidPrimitive::BOX;
  box.dimensions = { sx, sy, sz };  // BOX_X, BOX_Y, BOX_Z [m]

  geometry_msgs::msg::Pose pose;
  pose.position.x = x;
  pose.position.y = y;
  pose.position.z = z;
  pose.orientation.w = 1.0;

  obj.primitives.push_back(box);
  obj.primitive_poses.push_back(pose);
  obj.operation = moveit_msgs::msg::CollisionObject::ADD;
  return obj;
}

moveit_msgs::msg::CollisionObject makeCylinder(const std::string& id, double h, double r,
                                               double x, double y, double z)
{
  moveit_msgs::msg::CollisionObject obj;
  obj.id = id;
  obj.header.frame_id = "base_link";

  shape_msgs::msg::SolidPrimitive cyl;
  cyl.type = shape_msgs::msg::SolidPrimitive::CYLINDER;
  cyl.dimensions = { h, r };  // CYLINDER_HEIGHT, CYLINDER_RADIUS [m]

  geometry_msgs::msg::Pose pose;
  pose.position.x = x;
  pose.position.y = y;
  pose.position.z = z;
  pose.orientation.w = 1.0;

  obj.primitives.push_back(cyl);
  obj.primitive_poses.push_back(pose);
  obj.operation = moveit_msgs::msg::CollisionObject::ADD;
  return obj;
}
}  // namespace

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions opt;
  opt.automatically_declare_parameters_from_overrides(true);
  auto node = rclcpp::Node::make_shared("ex02_collision_objects", opt);
  const auto log = node->get_logger();
  const double pause = node->get_parameter_or("pause", 2.0);
  auto wait = [pause]() { std::this_thread::sleep_for(std::chrono::duration<double>(pause)); };

  PlanningSceneInterface scene;

  // ---------------------------------------------------------------- 1~2. 정의 + 추가
  RCLCPP_INFO(log, "[1-2] Add a box (table) and a cylinder");
  auto table = makeBox("ex_table", 0.40, 0.60, 0.02, 0.35, 0.0, -0.02);
  auto post = makeCylinder("ex_post", 0.30, 0.03, 0.30, 0.20, 0.15);
  scene.applyCollisionObjects({ table, post });
  wait();

  // ---------------------------------------------------------------- 3. 조회
  RCLCPP_INFO(log, "[3] Query");
  for (const auto& [id, pose] : scene.getObjectPoses(scene.getKnownObjectNames()))
  {
    RCLCPP_INFO(log, "  %s  [%.3f %.3f %.3f]", id.c_str(), pose.position.x, pose.position.y,
                pose.position.z);
  }

  // ---------------------------------------------------------------- 4. 이동
  RCLCPP_INFO(log, "[4] Move the cylinder (y: +0.20 -> -0.20)");
  moveit_msgs::msg::CollisionObject move;
  move.id = "ex_post";
  move.header.frame_id = "base_link";
  move.operation = moveit_msgs::msg::CollisionObject::MOVE;
  move.pose.position.x = 0.30;
  move.pose.position.y = -0.20;
  move.pose.position.z = 0.15;
  move.pose.orientation.w = 1.0;
  scene.applyCollisionObject(move);
  wait();

  // ---------------------------------------------------------------- 5. 부착
  // - link_name: 붙일 링크
  // - touch_links: 이 링크들과는 닿아도 충돌로 안 봄 (안 주면 쥔 순간 충돌 상태)
  RCLCPP_INFO(log, "[5] Attach a small box to the gripper");
  moveit_msgs::msg::AttachedCollisionObject held;
  held.link_name = "gripper_base";
  held.object = makeBox("ex_held", 0.03, 0.03, 0.06, 0.0, 0.0, 0.14);
  held.object.header.frame_id = "gripper_base";  // 자세를 그리퍼 기준으로 줌
  held.touch_links = { "gripper_base", "link7", "link8" };
  scene.applyAttachedCollisionObject(held);
  RCLCPP_INFO(log, "  attached objects: %zu — moves with the arm in rviz",
              scene.getAttachedObjects().size());
  wait();

  // ---------------------------------------------------------------- 6. 분리
  // - 부착 해제만 하면 물체는 그 자리에 world 물체로 남음
  RCLCPP_INFO(log, "[6] Detach -> stays in place as a world object");
  moveit_msgs::msg::AttachedCollisionObject release;
  release.link_name = "gripper_base";
  release.object.id = "ex_held";
  release.object.operation = moveit_msgs::msg::CollisionObject::REMOVE;
  scene.applyAttachedCollisionObject(release);
  wait();

  // ---------------------------------------------------------------- 7. 제거
  RCLCPP_INFO(log, "[7] Remove everything");
  std::vector<moveit_msgs::msg::CollisionObject> removes;
  for (const auto& id : { "ex_table", "ex_post", "ex_held" })
  {
    moveit_msgs::msg::CollisionObject rm;
    rm.id = id;
    rm.header.frame_id = "base_link";
    rm.operation = moveit_msgs::msg::CollisionObject::REMOVE;
    removes.push_back(rm);
  }
  scene.applyCollisionObjects(removes);
  // 단축형: scene.removeCollisionObjects({"ex_table", ...}) — 비동기
  RCLCPP_INFO(log, "  objects left: %zu", scene.getKnownObjectNames().size());

  rclcpp::shutdown();
  return 0;
}

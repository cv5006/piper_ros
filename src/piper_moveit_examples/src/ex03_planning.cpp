// ex03 — 계획 (plan 만, 실행 안 함)
//
// 목표를 주는 방법
// 1. named target   SRDF <group_state> 이름
// 2. joint target   관절값 6개
// 3. pose target    link6 자세 (MoveIt 기본)
// 4. TCP 기준 목표  손끝 자세 → link6 자세로 환산해서 줌
//
// 계획에 영향을 주는 것
// 5. 장애물         Planning Scene 에 물체 추가 → 돌아가는 경로
// 6. 플래너 선택    OMPL(기본, 샘플링) vs Pilz LIN(손끝 직선) — 반복 비교는 ex03_1
// 7. 카테시안 경로  플래너 아님. 손끝 직선 보간 + 구간별 IK → fraction(간 비율)
//
// 요점
// - setStartState(): 지금 자세가 아닌 곳에서 출발하는 계획도 가능 (팔 안 움직임)
// - 계획 결과(궤적)는 rviz 에 자동 표시 (MotionPlanning > Planned Path)
// - 계획 성공 ≠ 실행 성공. 실행은 ex04
//
// 실행
// - ros2 launch piper_moveit_examples stack.launch.py
// - ros2 launch piper_moveit_examples example.launch.py ex:=ex3 pause:=4.0

#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/utils/moveit_error_code.h>
#include <tf2_eigen/tf2_eigen.hpp>

using moveit::planning_interface::MoveGroupInterface;
using moveit::planning_interface::PlanningSceneInterface;

namespace
{
// link6 원점 → 손끝(TCP): link6 의 z 방향 0.1358 m (URDF · 그리퍼 메시 기준)
// - ⚠ 실제 집는 지점은 쥐는 물체 · 손가락 위치에 따라 직접 정할 것
const Eigen::Isometry3d kLink6ToTcp(Eigen::Translation3d(0.0, 0.0, 0.1358));

// 관절 한계에서 떨어진 출발 자세
const std::vector<double> kStartQ = { 0.0, 0.9, -0.9, 0.0, 0.6, 0.0 };

// 손끝 자세 → link6 자세: T_link6 = T_tcp · (T_link6→tcp)^-1
geometry_msgs::msg::Pose tcpToLink6(const geometry_msgs::msg::Pose& tcp)
{
  Eigen::Isometry3d T_tcp;
  tf2::fromMsg(tcp, T_tcp);
  return tf2::toMsg(Eigen::Isometry3d(T_tcp * kLink6ToTcp.inverse()));
}

// 계획 1회 + 결과 출력
bool planAndReport(MoveGroupInterface& arm, const rclcpp::Logger& log, const char* label,
                   MoveGroupInterface::Plan& plan)
{
  const auto code = arm.plan(plan);
  if (code != moveit::core::MoveItErrorCode::SUCCESS)
  {
    RCLCPP_WARN(log, "  %s: FAILED (%s)", label, moveit::core::error_code_to_string(code).c_str());
    return false;
  }
  const auto& pts = plan.trajectory_.joint_trajectory.points;
  const double dur = rclcpp::Duration(pts.back().time_from_start).seconds();
  RCLCPP_INFO(log, "  %s: ok  planning %.3f s · waypoints %zu · duration %.2f s", label,
              plan.planning_time_, pts.size(), dur);
  return true;
}
}  // namespace

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions opt;
  opt.automatically_declare_parameters_from_overrides(true);
  auto node = rclcpp::Node::make_shared("ex03_planning", opt);
  const auto log = node->get_logger();
  const double pause = node->get_parameter_or("pause", 2.0);
  auto wait = [pause]() { std::this_thread::sleep_for(std::chrono::duration<double>(pause)); };

  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(node);
  std::thread spinner([&exec]() { exec.spin(); });

  MoveGroupInterface arm(node, "arm");
  PlanningSceneInterface scene;

  // 공통 설정
  arm.setPlanningTime(5.0);                 // 플래너 시간 상한 [s]
  arm.setNumPlanningAttempts(3);            // 시도 횟수 (가장 짧은 해 선택)
  arm.setMaxVelocityScalingFactor(0.2);     // joint_limits.yaml 속도의 20 %
  arm.setMaxAccelerationScalingFactor(0.2);  // ⚠ 이 레포에선 무효 (가속도 한계 미정의, ex04)

  // 출발 자세 = kStartQ (지금 팔 자세와 무관)
  moveit::core::RobotState start(*arm.getCurrentState(5.0));
  start.setJointGroupPositions("arm", kStartQ);
  start.update();
  arm.setStartState(start);

  // 출발 자세의 link6 · 손끝 — 이후 목표의 기준
  const Eigen::Isometry3d T_link6 = start.getGlobalLinkTransform("link6");
  const geometry_msgs::msg::Pose start_tcp = tf2::toMsg(Eigen::Isometry3d(T_link6 * kLink6ToTcp));

  MoveGroupInterface::Plan plan;

  // ---------------------------------------------------------------- 1. named target
  RCLCPP_INFO(log, "[1] named target 'zero' (SRDF)");
  arm.setNamedTarget("zero");
  planAndReport(arm, log, "zero", plan);
  wait();

  // ---------------------------------------------------------------- 2. joint target
  RCLCPP_INFO(log, "[2] joint target — joint 1 only, +0.8 rad");
  std::vector<double> q_goal = kStartQ;
  q_goal[0] += 0.8;
  arm.setJointValueTarget(q_goal);
  planAndReport(arm, log, "joint", plan);
  wait();

  // ---------------------------------------------------------------- 3. pose target (link6)
  RCLCPP_INFO(log, "[3] pose target — link6 y +0.15 m");
  geometry_msgs::msg::Pose link6_goal = tf2::toMsg(T_link6);
  link6_goal.position.y += 0.15;
  arm.setPoseTarget(link6_goal);  // 링크 생략 → getEndEffectorLink() = link6
  planAndReport(arm, log, "pose(link6)", plan);
  wait();

  // ---------------------------------------------------------------- 4. TCP 기준 목표
  RCLCPP_INFO(log, "[4] TCP target — fingertip y +0.15 m, z -0.05 m");
  geometry_msgs::msg::Pose tcp_goal = start_tcp;
  tcp_goal.position.y += 0.15;
  tcp_goal.position.z -= 0.05;
  const auto link6_for_tcp = tcpToLink6(tcp_goal);
  RCLCPP_INFO(log, "  TCP goal [%.3f %.3f %.3f] -> link6 goal [%.3f %.3f %.3f]",
              tcp_goal.position.x, tcp_goal.position.y, tcp_goal.position.z,
              link6_for_tcp.position.x, link6_for_tcp.position.y, link6_for_tcp.position.z);
  arm.setPoseTarget(link6_for_tcp);
  planAndReport(arm, log, "pose(tcp)", plan);
  wait();

  // ---------------------------------------------------------------- 5. 장애물
  RCLCPP_INFO(log, "[5] Obstacle — same goal as [2], wall in between");
  moveit_msgs::msg::CollisionObject wall;
  wall.id = "ex_wall";
  wall.header.frame_id = "base_link";
  shape_msgs::msg::SolidPrimitive box;
  box.type = shape_msgs::msg::SolidPrimitive::BOX;
  // 관절 1 이 0 → 0.8 rad 로 도는 길목(0.4 rad 방향)에 반경 방향으로 세운 판
  box.dimensions = { 0.30, 0.02, 0.36 };
  geometry_msgs::msg::Pose wall_pose;
  wall_pose.position.x = 0.25 * std::cos(0.4);
  wall_pose.position.y = 0.25 * std::sin(0.4);
  wall_pose.position.z = 0.18;
  wall_pose.orientation.z = std::sin(0.2);  // z 축 0.4 rad 회전
  wall_pose.orientation.w = std::cos(0.2);
  wall.primitives.push_back(box);
  wall.primitive_poses.push_back(wall_pose);
  wall.operation = moveit_msgs::msg::CollisionObject::ADD;
  scene.applyCollisionObject(wall);

  // OMPL 은 같은 문제도 가끔 실패 (무작위 샘플링) → 재시도가 기본 패턴 (ex03_1 에서 비교)
  arm.setJointValueTarget(q_goal);
  for (int attempt = 1; attempt <= 5; ++attempt)
  {
    if (planAndReport(arm, log, "joint + wall", plan))
    {
      break;
    }
    RCLCPP_INFO(log, "  retry %d/5", attempt);
  }
  RCLCPP_INFO(log, "  - compare waypoints / duration with [2] -> goes around the wall");
  wait();
  scene.removeCollisionObjects({ "ex_wall" });
  std::this_thread::sleep_for(std::chrono::milliseconds(500));

  // ---------------------------------------------------------------- 6. Pilz LIN
  RCLCPP_INFO(log, "[6] Switch planner — Pilz LIN (straight line), same goal as [3]");
  arm.setPlanningPipelineId("pilz_industrial_motion_planner");
  arm.setPlannerId("LIN");  // PTP: 관절 직선 (⚠ 이 레포에선 안 됨) · LIN: 손끝 직선 · CIRC: 원호
  arm.setPoseTarget(link6_goal);
  planAndReport(arm, log, "LIN", plan);
  arm.setPlanningPipelineId("ompl");  // 원복
  arm.setPlannerId("");
  wait();

  // ---------------------------------------------------------------- 7. 카테시안 경로
  RCLCPP_INFO(log, "[7] Cartesian path — 2 straight-line waypoints");
  std::vector<geometry_msgs::msg::Pose> waypoints;
  auto p = tf2::toMsg(T_link6);
  p.position.z -= 0.08;
  waypoints.push_back(p);
  p.position.y += 0.10;
  waypoints.push_back(p);
  moveit_msgs::msg::RobotTrajectory cart;
  const double fraction = arm.computeCartesianPath(waypoints, 0.005 /*eef_step [m]*/,
                                                   0.0 /*jump_threshold: 0 = 끔*/, cart);
  RCLCPP_INFO(log, "  fraction = %.3f (1.0 = reached the end)", fraction);
  RCLCPP_INFO(log, "  - below 1.0: IK failure / collision on the way -> trajectory up to that point");

  arm.clearPoseTargets();
  rclcpp::shutdown();
  spinner.join();
  return 0;
}

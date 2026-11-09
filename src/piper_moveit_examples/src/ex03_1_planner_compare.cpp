// ex03_1 — 플래너 비교: OMPL vs Pilz LIN (ex03 [5] · [6] 의 보충)
//
// 같은 출발 · 목표(관절 1 을 +0.8 rad 돌린 자리의 link6 자세)를 두 플래너로 각각 여러 번 계획
// 1. 벽 없음   둘 다 성공
// 2. 벽 있음   LIN 실패 · OMPL 대개 성공
//
// 보는 것
// - OMPL (샘플링 기반)
//   - 장애물 회피 ○
//   - 매번 다른 경로 (무작위 샘플링) → 경유점 수 · 소요 시간 · 경로 길이가 매번 다름
//   - 가끔 실패 → 재시도 · setPlanningTime 늘리기
// - Pilz LIN (손끝 직선)
//   - 장애물 회피 ✕ → 직선이 막히면 실패
//   - 매번 같은 경로
// - 고르는 기준
//   - 경로를 예측 · 재현해야 함 → Pilz (막히면 경유점을 직접 줌)
//   - 장애물 사이를 알아서 빠져나가야 함 → OMPL
//
// ⚠ Pilz PTP(관절 직선)는 이 레포에서 안 됨
// - 'acceleration limit not set for group arm' — joint_limits.yaml 에 가속도 한계 없음 (README §4)
//
// 실행 (계획만, 팔 안 움직임)
// - ros2 launch piper_moveit_examples stack.launch.py
// - ros2 launch piper_moveit_examples example.launch.py ex:=ex3_1

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

using moveit::planning_interface::MoveGroupInterface;
using moveit::planning_interface::PlanningSceneInterface;

namespace
{
const std::vector<double> kStartQ = { 0.0, 0.9, -0.9, 0.0, 0.6, 0.0 };
constexpr int kRepeat = 3;  // 플래너마다 반복 횟수

// 관절 공간 경로 길이 [rad] — 경유점 사이 관절 변화량의 합
double jointPathLength(const moveit_msgs::msg::RobotTrajectory& t)
{
  const auto& pts = t.joint_trajectory.points;
  double len = 0.0;
  for (size_t i = 1; i < pts.size(); ++i)
  {
    double d2 = 0.0;
    for (size_t j = 0; j < pts[i].positions.size(); ++j)
    {
      const double d = pts[i].positions[j] - pts[i - 1].positions[j];
      d2 += d * d;
    }
    len += std::sqrt(d2);
  }
  return len;
}

// 한 플래너로 kRepeat 번 계획 → 결과 한 줄씩
void compare(MoveGroupInterface& arm, const rclcpp::Logger& log, const std::string& pipeline,
             const std::string& planner, const char* label)
{
  arm.setPlanningPipelineId(pipeline);
  arm.setPlannerId(planner);
  for (int i = 1; i <= kRepeat; ++i)
  {
    MoveGroupInterface::Plan plan;
    const auto code = arm.plan(plan);
    if (code != moveit::core::MoveItErrorCode::SUCCESS)
    {
      RCLCPP_WARN(log, "  %-5s #%d  FAILED (%s)", label, i,
                  moveit::core::error_code_to_string(code).c_str());
      continue;
    }
    const auto& pts = plan.trajectory_.joint_trajectory.points;
    RCLCPP_INFO(log, "  %-5s #%d  ok  waypoints %3zu · duration %.2f s · joint path %.3f rad", label, i,
                pts.size(), rclcpp::Duration(pts.back().time_from_start).seconds(),
                jointPathLength(plan.trajectory_));
  }
}

moveit_msgs::msg::CollisionObject makeWall()
{
  // 출발 → 목표 사이(0.4 rad 방향)에 반경 방향으로 세운 판 (ex03 [5] 와 같음)
  moveit_msgs::msg::CollisionObject wall;
  wall.id = "ex_wall";
  wall.header.frame_id = "base_link";
  shape_msgs::msg::SolidPrimitive box;
  box.type = shape_msgs::msg::SolidPrimitive::BOX;
  box.dimensions = { 0.30, 0.02, 0.36 };
  geometry_msgs::msg::Pose pose;
  pose.position.x = 0.25 * std::cos(0.4);
  pose.position.y = 0.25 * std::sin(0.4);
  pose.position.z = 0.18;
  pose.orientation.z = std::sin(0.2);
  pose.orientation.w = std::cos(0.2);
  wall.primitives.push_back(box);
  wall.primitive_poses.push_back(pose);
  wall.operation = moveit_msgs::msg::CollisionObject::ADD;
  return wall;
}
}  // namespace

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions opt;
  opt.automatically_declare_parameters_from_overrides(true);
  auto node = rclcpp::Node::make_shared("ex03_1_planner_compare", opt);
  const auto log = node->get_logger();
  const double pause = node->get_parameter_or("pause", 2.0);
  auto wait = [pause]() { std::this_thread::sleep_for(std::chrono::duration<double>(pause)); };

  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(node);
  std::thread spinner([&exec]() { exec.spin(); });

  MoveGroupInterface arm(node, "arm");
  PlanningSceneInterface scene;
  arm.setPlanningTime(5.0);
  arm.setMaxVelocityScalingFactor(0.2);

  // 출발 · 목표 고정
  moveit::core::RobotState start(*arm.getCurrentState(5.0));
  start.setJointGroupPositions("arm", kStartQ);
  start.update();
  arm.setStartState(start);
  // 목표 = 관절 1 을 +0.8 rad 돌린 자세의 link6 자세 (LIN 은 자세 목표만 받음)
  moveit::core::RobotState goal_state(start);
  std::vector<double> goal_q = kStartQ;
  goal_q[0] += 0.8;
  goal_state.setJointGroupPositions("arm", goal_q);
  goal_state.update();
  const Eigen::Isometry3d T_goal = goal_state.getGlobalLinkTransform("link6");
  geometry_msgs::msg::Pose goal;
  goal.position.x = T_goal.translation().x();
  goal.position.y = T_goal.translation().y();
  goal.position.z = T_goal.translation().z();
  const Eigen::Quaterniond q_goal(T_goal.rotation());
  goal.orientation.x = q_goal.x();
  goal.orientation.y = q_goal.y();
  goal.orientation.z = q_goal.z();
  goal.orientation.w = q_goal.w();
  arm.setPoseTarget(goal);

  // ---------------------------------------------------------------- 1. 벽 없음
  RCLCPP_INFO(log, "[1] No wall — same goal, %d times each", kRepeat);
  compare(arm, log, "ompl", "", "OMPL");
  compare(arm, log, "pilz_industrial_motion_planner", "LIN", "LIN");
  RCLCPP_INFO(log, "  - LIN: identical every time · OMPL: may differ each time");
  wait();

  // ---------------------------------------------------------------- 2. 벽 있음
  RCLCPP_INFO(log, "[2] With wall — same goal, %d times each", kRepeat);
  scene.applyCollisionObject(makeWall());
  compare(arm, log, "pilz_industrial_motion_planner", "LIN", "LIN");
  compare(arm, log, "ompl", "", "OMPL");
  RCLCPP_INFO(log, "  - LIN: straight line blocked by the wall -> fails (no avoidance)");
  RCLCPP_INFO(log, "  - OMPL: finds a way around -> usually succeeds, path differs each time");
  RCLCPP_INFO(log, "  - OMPL INVALID_MOTION_PLAN: caught by the dense post-plan collision check -> retry");
  wait();

  scene.removeCollisionObjects({ "ex_wall" });
  arm.setPlanningPipelineId("ompl");
  arm.setPlannerId("");
  rclcpp::shutdown();
  spinner.join();
  return 0;
}

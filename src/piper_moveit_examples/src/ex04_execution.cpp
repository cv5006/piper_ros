// ex04 — 실행 (팔이 움직임)
//
// 순서
// 1. plan → 확인 → execute   계획과 실행 분리 (권장)
// 2. move()                   계획 + 실행 한 번에
// 3. asyncExecute + stop()    실행 중 정지
// 4. 그리퍼 그룹              named state open / close
// 5. 카테시안 경로 실행       속도 스케일링 확인 후 실행
//
// 요점
// - 실행 = 궤적을 FollowJointTrajectory 액션으로 컨트롤러에 전달
//   - mock:  ros2_control 가짜 하드웨어가 받음
//   - real:  실물 어댑터가 받아 piper 드라이버로 보냄
// - 이 코드는 mock / real 구분 없음 → 같은 코드가 실물도 움직임
// - 실행 전 현재 자세 = 계획 출발 자세여야 함 (어긋나면 실행 거부)
//
// ⚠ 실물(real:=true)
// - 팔 주변 비우기 · 비상정지 손 닿는 곳에
// - 처음엔 speed_percent 낮게 (stack.launch.py 기본 20)
//
// 실행
// - ros2 launch piper_moveit_examples stack.launch.py
// - ros2 launch piper_moveit_examples example.launch.py ex:=ex4

#include <chrono>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/utils/moveit_error_code.h>

using moveit::planning_interface::MoveGroupInterface;

namespace
{
const std::vector<double> kStartQ = { 0.0, 0.9, -0.9, 0.0, 0.6, 0.0 };

double durationOf(const moveit_msgs::msg::RobotTrajectory& t)
{
  const auto& pts = t.joint_trajectory.points;
  return pts.empty() ? 0.0 : rclcpp::Duration(pts.back().time_from_start).seconds();
}

void printJoints(MoveGroupInterface& arm, const rclcpp::Logger& log, const char* label)
{
  const auto q = arm.getCurrentJointValues();
  RCLCPP_INFO(log, "  %s q = [%.3f %.3f %.3f %.3f %.3f %.3f]", label, q[0], q[1], q[2], q[3],
              q[4], q[5]);
}

bool ok(const moveit::core::MoveItErrorCode& code, const rclcpp::Logger& log, const char* what)
{
  if (code == moveit::core::MoveItErrorCode::SUCCESS)
  {
    return true;
  }
  RCLCPP_ERROR(log, "  %s FAILED: %s", what, moveit::core::error_code_to_string(code).c_str());
  return false;
}
}  // namespace

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions opt;
  opt.automatically_declare_parameters_from_overrides(true);
  auto node = rclcpp::Node::make_shared("ex04_execution", opt);
  const auto log = node->get_logger();
  const double pause = node->get_parameter_or("pause", 2.0);
  auto wait = [pause]() { std::this_thread::sleep_for(std::chrono::duration<double>(pause)); };

  // asyncExecute 중에도 상태 수신이 돌도록 멀티스레드
  rclcpp::executors::MultiThreadedExecutor exec;
  exec.add_node(node);
  std::thread spinner([&exec]() { exec.spin(); });

  MoveGroupInterface arm(node, "arm");
  MoveGroupInterface gripper(node, "gripper");
  arm.setMaxVelocityScalingFactor(0.2);
  arm.setMaxAccelerationScalingFactor(0.2);
  arm.setPlanningTime(5.0);

  auto finish = [&](int rc) {
    rclcpp::shutdown();
    spinner.join();
    return rc;
  };

  // ---------------------------------------------------------------- 1. plan → execute
  RCLCPP_INFO(log, "[1] plan -> check -> execute: to the start posture");
  printJoints(arm, log, "now");
  MoveGroupInterface::Plan plan;
  arm.setStartStateToCurrentState();
  arm.setJointValueTarget(kStartQ);
  if (!ok(arm.plan(plan), log, "plan"))
  {
    return finish(1);
  }
  RCLCPP_INFO(log, "  duration %.2f s -> execute", durationOf(plan.trajectory_));
  // 여기서 궤적 검사 가능 (경유점 · 소요 시간 · 관절 범위 등)
  if (!ok(arm.execute(plan), log, "execute"))
  {
    return finish(1);
  }
  printJoints(arm, log, "arrived");
  wait();

  // ---------------------------------------------------------------- 2. move()
  RCLCPP_INFO(log, "[2] move(): joint 1 +0.6 rad");
  std::vector<double> q = kStartQ;
  q[0] += 0.6;
  arm.setJointValueTarget(q);
  if (!ok(arm.move(), log, "move"))
  {
    return finish(1);
  }
  printJoints(arm, log, "arrived");
  wait();

  // ---------------------------------------------------------------- 3. asyncExecute + stop
  RCLCPP_INFO(log, "[3] asyncExecute -> stop() after 1 s");
  arm.setMaxVelocityScalingFactor(0.05);  // 천천히 → 도중에 멈추는 것이 보이게
  q[0] = kStartQ[0] - 0.6;
  arm.setStartStateToCurrentState();
  arm.setJointValueTarget(q);
  if (ok(arm.plan(plan), log, "plan"))
  {
    RCLCPP_INFO(log, "  duration %.2f s", durationOf(plan.trajectory_));
    arm.asyncExecute(plan);  // 바로 반환
    std::this_thread::sleep_for(std::chrono::seconds(1));
    arm.stop();
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    printJoints(arm, log, "stopped");
  }
  arm.setMaxVelocityScalingFactor(0.2);
  wait();

  // ---------------------------------------------------------------- 4. 그리퍼
  RCLCPP_INFO(log, "[4] Gripper open -> close (SRDF named states)");
  gripper.setNamedTarget("open");
  ok(gripper.move(), log, "gripper open");
  wait();
  gripper.setNamedTarget("close");
  ok(gripper.move(), log, "gripper close");
  wait();

  // ---------------------------------------------------------------- 5. 카테시안 경로
  RCLCPP_INFO(log, "[5] Cartesian path: straight line z -0.08 m from the start posture");
  arm.setStartStateToCurrentState();
  arm.setJointValueTarget(kStartQ);
  if (!ok(arm.move(), log, "back to start"))
  {
    return finish(1);
  }
  auto target = arm.getCurrentPose("link6").pose;
  target.position.z -= 0.08;
  // 속도 스케일링 — 같은 경로를 스케일 1.0 / 0.2 로 계산해 비교
  // - Humble(2.5.x): computeCartesianPath 도 setMax*ScalingFactor 를 받음
  // - ⚠ 이 레포 joint_limits.yaml: has_acceleration_limits: false
  //   - → 가속도 1 rad/s² 고정, 가속도 스케일 무시
  //   - → 짧은 이동(가속 구간이 지배)은 스케일을 바꿔도 소요 시간 같음
  //   - 스케일이 먹게 하려면 joint_limits.yaml 에 max_acceleration 지정 (README 참조)
  // - 실물 속도 상한은 어댑터 speed_percent 가 따로 걸고 있음
  moveit_msgs::msg::RobotTrajectory fast;
  arm.setMaxVelocityScalingFactor(1.0);
  arm.setMaxAccelerationScalingFactor(1.0);
  arm.computeCartesianPath({ target }, 0.005, 0.0, fast);
  arm.setMaxVelocityScalingFactor(0.2);
  arm.setMaxAccelerationScalingFactor(0.2);

  moveit_msgs::msg::RobotTrajectory cart;
  const double fraction = arm.computeCartesianPath({ target }, 0.005 /*eef_step [m]*/,
                                                   0.0 /*jump_threshold: 0 = 끔*/, cart);
  RCLCPP_INFO(log, "  fraction = %.3f", fraction);
  RCLCPP_INFO(log, "  duration: scale 1.0 -> %.2f s · 0.2 -> %.2f s", durationOf(fast),
              durationOf(cart));
  if (fraction < 0.999)
  {
    RCLCPP_WARN(log, "  did not reach the end -> not executing");
    return finish(0);
  }
  ok(arm.execute(cart), log, "cartesian execute");
  wait();

  // ---------------------------------------------------------------- 복귀
  RCLCPP_INFO(log, "[end] Back to the start posture");
  arm.setStartStateToCurrentState();
  arm.setJointValueTarget(kStartQ);
  ok(arm.move(), log, "return");

  return finish(0);
}

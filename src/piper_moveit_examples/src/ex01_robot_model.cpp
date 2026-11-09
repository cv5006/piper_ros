// ex01 — 로봇 모델 (URDF + SRDF)
//
// 보는 것
// - SRDF 가 정의한 것: 그룹 · named state · 충돌 제외 쌍
// - 그룹의 끝 링크 = MoveIt 이 목표 자세를 맞추는 링크 (piper: link6, 손끝 아님)
// - FK: 관절값 → 링크 자세
// - IK: 링크 자세 → 관절값 (kinematics.yaml 의 솔버, piper: KDL)
//
// 실행
// - ros2 launch piper_moveit_examples stack.launch.py
// - ros2 launch piper_moveit_examples example.launch.py ex:=ex1
//
// 팔을 움직이지 않음

#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>

using moveit::planning_interface::MoveGroupInterface;

namespace
{
std::string join(const std::vector<std::string>& v)
{
  std::string s;
  for (const auto& e : v)
  {
    s += (s.empty() ? "" : ", ") + e;
  }
  return s;
}

void printPose(const rclcpp::Logger& log, const char* label, const Eigen::Isometry3d& T)
{
  const Eigen::Vector3d p = T.translation();
  const Eigen::Quaterniond q(T.rotation());
  RCLCPP_INFO(log, "  %s  p = [%.4f %.4f %.4f]  q(xyzw) = [%.3f %.3f %.3f %.3f]",
              label, p.x(), p.y(), p.z(), q.x(), q.y(), q.z(), q.w());
}
}  // namespace

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions opt;
  opt.automatically_declare_parameters_from_overrides(true);
  auto node = rclcpp::Node::make_shared("ex01_robot_model", opt);
  const auto log = node->get_logger();

  // MoveGroupInterface 는 콜백(상태 수신)이 돌아야 동작 → 별도 스레드에서 spin
  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(node);
  std::thread spinner([&exec]() { exec.spin(); });

  MoveGroupInterface arm(node, "arm");
  const moveit::core::RobotModelConstPtr model = arm.getRobotModel();

  // ---------------------------------------------------------------- 1. 모델 전체
  RCLCPP_INFO(log, "[1] Model");
  RCLCPP_INFO(log, "  name: %s", model->getName().c_str());
  RCLCPP_INFO(log, "  model frame: %s", model->getModelFrame().c_str());
  RCLCPP_INFO(log, "  joints (all): %s", join(model->getJointModelNames()).c_str());

  // ---------------------------------------------------------------- 2. 그룹 (SRDF)
  RCLCPP_INFO(log, "[2] Groups — SRDF <group>");
  for (const auto* g : model->getJointModelGroups())
  {
    RCLCPP_INFO(log, "  %s", g->getName().c_str());
    RCLCPP_INFO(log, "    joints: %s", join(g->getActiveJointModelNames()).c_str());
    RCLCPP_INFO(log, "    links: %s", join(g->getLinkModelNames()).c_str());
    RCLCPP_INFO(log, "    named state: %s", join(g->getDefaultStateNames()).c_str());
  }

  // ---------------------------------------------------------------- 3. 끝 링크
  RCLCPP_INFO(log, "[3] Link that pose targets apply to");
  RCLCPP_INFO(log, "  arm.getEndEffectorLink() = %s", arm.getEndEffectorLink().c_str());
  RCLCPP_INFO(log, "  - no <end_effector> in SRDF -> tip of the group chain (link6)");
  RCLCPP_INFO(log, "  - the fingertip (TCP) is further out -> corrected in ex3");

  // ---------------------------------------------------------------- 4. 충돌 제외 쌍
  const auto& pairs = model->getSRDF()->getDisabledCollisionPairs();
  RCLCPP_INFO(log, "[4] Collision-check exclusions — SRDF <disable_collisions>: %zu pairs", pairs.size());
  for (size_t i = 0; i < pairs.size() && i < 5; ++i)
  {
    RCLCPP_INFO(log, "  %s - %s (%s)", pairs[i].link1_.c_str(), pairs[i].link2_.c_str(),
                pairs[i].reason_.c_str());
  }
  RCLCPP_INFO(log, "  ... (first 5 only)");

  // ---------------------------------------------------------------- 5. 관절 한계
  RCLCPP_INFO(log, "[5] Joint limits [rad] — URDF <limit>");
  const auto* jmg = model->getJointModelGroup("arm");
  for (const auto* j : jmg->getActiveJointModels())
  {
    const auto& b = j->getVariableBounds()[0];
    RCLCPP_INFO(log, "  %s  [%+.3f, %+.3f]  max vel %.3f rad/s", j->getName().c_str(),
                b.min_position_, b.max_position_, b.max_velocity_);
  }

  // ---------------------------------------------------------------- 6. 현재 상태 + FK
  RCLCPP_INFO(log, "[6] Current state and FK");
  moveit::core::RobotStatePtr state = arm.getCurrentState(5.0);  // /joint_states 수신 대기
  if (!state)
  {
    RCLCPP_ERROR(log, "  no current state received -> is the stack running?");
    rclcpp::shutdown();
    spinner.join();
    return 1;
  }
  std::vector<double> q;
  state->copyJointGroupPositions(jmg, q);
  RCLCPP_INFO(log, "  q = [%.3f %.3f %.3f %.3f %.3f %.3f]", q[0], q[1], q[2], q[3], q[4], q[5]);
  const Eigen::Isometry3d T_link6 = state->getGlobalLinkTransform("link6");
  printPose(log, "FK link6       ", T_link6);
  printPose(log, "FK gripper_base", state->getGlobalLinkTransform("gripper_base"));
  RCLCPP_INFO(log, "  - gripper_base = link6 (fixed joint, zero offset) -> fingertip is ~0.136 m along z");

  // ---------------------------------------------------------------- 7. IK
  RCLCPP_INFO(log, "[7] IK — recover joint values from the link6 pose of another posture q_ref");
  const std::vector<double> q_ref = { 0.4, 1.0, -1.1, 0.3, 0.5, 0.2 };
  moveit::core::RobotState ref(*state);
  ref.setJointGroupPositions(jmg, q_ref);
  ref.update();
  const Eigen::Isometry3d T_goal = ref.getGlobalLinkTransform("link6");
  printPose(log, "goal link6     ", T_goal);

  moveit::core::RobotState probe(*state);  // 초기값(seed) = 현재 자세
  if (probe.setFromIK(jmg, T_goal, 0.1 /*시간 제한 [s]*/))
  {
    std::vector<double> q_ik;
    probe.copyJointGroupPositions(jmg, q_ik);
    RCLCPP_INFO(log, "  q_ref = [%.3f %.3f %.3f %.3f %.3f %.3f]", q_ref[0], q_ref[1], q_ref[2],
                q_ref[3], q_ref[4], q_ref[5]);
    RCLCPP_INFO(log, "  q_ik  = [%.3f %.3f %.3f %.3f %.3f %.3f]", q_ik[0], q_ik[1], q_ik[2],
                q_ik[3], q_ik[4], q_ik[5]);
    printPose(log, "FK(q_ik) link6 ", probe.getGlobalLinkTransform("link6"));
    RCLCPP_INFO(log, "  - same pose, but q_ik may differ from q_ref: multiple solutions (elbow / wrist flip)");
    RCLCPP_INFO(log, "  - which solution you get depends on the seed");
  }
  else
  {
    RCLCPP_WARN(log, "  IK failed — numerical solver (KDL): may fail depending on seed / timeout");
  }

  rclcpp::shutdown();
  spinner.join();
  return 0;
}

// 같은 두 자세, 두 경로 — 관절 보간과 손끝 직선. (M4-1)
//
// **두 직선은 같은 동작이 아니다.**
//
//   ① 관절 보간 (OMPL)                 관절 공간의 길. 손끝 자취는 휜다
//   ② 손끝 직선 (computeCartesianPath)  작업 공간의 직선. 관절값은 제멋대로
//   ③ 작업영역 밖으로 ②를 요구          다 못 간다. fraction 이 숫자로 말한다
//
// 플래너는 OMPL 하나. 직선은 플래너가 아니라 MoveIt 의 카테시안 보간이 만든다.
// MoveIt 스택을 먼저 띄울 것. 기동 시 'No kinematics plugins defined' 경고 한 줄은
// 자기 쪽 로봇 모델 때문이고 결과에 영향 없다.
//
//   ros2 run piper_lecture_demo path_compare

#include <chrono>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/robot_state/robot_state.h>

#include <geometry_msgs/msg/pose.hpp>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

using namespace std::chrono_literals;
using moveit::planning_interface::MoveGroupInterface;
using moveit::planning_interface::PlanningSceneInterface;

namespace
{
const char* kGroup = "arm";

// 관절 한계에서 떨어진, 데모하기 좋은 시작 자세
const std::vector<double> kStartQ = { 0.0, 0.9, -0.9, 0.0, 0.6, 0.0 };
// 팔을 반경 0.383 m 로 뻗은 자세. 여기서 바깥으로 0.25 m 를 더 요구하면
// 목표가 도달 한계(URDF 링크 길이로 0.627 m)를 넘는다 -> ③ 이 된다.
const std::vector<double> kStretchedQ = { 0.0, 1.70, -1.38, 0.0, 0.5, 0.0 };

void pause_for(const rclcpp::Logger& log, const char* what, int seconds)
{
  RCLCPP_INFO(log, "  --- %s (waiting %d s) ---", what, seconds);
  std::this_thread::sleep_for(std::chrono::seconds(seconds));
}

// 궤적의 각 경유점에서 손끝이 어디 있었는지를 뽑는다.
std::vector<Eigen::Vector3d> traceOf(const moveit_msgs::msg::RobotTrajectory& traj,
                                     const moveit::core::RobotModelConstPtr& model,
                                     const std::string& group,
                                     const std::string& tip)
{
  std::vector<Eigen::Vector3d> trace;
  moveit::core::RobotState state(model);
  state.setToDefaultValues();
  const auto* jmg = model->getJointModelGroup(group);
  for (const auto& pt : traj.joint_trajectory.points)
  {
    state.setJointGroupPositions(jmg, pt.positions);
    state.update();
    trace.push_back(state.getGlobalLinkTransform(tip).translation());
  }
  return trace;
}

// 관절 목표로 보낸다. 실패를 조용히 넘기면 뒤 단계의 시작 자세가 달라져
// 결과 해석이 통째로 틀어진다 — 실제로 한 번 겪었다.
bool moveToJoints(MoveGroupInterface& arm, const std::vector<double>& q,
                  const rclcpp::Logger& log, const char* label)
{
  MoveGroupInterface::Plan plan;
  arm.setJointValueTarget(q);
  if (arm.plan(plan) != moveit::core::MoveItErrorCode::SUCCESS)
  {
    RCLCPP_ERROR(log, "    [%s] planning to the joint goal failed - do not trust what follows", label);
    return false;
  }
  if (arm.execute(plan) != moveit::core::MoveItErrorCode::SUCCESS)
  {
    RCLCPP_ERROR(log, "    [%s] execution failed", label);
    return false;
  }
  return true;
}

double pathLength(const std::vector<Eigen::Vector3d>& trace)
{
  double d = 0.0;
  for (size_t i = 1; i < trace.size(); ++i)
  {
    d += (trace[i] - trace[i - 1]).norm();
  }
  return d;
}

// 직선에서 가장 많이 벗어난 거리. 「휘었다」를 숫자로 만든다.
double maxDeviation(const std::vector<Eigen::Vector3d>& trace)
{
  if (trace.size() < 2)
  {
    return 0.0;
  }
  const Eigen::Vector3d a = trace.front();
  const Eigen::Vector3d b = trace.back();
  const double len = (b - a).norm();
  if (len < 1e-9)
  {
    return 0.0;
  }
  const Eigen::Vector3d dir = (b - a) / len;
  double worst = 0.0;
  for (const auto& p : trace)
  {
    const Eigen::Vector3d v = p - a;
    worst = std::max(worst, (v - dir * v.dot(dir)).norm());
  }
  return worst;
}
}  // namespace

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  options.automatically_declare_parameters_from_overrides(true);
  auto node = rclcpp::Node::make_shared("path_compare", options);
  const auto log = node->get_logger();

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::thread spinner([&executor]() { executor.spin(); });

  MoveGroupInterface arm(node, kGroup);
  PlanningSceneInterface scene;

  // obstacle 데모가 세워둔 기둥이 남아 있으면 이 데모의 자세가 막힌다. 먼저 치운다.
  {
    std::vector<std::string> stale;
    for (const auto& name : scene.getKnownObjectNames())
    {
      if (name.rfind("lecture_", 0) == 0)
      {
        stale.push_back(name);
      }
    }
    if (!stale.empty())
    {
      RCLCPP_INFO(log, "removing %zu object(s) left behind by another demo", stale.size());
      scene.removeCollisionObjects(stale);
      std::this_thread::sleep_for(1s);
    }
  }

  arm.setPlanningPipelineId("ompl");
  // setPlannerId 는 부르지 않는다 — 이 레포에는 ompl_planning.yaml 이 없어
  // 그룹별 플래너 설정이 비어 있고, 이름을 주면 move_group 이 무시한다 (기본값 RRTConnect).
  arm.setMaxVelocityScalingFactor(0.3);
  arm.setMaxAccelerationScalingFactor(0.3);
  arm.setPlanningTime(5.0);

  const auto model = arm.getRobotModel();
  const std::string tip = arm.getEndEffectorLink();

  auto marker_pub = node->create_publisher<visualization_msgs::msg::MarkerArray>(
      "lecture_markers", rclcpp::QoS(1).transient_local());

  visualization_msgs::msg::MarkerArray arr;
  auto lineMarker = [&](int id, const std::vector<Eigen::Vector3d>& trace,
                        double r, double g, double b, const std::string& ns) {
    visualization_msgs::msg::Marker m;
    m.header.frame_id = arm.getPlanningFrame();
    m.header.stamp = node->now();
    m.ns = ns;
    m.id = id;
    m.type = visualization_msgs::msg::Marker::LINE_STRIP;
    m.action = visualization_msgs::msg::Marker::ADD;
    m.pose.orientation.w = 1.0;
    m.scale.x = 0.006;
    m.color.r = static_cast<float>(r);
    m.color.g = static_cast<float>(g);
    m.color.b = static_cast<float>(b);
    m.color.a = 0.95f;
    for (const auto& p : trace)
    {
      geometry_msgs::msg::Point q;
      q.x = p.x();
      q.y = p.y();
      q.z = p.z();
      m.points.push_back(q);
    }
    return m;
  };

  RCLCPP_INFO(log, "=========================================================");
  RCLCPP_INFO(log, " path_compare - two paths between the same two poses. both were called a straight line");
  RCLCPP_INFO(log, "=========================================================");

  // ---------------------------------------------------------------- 시작 자세
  RCLCPP_INFO(log, "[0] moving to the start pose");
  MoveGroupInterface::Plan plan;
  if (!moveToJoints(arm, kStartQ, log, "start"))
  {
    RCLCPP_ERROR(log, "check that move_group is up and the planning scene is empty");
    rclcpp::shutdown();
    spinner.join();
    return 1;
  }

  const auto start_pose = arm.getCurrentPose(tip).pose;
  auto goal = start_pose;
  goal.position.y += 0.22;
  goal.position.z -= 0.12;

  RCLCPP_INFO(log, "    start [%.3f %.3f %.3f]  ->  goal [%.3f %.3f %.3f]",
              start_pose.position.x, start_pose.position.y, start_pose.position.z,
              goal.position.x, goal.position.y, goal.position.z);
  const double straight = std::hypot(goal.position.y - start_pose.position.y,
                                     goal.position.z - start_pose.position.z);
  RCLCPP_INFO(log, "    straight-line distance between the two points = %.4f m", straight);

  // ------------------------------------------------ ① 관절 공간에서 이은 길
  pause_for(log, "[1] joint interpolation - a path found in joint space (OMPL)", 2);
  arm.setPoseTarget(goal);
  std::vector<Eigen::Vector3d> trace_joint;
  if (arm.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS)
  {
    trace_joint = traceOf(plan.trajectory_, model, kGroup, tip);
    RCLCPP_INFO(log, "    %zu waypoints", trace_joint.size());
    RCLCPP_INFO(log, "    TCP path length = %.4f m  (%.1f%% of the straight line)",
                pathLength(trace_joint), 100.0 * pathLength(trace_joint) / straight);
    RCLCPP_INFO(log, "    max deviation from the straight line = %.4f m", maxDeviation(trace_joint));
    arr.markers.push_back(lineMarker(0, trace_joint, 0.95, 0.55, 0.15, "joint_interp"));
    marker_pub->publish(arr);
    arm.execute(plan);
  }
  else
  {
    RCLCPP_WARN(log, "    planning failed");
  }

  // ------------------------------------------------ ② 작업 공간에서 이은 직선
  pause_for(log, "[2] straight TCP line - joined straight in task space (computeCartesianPath)", 2);
  moveToJoints(arm, kStartQ, log, "return to start");

  std::vector<geometry_msgs::msg::Pose> waypoints = { goal };
  moveit_msgs::msg::RobotTrajectory cartesian;
  const double fraction = arm.computeCartesianPath(waypoints, 0.005, 0.0, cartesian);
  const auto trace_cart = traceOf(cartesian, model, kGroup, tip);

  RCLCPP_INFO(log, "    fraction = %.3f  (covered %.1f%% of the requested line)", fraction, fraction * 100.0);
  if (!trace_cart.empty())
  {
    RCLCPP_INFO(log, "    %zu waypoints", trace_cart.size());
    RCLCPP_INFO(log, "    TCP path length = %.4f m", pathLength(trace_cart));
    RCLCPP_INFO(log, "    max deviation from the straight line = %.4f m  <- compare with [1]",
                maxDeviation(trace_cart));
    arr.markers.push_back(lineMarker(1, trace_cart, 0.25, 0.60, 0.95, "cartesian_line"));
    marker_pub->publish(arr);
  }
  if (fraction > 0.99)
  {
    arm.execute(cartesian);
  }
  else
  {
    RCLCPP_WARN(log, "    could not cover the whole line. skipping execution");
  }

  // --------------------------------------- ③ 특이점 근처에서 같은 것을 요구하면
  pause_for(log, "[3] asking for a straight line towards the workspace boundary", 2);
  if (!moveToJoints(arm, kStretchedQ, log, "extended pose"))
  {
    RCLCPP_ERROR(log, "    skipping [3]");
    rclcpp::shutdown();
    spinner.join();
    return 1;
  }

  const auto stretched_pose = arm.getCurrentPose(tip).pose;
  auto far_goal = stretched_pose;
  far_goal.position.x += 0.25;   // 도달 한계 너머로 곧게 민다

  RCLCPP_INFO(log, "    start [%.3f %.3f %.3f]  ->  goal [%.3f %.3f %.3f]  (line %.3f m)",
              stretched_pose.position.x, stretched_pose.position.y, stretched_pose.position.z,
              far_goal.position.x, far_goal.position.y, far_goal.position.z, 0.25);

  std::vector<geometry_msgs::msg::Pose> far_waypoints = { far_goal };
  moveit_msgs::msg::RobotTrajectory far_cartesian;
  const double far_fraction = arm.computeCartesianPath(far_waypoints, 0.005, 0.0, far_cartesian);
  const auto trace_far = traceOf(far_cartesian, model, kGroup, tip);

  RCLCPP_INFO(log, "    fraction = %.3f  (stopped at %.1f%% of the requested line)",
              far_fraction, far_fraction * 100.0);
  if (!trace_far.empty())
  {
    const double r0 = std::hypot(stretched_pose.position.x, stretched_pose.position.y);
    const double r1 = std::hypot(trace_far.back().x(), trace_far.back().y());
    RCLCPP_INFO(log, "    distance actually travelled = %.4f m / requested %.4f m",
                pathLength(trace_far), 0.25);
    RCLCPP_INFO(log, "    TCP radius %.3f m -> stopped at %.3f m", r0, r1);
    RCLCPP_INFO(log, "    NOTE position alone, this arm reaches a radius of 0.627 m (from the URDF).");
    RCLCPP_INFO(log, "      Yet it stopped at %.3f m, because Cartesian interpolation also holds", r1);
    RCLCPP_INFO(log, "      the TCP orientation fixed. Reaching a point and reaching it in the"
              " pose you want are different things - M3.");
    arr.markers.push_back(lineMarker(2, trace_far, 0.90, 0.25, 0.25, "beyond_limit"));
    marker_pub->publish(arr);
  }
  if (far_fraction > 0.0)
  {
    arm.execute(far_cartesian);
  }

  RCLCPP_INFO(log, " ");
  RCLCPP_INFO(log, " Summary - a straight line in joint space and one in task space are"
              " different motions.");
  RCLCPP_INFO(log, " And a straight line in task space is not always reachable.");
  RCLCPP_INFO(log, " Compare the traces in the rviz Marker display: orange (joint),"
              " blue (straight), red (limit).");

  rclcpp::shutdown();
  spinner.join();
  return 0;
}

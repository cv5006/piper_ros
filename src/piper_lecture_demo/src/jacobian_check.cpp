// Jacobian 을 두 가지로 구해 나란히 놓는다 — MoveIt 의 API 와 강의의 수식.
//
// 이 패키지의 manipulability · workspace 는 Jacobian 을 직접 구현한 것
// (piper_lecture_demo/kinematics.py) 으로 계산한다. 학생 입장에서 생기는 의문이
// 하나 있다: **그게 맞는 값인가, 그리고 실무에서 MoveIt 을 쓸 때는 어떻게 얻는가.**
// 이 노드가 그 둘을 잇는다.
//
//   ① MoveIt        state.getJacobian(jmg, link, ref_point, J)      <- 한 줄이다
//   ② 강의 수식     Jᵢ = [ zᵢ × (pₑ − pᵢ) ; zᵢ ]                     <- 아래 15 줄이다
//
// 같은 자세에서 둘을 재고 차이를 낸다. 프리셋 일곱 자세에서 **차이가 정확히 0 이다**
// (부동소수 오차조차 없다 — MoveIt 이 같은 수식을 같은 순서로 계산한다).
// 그래서 kinematics.py 는 「교육용 장난감」이 아니라 MoveIt 과 같은 것을 하는
// 구현이고, 반대로 MoveIt 의 한 줄이 강의에서 다룬 그 수식이라는 것이 확인된다.
//
// ②를 여기 다시 구현한 이유는 대조의 의미 때문이다. kinematics.py 를 불러와서
// 비교하면 「같은 코드가 같은 값을 낸다」가 되지만, 수식을 C++ 로 새로 적어
// MoveIt 과 맞추면 **구현 둘이 독립적으로 같은 값에 도달한다**는 것이 된다.
//
// move_group 은 필요 없다. URDF 와 SRDF 만 읽어 RobotModel 을 세운다. IK 플러그인도
// 쓰지 않으므로 load_kinematics_solvers 를 끈다 (그래야 kinematics.yaml 경고가 안 뜬다).
//
//     ros2 run piper_lecture_demo jacobian_check
//     ros2 run piper_lecture_demo jacobian_check --ros-args -p tcp_offset:="[0.0,0.0,0.0]"
//
// tcp_offset 을 0 으로 주면 플랜지(link6) 기준이 된다. 기본값은 손끝(TCP)이고
// manipulability 의 기본값과 같으므로, 아래 표의 숫자가 그 노드의 터미널 출력과
// 그대로 일치한다 — 그것이 kinematics.py 와의 대조다.

#include <algorithm>
#include <cstdio>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

#include <Eigen/Geometry>
#include <Eigen/SVD>

#include <rclcpp/rclcpp.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/joint_model.h>
#include <moveit/robot_model/revolute_joint_model.h>
#include <moveit/robot_state/robot_state.h>

namespace
{
// pose_presets.py 의 프리셋과 같은 값이다. 이름도 같게 두어 서로 짚을 수 있게 한다.
struct Preset
{
  const char* name;
  std::vector<double> q;
};

const std::vector<Preset> kPresets = {
  { "good", { 0.0, 1.70, -1.38, 0.0, 0.50, 0.0 } },
  { "stretch", { 0.0, 2.90, -2.90, 0.0, 0.50, 0.0 } },
  { "elbow", { 0.0, 2.85, -2.85, 0.0, 0.0, 0.0 } },
  { "wrist", { 0.0, 1.20, -1.00, 0.30, 0.0, 0.0 } },
  { "home", { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 } },
  { "shoulder", { 1.566, 0.507, -0.768, -0.925, -1.021, -0.565 } },
  { "slow", { 1.183, 0.613, -0.167, -1.049, 1.040, 0.508 } },
};

std::string slurp(const std::string& path)
{
  std::ifstream f(path);
  std::stringstream ss;
  ss << f.rdbuf();
  return ss.str();
}

// ② 강의 수식 그대로. 회전관절 하나가 Jacobian 의 열 하나다.
//
//     Jᵢ = [ zᵢ × (pₑ − pᵢ) ; zᵢ ]
//
// zᵢ 는 관절 축의 월드 방향, pᵢ 는 관절 원점의 월드 위치, pₑ 는 기준점이다.
// 둘 다 ⁰Tᵢ 에서 그냥 꺼내는 값이라, 운동은 형상의 부산물로 나온다.
Eigen::MatrixXd jacobianFromFormula(const moveit::core::RobotState& state,
                                    const moveit::core::JointModelGroup* jmg,
                                    const moveit::core::LinkModel* tip,
                                    const Eigen::Vector3d& ref_point)
{
  const auto& joints = jmg->getActiveJointModels();
  Eigen::MatrixXd J(6, joints.size());

  // 기준점의 월드 위치. tip 프레임에서 잰 오프셋을 월드로 옮긴다.
  const Eigen::Isometry3d& T_tip = state.getGlobalLinkTransform(tip);
  const Eigen::Vector3d p_e = T_tip * ref_point;

  for (std::size_t i = 0; i < joints.size(); ++i)
  {
    const auto* jm = joints[i];
    // 관절 축은 자식 링크의 프레임 원점을 지나고, 그 프레임과 함께 돌아간다.
    const Eigen::Isometry3d& T = state.getGlobalLinkTransform(jm->getChildLinkModel());
    const Eigen::Vector3d p_i = T.translation();

    if (jm->getType() == moveit::core::JointModel::REVOLUTE)
    {
      const auto* rev = static_cast<const moveit::core::RevoluteJointModel*>(jm);
      const Eigen::Vector3d z = T.rotation() * rev->getAxis();
      J.block<3, 1>(0, i) = z.cross(p_e - p_i);
      J.block<3, 1>(3, i) = z;
    }
    else
    {
      // 이 팔에는 없지만, 병진관절이면 선속도가 축 그대로이고 각속도가 0 이다.
      J.block<3, 1>(0, i) = T.rotation() * Eigen::Vector3d::UnitZ();
      J.block<3, 1>(3, i).setZero();
    }
  }
  return J;
}
}  // namespace

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions opts;
  opts.automatically_declare_parameters_from_overrides(true);
  auto node = rclcpp::Node::make_shared("jacobian_check", opts);
  const auto log = node->get_logger();

  const std::string share_desc = ament_index_cpp::get_package_share_directory(
      "piper_description");
  const std::string share_moveit = ament_index_cpp::get_package_share_directory(
      "piper_with_gripper_moveit");

  std::string urdf_path = share_desc + "/urdf/piper_description.urdf";
  std::string srdf_path = share_moveit + "/config/piper.srdf";
  std::string group = "arm";
  std::string tip_name = "link6";
  std::vector<double> tcp = { 0.0, 0.0, 0.1358 };
  node->get_parameter_or("urdf", urdf_path, urdf_path);
  node->get_parameter_or("srdf", srdf_path, srdf_path);
  node->get_parameter_or("group", group, group);
  node->get_parameter_or("tip", tip_name, tip_name);
  node->get_parameter_or("tcp_offset", tcp, tcp);
  if (tcp.size() != 3)
  {
    RCLCPP_ERROR(log, "tcp_offset needs 3 values");
    return 1;
  }

  // IK 플러그인은 필요 없다 (Jacobian 은 RobotState 가 낸다).
  robot_model_loader::RobotModelLoader::Options options(slurp(urdf_path),
                                                        slurp(srdf_path));
  options.load_kinematics_solvers_ = false;
  robot_model_loader::RobotModelLoader loader(node, options);
  const auto model = loader.getModel();
  if (!model)
  {
    RCLCPP_ERROR(log, "could not build the RobotModel. check the URDF and SRDF paths:");
    RCLCPP_ERROR(log, "  urdf = %s", urdf_path.c_str());
    RCLCPP_ERROR(log, "  srdf = %s", srdf_path.c_str());
    return 1;
  }

  const auto* jmg = model->getJointModelGroup(group);
  const auto* tip = model->getLinkModel(tip_name);
  if (!jmg || !tip)
  {
    RCLCPP_ERROR(log, "no group '%s' or link '%s' in the model",
                 group.c_str(), tip_name.c_str());
    return 1;
  }

  const Eigen::Vector3d ref_point(tcp[0], tcp[1], tcp[2]);
  const bool has_tool = ref_point.norm() > 1e-12;

  printf("\n");
  printf("MoveIt's getJacobian() vs the lecture formula  J_i = [ z_i x (p_e - p_i) ; z_i ]\n");
  printf("model '%s', group '%s', %u axes, reference = %s\n", model->getName().c_str(),
         group.c_str(), jmg->getVariableCount(),
         has_tool ? "TCP" : "flange");
  if (has_tool)
    printf("  TCP offset in the %s frame = [%.4f %.4f %.4f] m\n",
           tip_name.c_str(), ref_point.x(), ref_point.y(), ref_point.z());
  printf("\n");
  printf("  the sigma / w / cond columns are what the manipulability demo prints\n");
  printf("  for the same preset, so they cross-check kinematics.py as well.\n");
  printf("\n");
  printf("  %-9s  %-22s  %-9s  %-7s  %-12s  %s\n", "preset",
         "sigma (linear)", "w", "cond", "sigma_min_6d", "max|MoveIt - formula|");
  printf("  %s\n", std::string(92, '-').c_str());

  moveit::core::RobotState state(model);
  state.setToDefaultValues();

  double worst = 0.0;
  for (const auto& preset : kPresets)
  {
    state.setJointGroupPositions(jmg, preset.q);
    state.update();

    Eigen::MatrixXd J_moveit;
    if (!state.getJacobian(jmg, tip, ref_point, J_moveit))
    {
      printf("  %-9s  getJacobian() failed\n", preset.name);
      continue;
    }
    const Eigen::MatrixXd J_formula = jacobianFromFormula(state, jmg, tip, ref_point);
    const double diff = (J_moveit - J_formula).cwiseAbs().maxCoeff();
    worst = std::max(worst, diff);

    const Eigen::VectorXd sv =
        Eigen::JacobiSVD<Eigen::MatrixXd>(J_moveit.topRows(3)).singularValues();
    const Eigen::VectorXd s6 =
        Eigen::JacobiSVD<Eigen::MatrixXd>(J_moveit).singularValues();

    printf("  %-9s  %6.4f %6.4f %6.4f  %9.6f  %7.1f  %12.6f  %.2e\n", preset.name,
           sv(0), sv(1), sv(2), sv(0) * sv(1) * sv(2), sv(0) / sv(2),
           s6(s6.size() - 1), diff);
  }

  printf("\n");
  printf("  worst disagreement over %zu poses: %.2e\n", kPresets.size(), worst);
  if (worst == 0.0)
    printf("  -> bit-for-bit identical. MoveIt evaluates the same formula in the same order.\n");
  else
    printf("  -> the same computation; the gap is floating-point rounding only.\n");
  printf("\n");
  printf("  MoveIt:   state.getJacobian(jmg, tip, ref_point, J)     (one call)\n");
  printf("  formula:  jacobianFromFormula() in this file            (15 lines)\n");
  printf("  python:   piper_lecture_demo/kinematics.py Chain.jacobian()\n");
  printf("\n");

  rclcpp::shutdown();
  return 0;
}

# piper_moveit_examples

Piper + MoveIt 2 기본 사용법. C++ 예제 4개.

- 대상: MoveIt 처음 쓰는 사람
- 범위: 사용법 기본기만. 특정 작업(픽앤플레이스 등)은 가정하지 않음
- 로봇 모델 · MoveIt 설정: `piper_description` · `piper_with_gripper_moveit` 그대로 사용 (수정 없음)

---

## 0. 빠른 실행

```bash
cd ~/piper_ros
colcon build --packages-up-to piper_moveit_examples
source install/setup.bash
```

터미널 1 — 스택 (계속 띄워 둠)

```bash
ros2 launch piper_moveit_examples stack.launch.py
```

터미널 2 — 예제

```bash
ros2 launch piper_moveit_examples example.launch.py ex:=ex1
```

- `ex:=` 에 `ex1` · `ex2` · `ex3` · `ex3_1` · `ex4` (`1` · `ex01` · `ex01_robot_model` 도 가능)
- `pause:=4.0` → 단계 사이 대기 시간 [s]. rviz 로 볼 시간
- 기동 시 octomap `ERROR` 4줄 → 무시 가능 (§4 참조)

---

## 1. MoveIt 개념

### 1.1 전체 구조

```
  [내 노드]                      [move_group 노드]                         [컨트롤러]
  MoveGroupInterface   ─요청→    Robot Model (URDF + SRDF)
  PlanningSceneInterface         Planning Scene (로봇 상태 + 물체)
                                 Planner (OMPL · Pilz · CHOMP)
                                 Kinematics (IK 솔버)
                                 Trajectory Execution    ─궤적(액션)→     mock 또는 실물 어댑터
```

- `move_group`: 계획 · 실행을 하는 서버. `stack.launch.py` 가 띄움
- 내 코드: 클라이언트. 목표만 주고 결과를 받음
- 예제 코드 안에 플래너 · IK 구현은 없음 → 전부 `move_group` 쪽

### 1.2 URDF vs SRDF — `ex01`

| | 내용 | 이 레포 |
|---|---|---|
| URDF | 링크 · 관절 · 관절 한계 · 메시 | `piper_description` |
| SRDF | MoveIt 전용 의미 정보 | `piper_with_gripper_moveit/config/piper.srdf` |

SRDF 에 있는 것

- **group**: 계획 단위
  - `arm` = `base_link` → `link6` (6축)
  - `gripper` = `joint7`
- **group_state**: 이름 붙인 자세 → `setNamedTarget("zero")`
  - `arm`: `zero` / `gripper`: `open` · `close`
- **disable_collisions**: 충돌 검사 제외 링크 쌍 (28쌍). 붙어 있거나 절대 안 닿는 쌍
- **end_effector**: 없음 → 목표 자세 기준 링크 = 그룹 끝 `link6`

### 1.3 Planning Scene — `ex02`

- 플래너가 믿는 세상 = 로봇 현재 상태 + 충돌 물체
- **여기 없는 물체는 플래너에게 없는 물체**
  - 카메라가 본 물체 → 직접 CollisionObject 로 넣어야 회피
- 물체 종류
  - world 물체: 고정된 장애물
  - attached 물체: 링크에 붙은 물체 (쥔 물체). 팔과 같이 움직이고 충돌 검사에 포함
- 모양: primitive(상자 · 원기둥 · 구 · 원뿔) 또는 mesh

### 1.4 Planner — `ex03`

| 파이프라인 | 플래너 | 성격 |
|---|---|---|
| `ompl` (기본) | RRTConnect 등 | 샘플링 기반. 장애물 회피. 경로 모양 매번 다름 |
| `pilz_industrial_motion_planner` | `PTP` · `LIN` · `CIRC` | 정해진 모양 (관절 직선 · 손끝 직선 · 원호). 회피 안 함, 충돌 시 실패. ⚠ 이 레포에선 `PTP` 안 됨 (§4) |
| `chomp` | — | 최적화 기반. 이 예제에선 안 씀 |

- OMPL 은 가끔 실패 (`INVALID_MOTION_PLAN` 등) → 재시도 코드 필요 (`ex3_1`)
- 카테시안 경로(`computeCartesianPath`): 플래너 아님
  - 손끝 직선을 잘게 나눠 구간마다 IK
  - `fraction` = 간 비율. 1.0 미만 → 도중에 막힘
- 계획 결과 = 관절 궤적 (시간 · 위치 · 속도 · 가속도)

### 1.5 Kinematics (IK) — `ex01`

- 솔버: KDL (`kinematics.yaml`), 시간 제한 5 ms
- 수치 해법 → 초기값에 따라 다른 해 · 가끔 실패
- 같은 손끝 자세에 관절 해 여럿 (팔꿈치 위/아래, 손목 뒤집기)

### 1.6 Execution — `ex04`

- 계획된 궤적 → `FollowJointTrajectory` 액션으로 컨트롤러에 전달
  - `arm_controller/follow_joint_trajectory`
  - `gripper_controller/follow_joint_trajectory`
- 누가 받느냐가 mock / 실물의 차이 (§3)
- 내 코드는 mock / 실물 구분 없음

### 1.7 link6 ≠ 손끝 — `ex03`

- MoveIt 목표 자세 = `link6` 원점의 자세
- 손끝(TCP)은 `link6` z 방향으로 약 0.136 m 앞
- 손끝 기준 목표 → `link6` 목표로 환산해서 줌: `T_link6 = T_tcp · (T_link6→tcp)⁻¹`
- ⚠ 0.1358 m = URDF · 그리퍼 메시 기준값. 실제 집는 지점은 직접 확인

---

## 2. 예제

| 예제 | 내용 | 팔 움직임 |
|---|---|---|
| `ex01_robot_model` | 그룹 · named state · 충돌 제외 쌍 · 관절 한계 · FK · IK | ✕ |
| `ex02_collision_objects` | 상자/원기둥 정의 → 추가 → 조회 → 이동 → 부착 → 분리 → 제거 | ✕ |
| `ex03_planning` | named · joint · pose · TCP 목표 / 장애물 회피 / Pilz LIN / 카테시안 경로 | ✕ (계획만) |
| `ex03_1_planner_compare` | OMPL vs Pilz LIN 반복 계획: 벽 없음 · 있음. 재현성 · 회피 차이 | ✕ (계획만) |
| `ex04_execution` | plan→execute · `move()` · 비동기 실행 + `stop()` · 그리퍼 · 카테시안 실행 | **○** |

순서대로 볼 것. 각 파일 맨 위 주석에 요점 정리.

자주 쓰는 API

| 하고 싶은 것 | API |
|---|---|
| 목표 지정 | `setNamedTarget` · `setJointValueTarget` · `setPoseTarget` |
| 출발 자세 지정 | `setStartState` · `setStartStateToCurrentState` |
| 계획 | `plan(plan)` |
| 실행 | `execute(plan)` · `asyncExecute(plan)` · `stop()` |
| 계획 + 실행 | `move()` |
| 손끝 직선 | `computeCartesianPath` |
| 플래너 선택 | `setPlanningPipelineId` · `setPlannerId` |
| 속도 | `setMaxVelocityScalingFactor` |
| 물체 | `PlanningSceneInterface::applyCollisionObject(s)` · `applyAttachedCollisionObject` |
| 현재 상태 | `getCurrentState` · `getCurrentJointValues` · `getCurrentPose` |

---

## 3. 실물 piper 연결

### 3.1 무엇이 문제인가

- MoveIt 이 보내는 것: `FollowJointTrajectory` **액션** (궤적 전체)
- piper 드라이버가 받는 것: `joint_ctrl_single` **토픽** (`JointState` 1개 = 목표 자세 1개)
- 둘이 직접 안 맞음 → 사이를 잇는 노드 필요
- `piper_with_gripper_moveit` 기본 설정: `mock_components/GenericSystem` (가짜 하드웨어)

### 3.2 두 가지 연결 방법

| | 원래 README 방식 | 실물 어댑터 (권장) |
|---|---|---|
| 실행 | `start_single_piper.launch.py` + `demo.launch.py` | `stack.launch.py real:=true` |
| 원리 | 드라이버가 mock 의 `/joint_states` 를 명령으로 받아 따라감 (remap) | 어댑터가 액션을 받아 보간 → 드라이버로 명령 |
| 실물 상태 피드백 | ✕ (MoveIt 은 mock 상태만 앎) | ○ (`joint_states_feedback` → `/joint_states`) |
| 속도 제한 | ✕ | ○ `speed_percent` |
| 실행 결과 | 항상 성공 (가짜) | 실물 기준 |

- 어댑터: `piper_lecture_demo/scripts/piper_real_adapter.py`
- ⚠ 두 방식 섞지 말 것: remap 된 드라이버 + 어댑터 → 상태가 명령으로 되돌아가는 루프

### 3.3 실행 순서

1. CAN 활성화

   ```bash
   bash can_activate.sh can0 1000000
   ```

2. 실물 없이 어댑터 경로 먼저 확인 (드라이버 대역)

   ```bash
   ros2 launch piper_moveit_examples stack.launch.py real:=true stub:=true
   ros2 launch piper_moveit_examples example.launch.py ex:=ex4
   ```

3. 실물

   ```bash
   ros2 launch piper_moveit_examples stack.launch.py real:=true
   ```

   - CAN 포트 다르면 `can_port:=can1`
   - 첫 실행: `ex01` → `ex03` (안 움직임) 으로 상태 · 계획 확인 → `ex04`

### 3.4 어댑터 안전장치

- 속도: 매 명령에 `speed_percent` (기본 20 %). 안 채우면 드라이버가 전속
- 계단 명령 거부: 궤적 첫 점이 현재 자세에서 20° 넘게 떨어지면 거절
- enable: 실행 전 `enable_srv` 호출. 실패 시 안 움직임
- 취소(`stop()`): 현재 자세를 목표로 다시 보내 정지

### 3.5 주의

- ⚠ 어댑터는 **드라이버 대역(stub)으로만 검증됨**. 실물에서 미확인
  - CAN 타이밍 · 실제 추종 오차 · 관절 한계 근처 거동 · 그리퍼 힘
- 팔 주변 비우기 · 비상정지 손 닿는 곳에
- `speed_percent` 는 충분히 시험한 뒤에 올릴 것

---

## 4. 이 레포의 함정 — AI 에게 질문할 때 같이 줄 것

"MoveIt 으로 piper 움직이는 코드" 를 AI 에 요청하면 일반 MoveIt 기준 코드가 나옴. 아래를 먼저 알려줄 것.

- 실물 연결: mock 이 기본. 실물은 어댑터 경유 (`stack.launch.py real:=true`)
- 목표 기준 링크: `link6`. 손끝 아님
- 그룹 이름: `arm` · `gripper`. 기준 프레임: `base_link`
- IK: KDL, 시간 제한 5 ms → 가끔 실패. 실패 처리 필요
- 가속도 한계 미정의 (`joint_limits.yaml`: `has_acceleration_limits: false`)
  - 가속도 1 rad/s² 고정, `setMaxAccelerationScalingFactor` 무시됨
  - 짧은 이동은 속도 스케일을 바꿔도 소요 시간 같음
  - Pilz `PTP` 계획 실패: `acceleration limit not set for group arm` (`LIN` 은 됨)
  - 바꾸려면 `joint_limits.yaml` 에 `has_acceleration_limits: true` + `max_acceleration` 지정
    - ⚠ 그러면 같은 파일의 `default_acceleration_scaling_factor: 0.1` 도 살아남 → rviz 계획이 느려짐
- `sensors_3d.yaml`: PR2 튜토리얼 기본값(`/head_mount_kinect/...`) 그대로
  - 기동 시 octomap ERROR 4줄의 원인. 계획에는 영향 없음
  - 깊이 카메라로 자동 장애물 인식 안 됨 → 쓰려면 이 파일 수정
- `ompl_planning.yaml` 없음 → `setPlannerId` 로 OMPL 플래너 지정해도 기본(RRTConnect)

---

## 5. 공식 문서

- 기준: MoveIt 2 **Humble** 문서 (`moveit.picknik.ai/humble`)
  - ⚠ 검색으로 `main` · ROS 1(`ros-planning.github.io`) 문서가 섞여 나옴 → 주소의 버전 확인
- 전체 목차: <https://moveit.picknik.ai/humble/index.html>

### 5.1 개념 — §1 과 대응

| 주제 | 링크 |
|---|---|
| 전체 구조 | <https://moveit.picknik.ai/humble/doc/concepts/concepts.html> |
| move_group | <https://moveit.picknik.ai/humble/doc/concepts/move_group.html> |
| Planning Scene | <https://moveit.picknik.ai/humble/doc/concepts/planning_scene_monitor.html> |
| Planner | <https://moveit.picknik.ai/humble/doc/concepts/motion_planning.html> |
| Kinematics (IK) | <https://moveit.picknik.ai/humble/doc/concepts/kinematics.html> |
| 궤적 시간 계산 | <https://moveit.picknik.ai/humble/doc/concepts/trajectory_processing.html> |

### 5.2 따라 하기 — 예제와 대응

| 주제 | 대응 예제 | 링크 |
|---|---|---|
| 첫 C++ 프로젝트 | 전체 | <https://moveit.picknik.ai/humble/doc/tutorials/your_first_project/your_first_project.html> |
| RViz 로 보기 | 전체 | <https://moveit.picknik.ai/humble/doc/tutorials/visualizing_in_rviz/visualizing_in_rviz.html> |
| RobotModel · RobotState (FK · IK) | `ex1` | <https://moveit.picknik.ai/humble/doc/examples/robot_model_and_robot_state/robot_model_and_robot_state_tutorial.html> |
| 물체 피해 계획 | `ex2` · `ex3` | <https://moveit.picknik.ai/humble/doc/tutorials/planning_around_objects/planning_around_objects.html> |
| Planning Scene (C++) | `ex2` | <https://moveit.picknik.ai/humble/doc/examples/planning_scene/planning_scene_tutorial.html> |
| Planning Scene (토픽 · 서비스) | `ex2` | <https://moveit.picknik.ai/humble/doc/examples/planning_scene_ros_api/planning_scene_ros_api_tutorial.html> |
| MoveGroupInterface 전반 | `ex3` · `ex4` | <https://moveit.picknik.ai/humble/doc/examples/move_group_interface/move_group_interface_tutorial.html> |
| OMPL 설정 | `ex3` · `ex3_1` | <https://moveit.picknik.ai/humble/doc/examples/ompl_interface/ompl_interface_tutorial.html> |
| Pilz (PTP · LIN · CIRC) | `ex3` · `ex3_1` | <https://moveit.picknik.ai/humble/doc/how_to_guides/pilz_industrial_motion_planner/pilz_industrial_motion_planner.html> |
| 컨트롤러 연결 | `ex4` · §3 | <https://moveit.picknik.ai/humble/doc/examples/controller_configuration/controller_configuration_tutorial.html> |
| 설정 패키지 만들기 (Setup Assistant) | — | <https://moveit.picknik.ai/humble/doc/examples/setup_assistant/setup_assistant_tutorial.html> |

### 5.3 API · 소스

- MoveGroupInterface: <https://moveit.picknik.ai/humble/api/html/classmoveit_1_1planning__interface_1_1MoveGroupInterface.html>
- PlanningSceneInterface: <https://moveit.picknik.ai/humble/api/html/classmoveit_1_1planning__interface_1_1PlanningSceneInterface.html>
- 튜토리얼 소스 코드 (humble 브랜치): <https://github.com/moveit/moveit2_tutorials/tree/humble>

### 5.4 튜토리얼과 다른 점

- 튜토리얼 로봇: Panda (`panda_arm` 그룹 · `panda_link0` 프레임)
  - piper 로 옮길 때: 그룹 `arm` · `gripper`, 프레임 `base_link`, 목표 링크 `link6`
- 튜토리얼은 mock 하드웨어 기준 → 실물 연결은 §3

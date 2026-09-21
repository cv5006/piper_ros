# piper_lecture_demo

로보틱스 기초 특강 2회차 **「운동 — Jacobian과 Inverse Kinematics」** 시연 데모.

`piper_description` · `piper_with_gripper_moveit` 을 그대로 쓴다. 새 설정은 만들지 않는다.

**측정값과 설계 근거는 [NOTES.md](NOTES.md) 에 따로 있다.** 이 문서는 실행 안내다.

---

## 데모

| 데모 | 무엇 | 강의 | MoveIt |
|---|---|---|---|
| **manipulability** | 타원체 + σ_min | M1 · M3-3 · M3-4 | 안 씀 |
| **workspace** | 작업영역 점구름 + 조건수 색 | M3-2 | 안 씀 |
| **ik_solutions** | 해 8개, 쓸 수 있는 것 1개 | M2 · M3-2 | 안 씀 |
| **obstacle** | 충돌 물체 spawn/despawn | M4-2 | 씀 |
| **tcp_vs_flange** | `link6` ≠ 손끝 | §12.3 | 씀 |
| **path_compare** | 같은 두 자세, 두 경로 | M4-1 | 씀 |
| rviz IK 마커 | 마커를 끌어 IK | M2 | 씀 (제작 없음) |

- `M`·`§` 는 강의 노트 절 번호. 레포에 없다.
- Jacobian 은 `kinematics.py` 에 직접 구현. 강의 수식 `Jᵢ = zᵢ × (pₑ − pᵢ)` 가 그대로 코드다.
- MoveIt 과 같은 값인지는 `jacobian_check` 로 확인 — **차이 정확히 0**.
- `ik_solutions` 는 URDF 스택 쪽이 낫다. 자세 프리셋이 거기 있고 타원체와 한 화면에 나온다.

---

## 실행

### ⚠ MoveIt 스택은 DISPLAY 가 있어야 뜬다

`move_group` 이 octomap 감시자에서 OpenGL 을 건드린다. 화면이 없으면 freeglut 스택
트레이스를 남기고 **죽는다.** SSH 실습이면 먼저:

```bash
export DISPLAY=:0
export XDG_RUNTIME_DIR=/run/user/$(id -u)
```

URDF 스택은 `move_group` 을 안 쓴다. `use_rviz:=false panel:=false` 로 화면 없이 돈다.

### launch 는 둘, 나머지는 run

| | 무엇 | 예 |
|---|---|---|
| `ros2 launch` | 스택. 강의 내내 켜 둔다 | `urdf_demos` · `moveit_demos` — 이 둘뿐 |
| `ros2 run` | 붙어서 한 가지 하고 빠진다 | `goto_pose` · `obstacle` · `tcp_vs_flange` · `path_compare` · `jacobian_check` |
| 버튼 | 위를 손으로 부른다 | `lecture_panel` (스택이 같이 띄운다) |

| 런치 | 무엇이 뜨나 |
|---|---|
| `urdf_demos.launch.py` | manipulability · workspace · ik_solutions + 패널 |
| `moveit_demos.launch.py` | piper 스택 + rviz IK 마커 · ik_solutions + 패널 |

⚠ **두 스택 동시 기동 금지.** 둘 다 `/robot_description` · `/joint_states` 를 발행해 덮어쓴다.

### URDF 스택

```bash
ros2 launch piper_lecture_demo urdf_demos.launch.py
ros2 launch piper_lecture_demo urdf_demos.launch.py workspace:=true   # 점구름까지
ros2 run piper_lecture_demo goto_pose.py elbow                        # 자세 전환
ros2 run piper_lecture_demo goto_pose.py                              # 프리셋 목록
```

- 슬라이더와 프리셋이 둘 다 산다. 슬라이더를 밀면 슬라이더가 이긴다.
- `goto_pose` 뒤에는 그 자세에 머문다.

**배선** — `/joint_states` 는 이 레포에서 **명령 토픽**이다. 발행자가 둘이면 덮어쓰므로 하나로 묶었다.

```
슬라이더 GUI ──/gui_joint_states──▶ pose_presets ──/joint_states──▶ robot_state_publisher
    ▲ (source_list 로 되받는다)                                    └▶ manipulability
goto_pose ──/pose_presets/goto──▶
```

GUI 가 되받으므로 프리셋 뒤에 슬라이더를 잡아도 팔이 안 튄다.

| 프리셋 | 보이는 것 |
|---|---|
| `good` | 둥근 타원체 |
| `stretch` | 팔을 편다 — 원반 |
| `elbow` | 팔꿈치+손목 특이점 — 선분 |
| `wrist` | 손목 특이점 — **둥근데 6자유도가 죽었다** |
| `home` | 전 관절 0 — **영자세가 곧 손목 특이점** |
| `shoulder` | 어깨 특이점 — 손목 중심이 `joint1` 축 위 |
| `slow` | 느린 자세 — **`elbow` 와 `w` 가 같은데 특이점이 아니다** |

**권장 진행: `good` → `stretch` → `elbow` → `slow`.** 앞 셋으로 "펴질수록 납작해진다",
`slow` 로 "w 가 작으면 특이점인가?"를 깬다.

```bash
ros2 param set /pose_presets duration 8.0    # 재기동 없이
ros2 param set /pose_presets loop true
```

**workspace** — `/joint_states` 를 안 쓰고 제 안에서 관절을 훑는다. 계산 16 초, 구름은
latch. 기본은 꺼둔다.

```bash
ros2 launch piper_lecture_demo urdf_demos.launch.py workspace:=true samples:="[1,61,61,9,9,1]"
```

rviz 의 **Workspace — reachable / oriented** 둘이 받는다.

### MoveIt 스택

piper 의 `demo.launch.py` 를 `include` 하고 **rviz 설정만 바꿔 끼운다.**

```bash
ros2 launch piper_lecture_demo moveit_demos.launch.py
ros2 launch piper_lecture_demo moveit_demos.launch.py panel:=false ik:=false
```

| | |
|---|---|
| piper `demo.launch.py` | rsp · move_group · ros2_control · rviz |
| 우리 `moveit_demos.rviz` | piper `moveit.rviz` + Lecture Markers · IK solutions ×2 |
| `ik_solutions` | 상주. 부를 때마다 다시 푼다 |
| `lecture_panel` | 버튼 창 |

### 강의 패널

두 런치가 같이 띄운다. 항상 위에 뜬다. **묶음 순서 = 강의 진행 순서.**

| # | 묶음 | 버튼 |
|---|---|---|
| 1 | Manipulability & poses (MoveIt 불필요) | Show/Hide ellipsoid · 프리셋 7개 (툴팁에 설명) |
| 2 | IK solutions (MoveIt 불필요) | Solve IK (약 1.5초) · Show/Hide usable · Show/Hide remaining |
| 3 | Obstacles (MoveIt 필요) | Add/Remove obstacle |

- 떠 있지 않은 스택의 묶음은 **회색으로 잠긴다** (1초마다 서비스 확인).
- 일은 딴 스레드에서. 누르는 동안 버튼 잠금 + 상태줄 `…`.
- 버튼 라벨·툴팁과 모든 터미널 출력은 **영어**. 주석과 이 문서만 한국어.

#### 감추기가 기본이다

**시각화는 전부 감춰진 채로 뜬다.** 팔만 있는 화면에서 버튼으로 하나씩 드러낸다.

| 감춰진 것 | 드러내는 버튼 | 처음부터 켜려면 |
|---|---|---|
| manipulability 타원체 | Show ellipsoid | `ellipsoid:=true` |
| 한계 안 해 · 초록 | Show usable solutions | `ik_valid:=true` |
| 한계 밖 해 · 빨강 | Show remaining solutions | (인자 없음) |

- 토글 라벨은 래치 토픽으로 실제 상태를 따른다 — `/manipulability/shown` ·
  `/ik_solutions/shown_valid` · `/ik_solutions/revealed`.
- ⚠ `ros2 param set /manipulability show false` 는 **안 듣는다.** 파라미터는 기동 시 1회만 읽는다.
- 숫자판(`/manipulability_readout`)은 감추지 않는다. 토글 버튼이 없어 되살릴 수단이 없다.
  아예 안 띄우려면 `readout:=none`.

버튼이 하는 일은 전부 터미널로도 된다.

```bash
ros2 service call /ik_solutions/solve std_srvs/srv/Trigger
ros2 service call /ik_solutions/reveal std_srvs/srv/SetBool "{data: true}"
ros2 service call /ik_solutions/show_valid std_srvs/srv/SetBool "{data: true}"
ros2 service call /manipulability/show std_srvs/srv/SetBool "{data: true}"
ros2 run piper_lecture_demo obstacle.py spawn
```

---

## 진행 순서

### ① 기본 사용법 (M2 · M4)

1. Planning Group = `arm` 확인.
2. 주황 마커를 끈다 → 로봇이 따라오면 **IK 가 풀린 것**.
3. 안 닿는 곳까지 끈다 → 로봇 정지, 마커만 간다. **해가 없다** (M3-2).
4. Plan & Execute → 여기부터 M4.

⚠ **이 팔은 마커를 끌어도 elbow up/down 이 안 튄다.** 관절 한계가 다른 해를 다 잘라낸다.
왜인지는 ④ 가 보여준다.

### ② 보간 차이 (M4-1)

Planning 탭의 **`Use Cartesian Path`** 체크박스 하나다.

| | 해제 | 체크 |
|---|---|---|
| 무엇을 잇나 | 관절 공간 | 작업 공간 |
| 손끝 자취 | 휜다 | 직선 |
| 못 가면 | 계획 실패 | **간 만큼만** |

- 체크한 쪽은 상태줄에 `Achieved xx % of Cartesian path` — **이 숫자가 M4-1 의 답이다.**
- 경계 쪽으로 밀면 100 % 가 안 나온다.
- 자취를 **겹쳐서** 보려면 `path_compare`. rviz 는 애니메이션만 보여주고 자취를 안 남긴다.

```bash
ros2 run piper_lecture_demo path_compare
```

⚠ 기동 시 `No kinematics plugins defined` 경고 한 줄. **결과에 영향 없다** —
`fraction = 0.627` · 이동 `0.1569 m` 로 런치로 넘기던 때와 동일.

### ③ 충돌 환경 (M4-2)

```bash
ros2 run piper_lecture_demo obstacle.py spawn         # 기본 기둥
ros2 run piper_lecture_demo obstacle.py list
ros2 run piper_lecture_demo obstacle.py despawn --all
```

세우고 **같은 목표로 다시 Plan** 하면 경로가 달라진다. 자리·크기·도형은 인자로 바꾼다.

```bash
obstacle.py spawn --id lecture_wall --shape box      --xyz 0.35 0 0.30 --size 0.02 0.40 0.30
obstacle.py spawn --id lecture_can  --shape cylinder --xyz 0.30 0.20 0.10 --size 0.20 0.05
obstacle.py spawn --id lecture_ball --shape sphere   --xyz 0.30 0 0.40 --size 0.06
```

- `cylinder --size` 는 **[높이, 반지름]** 순 (`shape_msgs/SolidPrimitive` 기준).
- rviz 로도 된다 — MotionPlanning ▸ Scene Objects ▸ Publish. `.scene` 내보내기/가져오기도 있다.
- 즉흥적으로는 rviz, 매번 같은 자리면 이 노드.

### ④ 해는 여럿, 쓸 수 있는 것은 하나 (M2 · M3-2)

패널의 **Solve IK**. 상주 노드라 팔을 옮기고 또 누르면 그 자리에서 다시 찾는다.

- 방법은 강의 반복식 그대로 — `q ← q + J⁺(x_target − f(q))`, 무작위 초기값, 중복 제거 (DLS 감쇠).
- **관절 한계는 걸지 않는다.** 한계 밖 해를 보여주는 것이 요지다.

| 디스플레이 | 토픽 | 색 | 언제 |
|---|---|---|---|
| IK solutions (within limits) | `/ik_solutions/valid` | 초록 | Show usable solutions |
| IK solutions (out of limits) | `/ik_solutions/invalid` | 빨강 | Show remaining solutions |

둘 다 **MarkerArray**. 해 하나가 로봇 한 벌, 링크마다 메시 Marker (`kinematics.Visuals`).
감추기는 **빈 메시지**로 한다 — 체크박스를 마우스로 찾지 않게.

| 누르는 차례 | 화면 |
|---|---|
| (시작) | 팔만 |
| Show usable solutions | 초록 하나 — 팔과 겹쳐 티가 안 난다. *"해가 몇 개일까?"* |
| Show remaining solutions | 빨강 일곱이 한꺼번에 |
| Show ellipsoid | 왜 그 자세가 그런지로 |

- ⚠ **초록 하나는 「지금 로봇이 있는 그 자세」다.** 정확히 겹쳐 그려져 화면에 변화가 없다.
  볼 것은 터미널 개수와 이어질 빨강이다.
- **자세를 옮기면 빨강은 다시 감춰진다.** 드러내는 장치가 자세마다 되감긴다.
- 초록은 유지된다 — 연출이 아니라 보기 설정이다.

```
found 8 solutions - 1 within joint limits / 7 outside
  [usable]       [0.3, 1.2, -1.0, 0.3, 0.6, -0.0]
  [out of limit] [0.3, 2.87, 1.621, 0.303, 2.55, 0.504]
                 joint3=+93deg(limit -170~+0) · joint5=+146deg(limit -70~+70)
```

**말할 것 둘.**

- 「해가 여럿」은 **기구학의 성질**, 「쓸 수 있는 해가 하나」는 **이 로봇의 성질**이다.
  - 손목 — 뒤집으려면 `joint4`·`joint6` 을 180° 돌려야 하나 한계가 ±100°·±120°. `joint5` 는 ±70°.
  - 팔꿈치 — `joint3`(−170°~0). 어깨 — `joint2`(0~180°).
- **구형 손목이 아니다** — `joint4`·`joint5` 축은 만나지만 `joint6` 축이 **0.091 m** 떨어져 있다.
  해석해가 안 떨어지고, 레포가 KDL 수치해를 쓰는 이유다 (§7.7).

#### 탐색을 왜 줄였나

시드 400개를 끝까지 돌려 **23.5 초**. 시간이 두 군데로 샜다.

| 원인 | |
|---|---|
| 수렴 판정 | `break` 기준 `1e-11`, 채택 기준 `1e-6` — 사실상 안 걸렸다. 수렴한 시드도 250회 완주 |
| 헛도는 시드 | 400개 중 **118개**가 끝내 수렴 못 하면서 250회를 꽉 채웠다 |

수렴 기준 `1e-9`, 60회까지 오차 `1e-3` 밖이면 접는다. 무작위 6 자세 대조:

| | 해집합 | 시간 |
|---|---|---|
| 400 시드 · 안 접음 | 기준 | 4.2 ~ **41.6** 초 |
| 40 시드 · 접음 | **6 자세 전부 동일** | 0.4 ~ 3.8 초 |

**해를 하나도 잃지 않는다.** 해가 8개뿐이라 시드를 더 뿌려도 같은 것을 또 찾는다.

기본은 `seeds = 80` + `time_budget = 3.0`. 특이점 근처는 시드 하나가 250회를 다 쓰기도 해
개수만으로는 상한이 안 잡힌다. 실제 사용량을 로그에 찍는다 — `seeds 43/80 · 3.1 s`.

```bash
ros2 param set /ik_solutions seeds 200
ros2 param set /ik_solutions time_budget 0.0   # 0 = 무제한
```

### ⑤ 맞춘 것은 손끝이 아니다 (§12.3)

```bash
ros2 run piper_lecture_demo tcp_vs_flange.py
```

- 빨간 점 `link6`(MoveIt 이 맞추는 자리), 파란 점 손끝, 사이 **0.1358 m**.
- **rviz 에서 볼 수 없는 것이 이것 하나다** — 목표 마커는 `link6` 에 붙어 있는데 화면에
  그렇게 안 쓰여 있다.
- TF 두 개만 읽으므로 URDF 스택에서도 돈다.
- ⚠ 실물 연결 상태로 띄우지 말 것 (§6-③).

---

## 실물 팔에 물리기 — `real:=true`

```bash
ros2 launch piper_lecture_demo moveit_demos.launch.py real:=true
ros2 launch piper_lecture_demo moveit_demos.launch.py real:=true stub:=true   # 실물 없이
```

`mock_components/GenericSystem` 자리를 `piper_real_adapter.py` 가 대신한다.

| MoveIt 이 부르는 것 | 드라이버가 가진 것 |
|---|---|
| `/arm_controller/follow_joint_trajectory` (joint1~6) | `joint_ctrl_single` — JointState 하나가 목표 자세 하나 |
| `/gripper_controller/follow_joint_trajectory` (joint7) | 같음 (`position[6]` 이 그리퍼) |
| `/joint_states` | `joint_states_feedback` |

1. **상태 다리** — `joint_states_feedback` → `/joint_states`. 드라이버의 `gripper` 를
   `joint7` 로 고치고 `joint8` 도 대칭으로 채운다. **`Missing joint8` 경고가 사라진다** (0 건).
2. **명령 다리** — `joint_ctrl_single` 로 목표를 쏜다. `velocity[6]` 에 속도(%)를 반드시 채운다.
3. **궤적 실행** — 액션 서버 둘. 드라이버에 보간이 없어 어댑터가 100 Hz 로 보간해 쏜다.

### 안전장치 넷

| | |
|---|---|
| 속도 | `speed_percent` 기본 **20**. 비우면 드라이버가 **전속**(100)을 건다 |
| 계단 명령 | 첫 경유점이 `max_step_rad`(기본 0.35 rad = 20°) 넘게 떨어지면 goal 거절 |
| enable | goal 받으면 `enable_srv` 를 먼저 부른다. 실패하면 안 움직인다 |
| 취소 | 지금 자세를 목표로 다시 쏴 **그 자리에 세운다** |

### ⚠ remap 을 하지 않는 이유

piper 의 `start_single_piper.launch.py` 는 `joint_ctrl_single` 을 `/joint_states` 로 remap
한다. 그러면 **상태 다리가 내보낸 값이 명령으로 되돌아와 루프가 된다.** `real:=true` 는
드라이버 노드를 remap 없이 직접 띄운다.

확인 — `/joint_states` 발행자 1 · 구독자 3(rsp · move_group · ik_solutions),
`/joint_ctrl_single` 발행자 1(어댑터) · 구독자 1(드라이버).

### 실물 없이 확인한 것

`can0` 없이 `stub:=true` 로 전 구간을 돌렸다.

| 확인됨 | |
|---|---|
| 액션 → 보간 → 명령 | goal `SUCCEEDED`, 스텁이 목표에 정확히 도달 |
| 속도 필드 | 모든 명령에 **20 %** (100 % 아님) |
| enable | 첫 goal 에서 `enable -> True` |
| 되먹임 없음 | 위 발행자/구독자 수 |
| `joint8` | 여덟 관절 발행, `Missing joint8` **0 건** |
| MoveIt 전 구간 | `path_compare` 계획·실행 완주 (`Execute request success!` ×4) |
| 계단 방어 | 72° goal 을 `ABORTED` + 사유로 거절 |

**확인 안 된 것** — CAN 타이밍 · 실제 추종 오차 · 관절 한계 거동 · 그리퍼 힘.
스텁은 토픽 인터페이스만 흉내 낸다. **실물에서 다시 재야 한다.**

### ⚠ 실물 연결 상태로 URDF 스택을 띄우지 말 것

`/joint_states` 가 **명령 토픽**이라 슬라이더와 프리셋이 **진짜 팔을 움직인다.**
특이점 자세가 프리셋에 있고 속도·정지 수단이 없다. URDF 스택은 실물에 물리지 않았다.

이 데모는 전부 `mock_components/GenericSystem` 위에서 돈다. 실물이 필요 없다.

---

## 직접 구현한 Jacobian 이 맞는가 — `jacobian_check`

```bash
ros2 run piper_lecture_demo jacobian_check
ros2 run piper_lecture_demo jacobian_check --ros-args -p tcp_offset:="[0.0,0.0,0.0]"   # 플랜지
```

| | 어떻게 | 분량 |
|---|---|---|
| **MoveIt** | `state.getJacobian(jmg, tip, ref_point, J)` | 한 줄 |
| **강의 수식** | `Jᵢ = [ zᵢ × (pₑ − pᵢ) ; zᵢ ]` 를 C++ 로 직접 | 15 줄 |

```
  preset     sigma (linear)          w          cond     sigma_min_6d  max|MoveIt - formula|
  good       0.6744 0.5041 0.2373   0.080674      2.8      0.103945  0.00e+00
  stretch    0.8947 0.7470 0.0851   0.056877     10.5      0.007470  0.00e+00
  elbow      0.9285 0.7554 0.0120   0.008414     77.4      0.000050  0.00e+00
  wrist      0.6777 0.4155 0.2131   0.059999      3.2      0.000057  0.00e+00
  home       0.5672 0.1914 0.0729   0.007911      7.8      0.000061  0.00e+00
  shoulder   0.6084 0.2768 0.1183   0.019928      5.1      0.000076  0.00e+00
  slow       0.4428 0.2828 0.0680   0.008510      6.5      0.023764  0.00e+00

  worst disagreement over 7 poses: 0.00e+00
  -> bit-for-bit identical. MoveIt evaluates the same formula in the same order.
```

- **차이가 부동소수 오차가 아니라 정확히 0.** MoveIt 이 같은 수식을 같은 순서로 계산한다.
- `sigma`·`w`·`cond`·`sigma_min_6d` 는 manipulability 가 같은 프리셋에서 찍는 값과 일치
  → **`kinematics.py` 까지 같이 대조된다.**
- 수식을 C++ 로 **다시** 구현한 것이 핵심. import 해서 비교하면 「같은 코드가 같은 값」에
  그친다. 독립 구현 둘이 같은 값이면 **수식이 맞다는 증거**다.
- `move_group` 불필요. URDF + SRDF 로 `RobotModel` 만 세운다.
- ⚠ 함정 — `RobotModelLoader::Options(urdf_string, srdf_string)` 를 쓰면 운동학 파라미터
  접두사가 `robot_description_kinematics` 가 아니라 **`_kinematics`** 가 된다
  (`robot_description_` 이 빈 문자열이라서).

---

## 더 읽을 것 — [NOTES.md](NOTES.md)

| 절 | 무엇 |
|---|---|
| 설계 결정 | 숫자판을 Image 로 낸 이유 · IK 해를 Marker 로 그린 이유 · 증상 둘 |
| workspace 의 두 구름 | `reachable` 과 `oriented` 가 왜 다른가 |
| 특이점 판정 기준 | `w` · `cond` · `σ_min` 중 무엇으로 자를 것인가 (실측 근거) |
| 확인된 수치 | TCP 기준점 · 특이점 세 자리 · 작업영역 · 보간 두 종 |
| piper 설정은 하나도 덮지 않는다 | IK 제한시간을 덮었다가 뺀 이유 |
| 레포에서 발견한 것 | 가속도 한계 없음 · `joint8` 누락 · `ompl_planning.yaml` 없음 |

---

## 파일

```
piper_lecture_demo/
├── README.md                          실행 안내 (이 문서)
├── NOTES.md                           측정 · 설계 근거
├── piper_lecture_demo/kinematics.py   URDF -> FK · Jacobian (numpy 뿐)
├── scripts/
│   ├── manipulability.py              타원체 + σ_min 본체
│   ├── pose_presets.py                자세 드라이버 (/joint_states 유일 발행자)
│   ├── goto_pose.py                   자세 전환 명령
│   ├── workspace.py                   작업영역 점구름
│   ├── obstacle.py                    충돌 물체 spawn/despawn
│   ├── tcp_vs_flange.py               link6 vs 손끝 (TF 만 읽는다)
│   ├── ik_solutions.py                IK 해 전수 탐색 (상주 · 서비스)
│   ├── piper_real_adapter.py          MoveIt 스택을 실물 팔에 물린다
│   ├── piper_driver_stub.py           드라이버 대역 (실물 없이 시험)
│   └── lecture_panel.py               버튼 창 (PyQt5)
├── src/
│   ├── path_compare.cpp               관절 보간 vs 손끝 직선
│   └── jacobian_check.cpp             MoveIt getJacobian() 대조
├── launch/
│   ├── urdf_demos.launch.py           URDF 만 읽는 셋 + 패널
│   └── moveit_demos.launch.py         piper 런치 + 우리 rviz 설정 + 패널
└── config/
    ├── urdf_demos.rviz                타원체 · 숫자판 · IK 해 · 작업영역
    └── moveit_demos.rviz              piper moveit.rviz + Marker 디스플레이
```

## 빌드

```bash
cd <워크스페이스> && colcon build --packages-select piper_lecture_demo --symlink-install
```

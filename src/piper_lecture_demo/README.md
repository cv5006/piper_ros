# piper_lecture_demo

로보틱스 기초 특강 2회차 **「운동 — Jacobian과 Inverse Kinematics」** 시연용 데모 모음.

로봇 모델도 MoveIt 설정도 새로 만들지 않는다. `piper_description` 과
`piper_with_gripper_moveit` 의 것을 그대로 쓴다.

---

## 데모 여섯 개

| | 데모 | 담당 | MoveIt | 실행 |
|---|---|---|---|---|
| **D1** | manipulability 타원체 + σ_min | M1 · M3-3 · M3-4 | **안 씀** | `d1_manipulability.launch.py` |
| **D2** | 작업영역 점구름 + 조건수 컬러맵 | M3-2 | **안 씀** | D1 런치에 `workspace:=true` |
| **D3** | 충돌 환경 spawn/despawn · `link6` ≠ 손끝 | M4-2 · §12.3 | 씀 | `d3_obstacle.py` · `d3_tcp_offset.py` |
| **D4** | 마커를 끌어 IK 를 만져본다 | M2 | 씀 | **제작 없음** — rviz MotionPlanning |
| **D5** | 같은 두 자세, 두 경로 | M4-1 | 씀 | `ros2 run … d5_interp_compare` |
| **D6** | 해는 8개, 쓸 수 있는 것은 1개 | M2 · M3-2 | **안 씀** | `d6_ik_branches.py` — **두 스택 어디서나** |

**D1 · D2 · D6 은 URDF 하나만 읽는다.** 의존성은 `numpy` 뿐이고 MoveIt 을 세우지 않은
팀도 그대로 돌린다. Jacobian 은 `piper_lecture_demo/kinematics.py` 에 직접 구현돼 있다 —
강의에서 다룬 `Jᵢ = zᵢ × (pₑ − pᵢ)` 가 그대로 코드다.

**D3 · D4 · D5 는 MoveIt 을 얹는다.** 스택을 먼저 띄우고 데모를 실행한다.

> **D6 은 D1 스택에서 도는 쪽이 낫다.** 해를 찾으려면 팔을 옮겨야 하는데, 그 수단인
> 자세 프리셋이 D1 것이다. 게다가 타원체와 해가 한 화면에 같이 나온다 —
> *"이 자세는 특이점이라 해가 낱개가 아니다"* 를 타원체로 보이면서 말할 수 있다.
> D1 런치가 D6 을 같이 띄운다 (`ik:=false` 로 끈다).

---

## 실행

### launch 와 run 을 언제 쓰나

| | 무엇 | 예 |
|---|---|---|
| **`ros2 launch`** | **스택을 띄운다.** 로봇 모델 · rviz · 부속 노드가 한꺼번에 뜬다. 강의 내내 켜 둔다 | `d1_manipulability` · `moveit_demo` — **이 둘뿐이다** |
| **`ros2 run`** | **이미 떠 있는 스택에 붙어 한 가지 일을 하고 빠진다.** 자세를 바꾸거나, 물체를 세운다 | `d1_goto` · `d3_obstacle` · `d3_tcp_offset` · `d5_interp_compare` |
| **버튼** | 위의 것들을 강의 중에 손으로 부른다 | `lecture_panel` (스택이 같이 띄운다) |

스택은 한 번만 띄우고, 강의 중에 치는 것은 전부 `ros2 run` 쪽이다. rviz 를 내리지
않는 것이 이 구성의 요점이다 (강의 노트 §3.2).

> 예외가 하나 있다. `d5_interp_compare` 는 C++ 로 MoveGroupInterface 를 쓰는데 그것이
> 자기 쪽에도 로봇 모델을 들고 있어야 해서, 파라미터를 넘겨주는 런치를 쓴다.
> `ros2 run piper_lecture_demo d5_interp_compare` 로도 결과는 같지만
> `No kinematics plugins defined` 경고가 한 줄 뜬다.

**런치는 둘뿐이다.** 스택이 둘이기 때문이다 — URDF 만 읽는 쪽과 MoveIt 을 얹는 쪽.
데모마다 런치를 두지 않는다.

| 런치 | 무엇이 뜨나 |
|---|---|
| `d1_manipulability.launch.py` | D1 · D2 · D6 + 패널 (URDF 만) |
| `moveit_demo.launch.py` | D3 · D4 · D5 · D6 + 패널 (MoveIt 위) |

### URDF 계열 (D1 · D2 · D6)

```bash
ros2 launch piper_lecture_demo d1_manipulability.launch.py
ros2 launch piper_lecture_demo d1_manipulability.launch.py workspace:=true   # D2 까지
```

D1 은 **슬라이더와 프리셋을 둘 다 살려둔 채** 뜬다. 런치 인자로 고를 것이 없다.
**D6(IK 해 전수 탐색)과 강의 패널도 같이 뜬다** — 끄려면 `ik:=false panel:=false`.

```bash
ros2 launch piper_lecture_demo d1_manipulability.launch.py   # 한 번만
ros2 run piper_lecture_demo d1_goto.py elbow                 # 필요할 때마다
ros2 run piper_lecture_demo d1_goto.py slow
ros2 run piper_lecture_demo d1_goto.py                       # 목록 보기
```

슬라이더를 밀면 슬라이더가 이기고, `d1_goto` 를 쓰면 그 자세로 간 뒤 **머문다.**
보간이 끝난 뒤 슬라이더를 다시 밀면 슬라이더로 돌아온다.

**배선이 이렇게 되어 있다.** `/joint_states` 는 이 레포에서 명령 토픽이라 발행자가 둘이면
서로 덮어쓴다. 그래서 발행자를 하나로 묶었다.

```
슬라이더 GUI ──/gui_joint_states──▶ d1_preset ──/joint_states──▶ robot_state_publisher
    ▲ (source_list 로 /joint_states 를 되받아 슬라이더가 프리셋을 따라온다)   └▶ d1_manipulability
d1_goto ──/d1_preset/goto──▶
```

GUI 가 `/joint_states` 를 되받으므로 **프리셋 뒤에 슬라이더를 잡아도 팔이 튀지 않는다.**

| 프리셋 | 보이는 것 |
|---|---|
| `good` | 둥근 타원체 |
| `stretch` | 팔을 편다 — 원반이 된다 |
| `elbow` | 팔꿈치+손목 특이점 — 선분이 된다 |
| `wrist` | 손목 특이점 — **타원체는 둥근데 6자유도가 죽어 있다** |
| `home` | 모든 관절 0 — 이 로봇의 영자세가 곧 손목 특이점 |
| `shoulder` | 어깨 특이점 — 손목 중심이 `joint1` 축 위 |
| `slow` | 전체가 느린 자세 — **`elbow` 와 `w` 는 같은데 특이점이 아니다** |

**강의 진행 권장: `good` → `stretch` → `elbow` → `slow`.** 앞 셋으로 *"펴질수록
납작해진다"* 를 보인 뒤, 마지막에 같은 `w` 를 들이밀어 *"그럼 w 가 작으면 특이점인가?"*
를 깬다.

**타원체는 껐다 켤 수 있다.** D6 의 해 여덟 벌이 화면에 겹쳐 있으면 타원체까지 보태져
붐비는데, 그때 잠깐 치우고 자세만 보는 쪽이 낫다. 다시 켜면 그 자리에 그대로 돌아온다.

```bash
ros2 service call /d1_manipulability/show std_srvs/srv/SetBool "{data: false}"
```

패널의 **「타원체 감추기」** 버튼이 같은 일을 한다. 감춘 채로 띄우려면 런치 인자를 쓴다.

```bash
ros2 launch piper_lecture_demo d1_manipulability.launch.py ellipsoid:=false
```

⚠ `ros2 param set /d1_manipulability show false` 는 **듣지 않는다.** 파라미터는 기동할 때
한 번만 읽는다. 켜고 끄는 것은 위 서비스(=버튼)다.

지금 켜져 있는지는 `/d1_manipulability/shown` 에 래치돼 있다. 패널이 이것을 구독하므로
**터미널로 껐다 켜도, `ellipsoid:=false` 로 띄웠어도 버튼 라벨이 실제 상태를 따른다.**

> 숫자판(Image)은 따로 논다. 3D 화면을 가리지 않으므로 같이 끄지 않았다 —
> 필요하면 `readout:=none` 으로 아예 안 띄운다.

전환 속도와 왕복은 드라이버 파라미터로 바꾼다. 재기동은 필요 없다.

```bash
ros2 param set /d1_preset duration 8.0
ros2 param set /d1_preset loop true
ros2 param set /d1_preset preset wrist     # d1_goto.py 대신 이것도 된다
```

**D2 는 D1 스택에 얹혀 있다.** `d2_workspace.py` 가 `/joint_states` 를 쓰지 않고 관절을
제 안에서 훑기 때문에, 자기 스택을 따로 세울 이유가 없다. 계산에 16 초쯤 걸려서
기본으로는 꺼 두고 필요할 때 켠다. 구름은 latch 되므로 계산이 끝나면 그대로 남는다.

```bash
ros2 launch piper_lecture_demo d1_manipulability.launch.py workspace:=true
ros2 launch piper_lecture_demo d1_manipulability.launch.py \
    workspace:=true samples:="[1, 61, 61, 9, 9, 1]"      # 수직 단면만
```

rviz 의 **Workspace — reachable / oriented** 디스플레이 둘이 받는다. 타원체와 같은 화면이라
*"이 자세가 왜 경계 근처인가"* 를 점구름 위에서 짚을 수 있다.

### MoveIt 계열 (D3 · D4 · D5)

**스택은 piper 것을 그대로 쓴다.** 이 패키지의 런치는 그것을 `include` 하고
**rviz 설정만 바꿔 끼운다** — 스택을 다시 만들지 않는다.

```bash
ros2 launch piper_lecture_demo moveit_demo.launch.py
```

이 한 줄이 띄우는 것:

| | |
|---|---|
| piper 의 `demo.launch.py` | rsp · move_group · ros2_control · rviz |
| 우리 `moveit_demo.rviz` | piper 의 `moveit.rviz` + 디스플레이 셋 (Lecture Markers · IK solutions ×2) |
| `d6_ik_branches` | 상주하며 부를 때마다 IK 해를 다시 찾는다 |
| `lecture_panel` | **버튼 창.** 강의 중에 터미널을 치지 않기 위한 것 |

```bash
ros2 launch piper_lecture_demo moveit_demo.launch.py panel:=false ik:=false   # 끄고 싶으면
```

한 번 띄워두고 강의 내내 내리지 않는다. **기본 사용법과 보간 차이는 rviz 안에서
전부 끝난다** — 아래 진행 순서가 곧 D4 와 D5 다.

### 강의 패널 — 버튼으로 돌린다

`moveit_demo.launch.py` 와 `d1_manipulability.launch.py` 가 같이 띄운다.
rviz 옆에 두면 된다 (항상 위에 뜬다). 버튼은 세 묶음이고, **IK 쪽은 스택을 가리지 않는다.**

| 버튼 | 하는 일 |
|---|---|
| **IK 해 찾기 (D6)** | 지금 팔 자세의 손끝에 대해 해를 전부 찾아 rviz 에 낸다 (약 1.5 초) |
| **쓸 수 있는 해 감추기 / 보이기** | 초록을 냈다 뺐다 한다. 셋 다 끄면 **팔만 남는다** |
| **나머지 해 보이기 / 감추기** | 한계 밖 해를 화면에 냈다 뺐다 한다. **다시 풀지 않으므로 즉시** |
| **장애물 세우기 / 치우기** | planning scene 에 기둥을 더하고 뺀다 |
| **타원체 감추기 / 보이기** | D1 의 manipulability 타원체를 화면에서 뺐다 넣었다 한다 |
| **자세 프리셋** (good · stretch · elbow · wrist · slow · home · shoulder) | D1 스택의 자세 드라이버에게 보낸다 |

두 스택은 같이 띄우지 않으므로, 지금 떠 있지 않은 쪽 버튼을 누르면 상태줄에
「대상이 없다」고만 뜨고 아무 일도 일어나지 않는다.

**일은 딴 스레드에서 한다.** 누르는 동안 버튼이 잠기고 상태줄이 「…」로 바뀐다.
예전에는 버튼 콜백에서 그대로 기다려 그동안 창이 얼어붙었다.

버튼이 하는 일은 전부 터미널로도 된다.

```bash
ros2 service call /d6_ik_branches/solve std_srvs/srv/Trigger
ros2 service call /d6_ik_branches/reveal std_srvs/srv/SetBool "{data: true}"
ros2 service call /d6_ik_branches/show_valid std_srvs/srv/SetBool "{data: false}"
ros2 service call /d1_manipulability/show std_srvs/srv/SetBool "{data: false}"
ros2 run piper_lecture_demo d3_obstacle.py spawn
ros2 run piper_lecture_demo d1_goto.py elbow
```

### 진행 순서

**① 기본 사용법 (M2 · M4)**

1. Planning Group 이 `arm` 인지 확인한다.
2. 주황색 대화형 마커를 끌어 목표 자세를 옮긴다 → 로봇이 따라오면 **IK 가 풀린 것**이다.
3. 팔이 닿지 않는 곳까지 끌어본다 → 로봇이 멈추고 마커만 간다. **해가 없다** (M3-2).
4. Plan & Execute → 여기부터가 M4 다.

> ⚠ **이 팔에서는 마커를 끌어도 elbow up/down 이 튀지 않는다.** 관절 한계가 다른 해를
> 전부 잘라내기 때문이다. 왜 그런지는 아래 D6 가 보여준다.

**② 공간별 보간 차이 (M4-1)** — Planning 탭의 **`Use Cartesian Path`** 체크박스 하나다.

| | 체크 해제 | 체크 |
|---|---|---|
| 무엇을 잇나 | **관절 공간**에서 길을 찾는다 | **작업 공간**에서 곧게 잇는다 |
| 손끝 자취 | 휜다 | 직선 |
| 못 가면 | 계획 실패 | **간 만큼만** 준다 |

같은 목표를 두 번 계획해 비교한다. 체크한 쪽은 상태줄에
`Achieved xx % of Cartesian path` 가 뜨는데, **이 숫자가 M4-1 의 답이다** —
작업 공간의 직선은 언제나 갈 수 있는 것이 아니다. 작업영역 경계 쪽으로 목표를 밀면
100 % 가 안 나온다.

> 자취를 **겹쳐서** 보여주려면 `d5_interp_compare` 를 쓴다. rviz 는 계획된 궤적을
> 애니메이션으로만 보여주고 자취를 남기지 않는다. 이 노드는 두 경로의 손끝 자취를
> Marker 로 남기고 직선에서의 최대 이탈(0.0397 m vs 0.0000 m)을 숫자로 낸다.

```bash
ros2 run piper_lecture_demo d5_interp_compare
```

> ⚠ 기동할 때 `No kinematics plugins defined` 경고가 **한 줄** 뜬다. 이 노드가 자기 쪽에도
> 로봇 모델을 들고 있어야 해서 그렇고, **결과에는 영향이 없다** — `fraction = 0.627` ·
> 실제 이동 `0.1569 m` 로 런치로 파라미터를 넘겨주던 때와 소수점까지 같다. 확인했다.

**③ 충돌 환경을 준다 (M4-2)**

```bash
ros2 run piper_lecture_demo d3_obstacle.py spawn         # 기본 기둥
ros2 run piper_lecture_demo d3_obstacle.py list
ros2 run piper_lecture_demo d3_obstacle.py despawn --all
```

세우고 나서 **같은 목표로 다시 Plan** 하면 경로가 달라진다. 플래너에 준 것은 환경
하나뿐이다. 자리·크기·도형은 코드를 고치지 않고 인자로 바꾼다.

```bash
d3_obstacle.py spawn --id lecture_wall --shape box      --xyz 0.35 0 0.30 --size 0.02 0.40 0.30
d3_obstacle.py spawn --id lecture_can  --shape cylinder --xyz 0.30 0.20 0.10 --size 0.20 0.05
d3_obstacle.py spawn --id lecture_ball --shape sphere   --xyz 0.30 0 0.40 --size 0.06
```

> **rviz 로도 만들 수 있다.** MotionPlanning 패널의 **Scene Objects** 탭에서 도형을 더하고
> 마우스로 옮긴 뒤 Publish 하면 된다. 같은 탭의 **Export/Import Scene Geometry** 로
> `.scene` 파일에 저장했다가 다시 불러올 수도 있다. 즉흥적으로 만들 때는 rviz 가,
> 매번 같은 자리에 같은 것을 세워야 하는 강의에서는 이 노드가 편하다.

**④ 해는 여럿인데 쓸 수 있는 것은 하나다 (M2 · M3-2)**

패널의 **「IK 해 찾기」** 버튼을 누른다 (또는 `ros2 service call /d6_ik_branches/solve
std_srvs/srv/Trigger`). 노드는 상주하므로 **팔을 옮기고 또 누르면 그 자리에서 다시 찾는다.**

지금 팔이 있는 자리의 손끝 자세를 목표로 잡고, 그 자세를 만드는 관절값을 **전부** 찾는다.
찾는 방법은 강의에서 다룬 반복식 그대로다 — `q ← q + J⁺(x_target − f(q))` 를 무작위
초기값에서 돌리고 수렴한 것을 모아 중복을 지운다 (DLS 감쇠).

rviz 의 Trajectory 디스플레이 둘이 결과를 받는다. **Show Trail 이 켜져 있어 여러 자세가
한 화면에 겹쳐 보인다.**

| 디스플레이 | 토픽 | 색 | 언제 보이나 |
|---|---|---|---|
| IK solutions (within limits) | `/ik_branches/valid` | 초록 | 풀면 바로. **「쓸 수 있는 해 감추기」 로 끈다** |
| IK solutions (out of limits) | `/ik_branches/invalid` | 빨강 | **「나머지 해 보이기」 를 누르면** |

둘 다 **MarkerArray** 다. 해 하나가 로봇 한 벌이고, 링크마다 메시 Marker 를 제자리에
놓는다 (`kinematics.Visuals` 가 URDF 에서 링크별 메시와 트리를 읽는다).

**두 디스플레이 다 rviz 에서는 켜 두었다.** 빨강을 감추는 것은 노드가 빈 메시지를
내는 것으로 한다 — 강의 중에 체크박스를 마우스로 찾지 않고 버튼 하나로 연출이 되게.

*"해가 몇 개일까?"* 를 묻고 초록 하나를 보인 뒤 **나머지 일곱을 한꺼번에 드러내는
것**이 이 데모의 장치다. 드러낼 때 다시 풀지 않고 이미 찾아둔 것을 내보내므로 즉시 뜬다.

> ⚠ **초록 하나는 「지금 로봇이 있는 그 자세」다.** 진짜 로봇과 정확히 겹쳐 그려지므로
> 초록만으로는 화면에 변화가 없다 — 눈으로 볼 것은 터미널의 개수와, 이어서 드러낼
> 빨강 쪽이다. 「눌렀는데 아무 일도 안 일어난다」의 정체가 이것이다.

**빈 화면에서 시작하고 싶으면** 초록도 끈다. 셋(초록 · 빨강 · 타원체)을 다 끄면 팔만
남으므로, 거기서 하나씩 얹으며 진행할 수 있다.

```bash
ros2 launch piper_lecture_demo d1_manipulability.launch.py ik_valid:=false ellipsoid:=false
```

| 누르는 차례 | 화면 |
|---|---|
| (시작) | 팔만 |
| **쓸 수 있는 해 보이기** | 초록 하나 — 그런데 팔과 겹쳐서 티가 안 난다. *"해가 몇 개일까?"* |
| **나머지 해 보이기** | 빨강 일곱이 한꺼번에 — *"기구학적으로는 여덟인데 쓸 수 있는 건 하나"* |
| **타원체 보이기** | 왜 그 자세가 그런지로 넘어간다 |

**자세를 옮기면 빨강은 다시 감춰진다.** 프리셋으로 팔을 옮기고 다시 풀면 초록만 남으므로,
드러내는 장치가 자세마다 되감긴다. 초록은 건드리지 않는다 — 그쪽은 연출이 아니라
보기 설정이라 한 번 정하면 유지된다.

세 상태는 각각 `/d6_ik_branches/shown_valid` · `/d6_ik_branches/revealed` ·
`/d1_manipulability/shown` 에 래치돼 있고 패널이 구독한다. **터미널로 바꿔도 버튼 라벨이
실제 상태를 따라온다.**

터미널에는 한계를 벗어난 관절이 도 단위로 찍힌다.

```
찾은 해 8 개 — 관절 한계 안 1 개 / 밖 7 개
  [쓸 수 있다] [0.3, 1.2, -1.0, 0.3, 0.6, -0.0]
  [한계 밖]   [0.3, 2.87, 1.621, 0.303, 2.55, 0.504]
              joint3=+93deg(한계 -170~+0) · joint5=+146deg(한계 -70~+70)
```

#### 탐색을 왜 줄였나 — 400 시드는 낭비였다

한때 시드 400개를 끝까지 돌려 **23.5 초**가 걸렸다. 강의 중에 버튼 하나가 그만큼
물려 있으면 못 쓴다. 재보니 시간이 두 군데로 새고 있었다.

| | |
|---|---|
| 수렴 판정 | `break` 기준이 `1e-11` 인데 채택 기준은 `1e-6` 이라 **사실상 걸리지 않았다.** 이미 수렴한 시드도 250 회를 다 돌았다 |
| 헛도는 시드 | 400개 중 **118개는 끝내 수렴하지 못하는데도** 250 회를 꽉 채웠다 |

수렴 기준을 `1e-9` 로 올리고, 60 회까지 오차가 `1e-3` 밖이면 접는다. 그리고 시드를
줄였다. **관절 한계 안에서 뽑은 무작위 6 자세로 대조한 결과:**

| | 해집합 | 시간 |
|---|---|---|
| 400 시드 · 안 접음 (예전) | 기준 | 4.2 ~ **41.6** 초 |
| 40 시드 · 접음 | **6 자세 전부 동일** | 0.4 ~ 3.8 초 |

**해를 하나도 잃지 않는다.** 40개면 400개가 찾는 것을 다 찾는다 — 해가 8개뿐이라
시드를 더 뿌려도 같은 것을 다시 찾을 뿐이다.

기본값은 여유를 둬 `seeds = 80` 으로 두고, **시간으로도 끊는다** (`time_budget = 3.0`).
특이점 근처는 시드 하나가 250 회를 다 쓰기도 해서 개수만으로는 상한이 안 잡힌다.
로그에 실제로 몇 개를 썼는지 찍는다 — `시드 43/80 개 · 3.1 초`.

```bash
ros2 param set /d6_ik_branches seeds 200        # 더 뒤지고 싶으면
ros2 param set /d6_ik_branches time_budget 0.0  # 0 이면 시간 제한 없음
```

**여기서 말할 것 두 가지.**

「해가 여럿」은 **기구학의 성질**이고 「쓸 수 있는 해가 하나」는 **이 로봇의 성질**이다.
막는 것은 손목이다 — 손목을 뒤집으려면 `joint4`·`joint6` 을 180° 돌려야 하는데 한계가
각각 ±100°·±120° 라 닿지 않고, `joint5` 는 ±70° 뿐이다. 팔꿈치 뒤집기는
`joint3`(−170°~0), 어깨 뒤집기는 `joint2`(0~180°)가 막는다.

그리고 **이 팔은 구형 손목이 아니다** — `joint4`·`joint5` 축은 한 점에서 만나지만
`joint6` 축이 **0.091 m** 떨어져 있다. 해석해 공식이 깔끔하게 떨어지지 않는 구조이고,
레포가 KDL 수치해를 쓰는 이유다 (강의 노트 §7.7).

**⑤ 그런데 방금 맞춘 것은 손끝이 아니다 (§12.3)**

```bash
ros2 run piper_lecture_demo d3_tcp_offset.py
```

빨간 점이 `link6`(MoveIt 이 목표에 맞추는 자리), 파란 점이 손끝, 그 사이가 **0.1358 m** 다.
**rviz 에서 볼 수 없는 것이 이것 하나다** — MotionPlanning 의 목표 마커는 `link6` 에
붙어 있는데 화면에 그렇게 쓰여 있지 않다. TF 두 개만 읽으므로 D1 의 URDF 스택에서도 돈다.

⚠ 실물이 연결된 상태로 띄우지 말 것 (강의 노트 §6-③).

> **D1 · D2 와 MoveIt 스택을 동시에 띄우지 말 것.** 둘 다 `/robot_description` 과
> `/joint_states` 를 발행해 서로 덮어쓴다.

---

## 숫자를 어디에 띄우나 — rviz2 에는 오버레이가 없다

rviz2 기본 플러그인에는 **화면 고정 오버레이(HUD) 디스플레이가 없다.** Grid · RobotModel ·
Marker · PointCloud2 · Image 처럼 3D 씬 안에 그리는 것들뿐이다. 오버레이는 서드파티
(`ros-humble-rviz-2d-overlay-plugins` 등) 를 따로 설치해야 한다.

그래서 D1 은 **숫자판을 이미지로 그려 보낸다.** `/manipulability_readout` 토픽의
`sensor_msgs/Image` 를 rviz 기본 **Image 디스플레이**가 받는다. 3D 텍스트와 달리
카메라를 움직여도 크기·위치가 변하지 않고 팔에 가려지지도 않는다. 추가 설치가 없어
학생 배포 조건(§8 의 「numpy 만」)도 깨지 않는다 — 쓰는 것은 numpy 와 opencv 뿐이고
`cv_bridge` 없이 Image 메시지를 직접 채운다.

| `readout:=` | 결과 |
|---|---|
| `image` (기본) | 2D 숫자판만 |
| `marker` | 예전처럼 3D 텍스트 마커만 |
| `both` | 둘 다 |
| `none` | 숫자 없이 타원체만 |

```bash
ros2 launch piper_lecture_demo d1_manipulability.launch.py readout:=marker
```

숫자판에는 선속도 타원체의 반축 셋이 **막대**로 나온다 (길이는 1.0 을 꽉 찬 것으로 잡아
자세끼리 비교된다). 6자유도 `sigma_min` 이 `1e-3` 아래로 내려가면 **SINGULAR** 가
빨간 글씨로 뜬다 — `wrist` 프리셋에서 타원체는 둥근데 이 글씨만 켜지는 것이
이 데모의 핵심 장면이다.

## 증상 하나 — 「타원체가 로봇을 안 따라간다」

`/joint_states` 에 **발행자가 둘 이상**일 때 나온다. 두 발행자가 서로 덮어써서
`robot_state_publisher` 는 한쪽 값으로 TF 를 만들고 D1 은 다른 쪽 값으로 타원체를 그린다.
둘 다 자기 입력에는 충실하므로 어느 쪽도 에러를 내지 않는다.

흔한 원인은 **앞서 띄운 데모가 아직 살아 있는 것**이다. MoveIt 스택의
`joint_state_broadcaster` 와 먼저 띄운 D1 의 슬라이더 GUI 가 모두 `/joint_states` 를
발행한다. **두 스택을 같이 띄우면 반드시 이렇게 된다.**

D1 은 이것을 감시해서 경고한다:

```
/joint_states 에 발행자가 2 개다. 서로 덮어쓰기 때문에 로봇 모델과 타원체가 따로 논다.
```

확인과 정리:

```bash
ros2 topic info /joint_states --verbose     # 발행자가 몇 개인지
pkill -f joint_state_publisher              # 남은 것 정리
```

> 이것은 버그가 아니라 이 레포의 성질이다. `/joint_states` 가 **명령 토픽**이라
> 아무나 쓸 수 있고, 마지막에 쓴 쪽이 이긴다 — 강의 M6-1 이 다루는 바로 그 사실이다.

## D6 을 왜 Marker 로 그리나 — MoveIt 디스플레이는 SRDF 를 요구한다

처음에는 MoveIt 의 **Trajectory 디스플레이**로 그렸다. 해 목록을 궤적처럼 담아 보내면
`Show Trail` 이 모든 자세를 한 화면에 세워 주기 때문이다. 그런데 그 디스플레이는
로봇 모델을 **URDF 와 SRDF 둘 다**로 만든다. D1 스택에는 `robot_description` 만 있고
`robot_description_semantic` 이 없으므로 이렇게 죽는다.

```
Could not find parameter robot_description_semantic and did not receive
robot_description_semantic via std_msgs::msg::String subscription within 10.000000 seconds.
[moveit_rdf_loader.rdf_loader]: Unable to parse SRDF
```

디스플레이에 빨간 오류 표시만 남고 **아무것도 그려지지 않는다.**

그래서 링크 메시를 Marker 로 직접 놓는 쪽으로 바꿨다. 얻은 것이 셋이다.

| | |
|---|---|
| **스택을 안 가린다** | D1 · MoveIt 어느 쪽에서도 같게 보인다. D6 이 MoveIt 을 아예 안 쓴다 |
| **안 깜박인다** | 애니메이션이라는 것이 없다. 아래 「증상 둘」 참조 |
| **색·투명도를 쥔다** | 초록/빨강과 알파를 노드가 정한다 (`alpha` 파라미터) |

> ⚠ **DELETEALL 마커에 `ns` 를 넣지 말 것.** 지우기는 원래 네임스페이스를 가리지 않는데
> `ns` 를 채우면 rviz 가 디스플레이를 **오류 상태로 표시한다.** 그런데 **그리기는 정상으로
> 되기 때문에** 화면만 보고는 원인을 찾을 수 없다 — 트리의 아이콘만 빨갛다.
> 마커를 하나씩 빼며 확인한 결과다 (`ns` 를 비우면 아이콘이 깨끗해진다).

## 증상 둘 — 「로봇이 계속 깜박거린다」

MoveIt 의 **Trajectory 디스플레이가 궤적을 계속 재생**하기 때문이다. D6 가 내는 것은
궤적 모양을 하고 있을 뿐 실제로는 해 목록인데, 디스플레이는 그것을 애니메이션으로
돌린다. `Loop Animation` 이 켜져 있으면 끝나고 다시 처음부터 돌아 **유령 로봇이
나타났다 사라졌다** 한다. 해가 하나뿐이어도 그 하나가 깜박인다.

화면을 연속 캡처해 프레임 간 변화를 재보면 분명하다 (rviz 3D 영역, 0.15 초 간격).

| | 바뀌는 화소 |
|---|---|
| `Loop Animation: true` | 매 프레임 **6.05 %** — 정확히 두 상태를 오간다 |
| `Loop Animation: false` | **0.00 %** — 완전히 정지 |

**지금은 이 문제가 없다.** IK 해를 Marker 로 그리면서 애니메이션이라는 것 자체가
사라졌다 (위 절). 같은 방법으로 다시 재면 **0.00 %** 다.

남아 있는 Trajectory 디스플레이는 MoveIt 스택의 `MotionPlanning`(계획된 경로) 하나뿐이고,
그쪽은 **재생되는 것이 맞다** — 계획한 궤적을 보여주는 것이 일이기 때문이다.

## 실물 팔에 물리기 — `real:=true`

```bash
ros2 launch piper_lecture_demo moveit_demo.launch.py real:=true
ros2 launch piper_lecture_demo moveit_demo.launch.py real:=true stub:=true   # 실물 없이
```

`mock_components/GenericSystem` 자리를 `piper_real_adapter.py` 가 대신한다. MoveIt 이
부르는 것과 piper 드라이버가 가진 것 사이가 비어 있어서, 그 사이를 메우는 노드다.

| MoveIt 이 부르는 것 | piper 드라이버가 가진 것 |
|---|---|
| `/arm_controller/follow_joint_trajectory` (joint1~6) | `joint_ctrl_single` — JointState 하나가 목표 자세 하나 |
| `/gripper_controller/follow_joint_trajectory` (joint7) | (같음. `position[6]` 이 그리퍼) |
| `/joint_states` (상태) | `joint_states_feedback` |

어댑터가 하는 일은 셋이다.

1. **상태 다리** — `joint_states_feedback` → `/joint_states`. 드라이버는 일곱째를
   **`gripper`** 라고 부르는데 모델은 `joint7` 이라 이름을 바꾸고, `joint8` 도 대칭으로
   채운다. **그래서 「`Missing joint8`」 경고가 여기서 사라진다** (측정: 0 건).
2. **명령 다리** — `joint_ctrl_single` 로 목표를 쏜다. `velocity[6]` 에 속도(%)를
   **반드시** 채운다.
3. **궤적 실행** — 액션 서버 둘. 드라이버의 `JointCtrl` 은 보간이 없으므로,
   경유점 사이를 어댑터가 보간해 100 Hz 로 쏘는 것이 곧 실행이다.

### 안전장치 넷

| | |
|---|---|
| **속도** | `speed_percent` (기본 **20**). 비워 두면 드라이버가 `MotionCtrl_2(…, 100)` 으로 **전속**을 건다 |
| **계단 명령** | 첫 경유점이 지금 자세에서 `max_step_rad`(기본 0.35 rad = 20°) 넘게 떨어져 있으면 goal 을 거절한다 |
| **enable** | goal 을 받으면 `enable_srv` 를 먼저 부른다. 실패하면 움직이지 않는다 |
| **취소** | 취소가 오면 지금 자세를 목표로 다시 쏴 **그 자리에 세운다** |

### ⚠ remap 을 하지 않는 이유

piper 의 `start_single_piper.launch.py` 는 `joint_ctrl_single` 을 `/joint_states` 로
remap 한다. 그 상태로 어댑터를 켜면 **상태 다리가 내보낸 값이 그대로 명령으로
되돌아가 루프가 된다.** 그래서 `real:=true` 는 그 런치를 쓰지 않고 드라이버 노드를
remap 없이 직접 띄운다. 확인: `/joint_states` 발행자 1 · 구독자 3(rsp · move_group ·
D6), `/joint_ctrl_single` 발행자 1(어댑터) · 구독자 1(드라이버).

### 실물 없이 어디까지 확인했나

`can0` 가 없는 상태에서 `stub:=true` 로 전 구간을 돌렸다.

| 확인된 것 | |
|---|---|
| 액션 → 보간 → 명령 | 궤적 goal 이 `SUCCEEDED`, 스텁 팔이 목표에 정확히 도달 |
| 속도 필드 | 스텁이 받은 모든 명령에 **20 %** 가 채워져 있다 (100 % 아님) |
| enable | 첫 goal 에서 `enable -> True` |
| 되먹임 없음 | 위 발행자/구독자 수 |
| `joint8` | `/joint_states` 에 여덟 관절, `Missing joint8` **0 건** |
| MoveIt 전 구간 | `d5_interp_compare` 의 계획·실행이 끝까지 돈다 (`Execute request success!` ×4) |
| 계단 방어 | 72° 떨어진 goal 을 `ABORTED` + 사유 문자열로 거절 |

**확인되지 않은 것** — CAN 타이밍 · 실제 추종 오차 · 관절 한계에서의 거동 ·
그리퍼 힘. 스텁은 토픽 인터페이스만 흉내 내고 1차 지연으로 따라가는 시늉을 할 뿐이다.
**실물에서 다시 재야 한다.**

> D1 스택은 실물에 물리지 않았다. `/joint_states` 를 명령으로 쓰는 구조라 물리면
> 프리셋 버튼이 그대로 팔을 움직이는데, 특이점 자세가 프리셋에 들어 있고 속도·정지
> 수단이 없다. 아래 경고가 그대로 유효하다.

## ⚠ 실물을 연결한 채로 띄우지 말 것

이 레포에서 `/joint_states` 는 상태 토픽이 아니라 **명령 토픽**이다
(`start_single_piper.launch.py` 의 `remappings`). 실물이 붙어 있으면
D1 의 슬라이더와 프리셋이 **진짜 팔을 움직인다.**

이 데모는 전부 `mock_components/GenericSystem` 위에서 돈다. 실물이 필요 없다.

---

## D2 의 두 구름이 무엇인가 — 이름에 주의

| 토픽 | 뜻 |
|---|---|
| `/workspace/reachable` | **닿기만 하면 되는 영역.** 자세는 따지지 않는다 |
| `/workspace/oriented` | **정해둔 접근 방향으로 닿는 영역** |

판정식은 이것 하나다.

```python
# TCP 프레임의 z 축이 월드 -Z(바닥) 과 이루는 각이 허용치 안인가
T[:3, 2] @ [0, 0, -1] >= cos(approach_tol_deg)
```

`link6` 의 `+z` 가 손가락 쪽을 가리키므로(URDF 로 확인 — `link6 -> link7` 방향과의
내적이 **1.0000**) 이 축이 곧 접근 방향이다. 축 **둘레** 회전(`joint6`)은 자유다 —
대칭 2지 그리퍼의 접근에는 상관이 없기 때문이다.

> ⚠ **교과서의 dexterous workspace 가 아니다.** 그쪽은 「**모든** 자세로 닿는 점의
> 집합」이고 6축 팔에서는 대개 텅 빈다. 여기서 내는 것은 「**작업이 정한 접근 방향
> 하나**로 닿는가」이고, 그래서 표시 이름도 `oriented` 로 두었다. 강의에서 말하려는
> *"닿기만 하는 영역과 원하는 자세로 닿는 영역은 다르다"* 에는 이쪽이 맞다.

허용각은 `approach_tol_deg` 로 바꾼다. 비율이 여기에 크게 붙으므로,
**허용각을 밝히지 않은 「작업영역의 몇 %」 는 아무 뜻이 없다.**

| 허용각 | 5° | 15° | **25°** | 35° | 45° | 60° | 90° |
|---|---|---|---|---|---|---|---|
| 부피 기준 | 0.9 % | 2.7 % | **4.3 %** | 10.1 % | 15.5 % | 27.7 % | 54.6 % |

(현재 기본 격자 `[11, 13, 13, 5, 5, 1]`, 2 cm 복셀 기준)

## 특이점 판정 기준 — 어느 지표로, 어디서 자를 것인가

### manipulability 와 특이점의 관계

**타원체는 관절속도 단위구 `‖q̇‖ ≤ 1` 을 J 가 보낸 상이다.** 반축 길이가 σ 이고,
`w = √det(JJᵀ) = σ₁σ₂σ₃` 는 그 **부피**다.

특이점은 「어떤 방향으로는 아무리 관절을 돌려도 못 움직이는 자세」다. 그 방향의 반축이
0 이 되므로 타원체가 원반으로, 다시 선분으로 **차원이 하나씩 내려앉는다.** 그래서

```
det J = 0   ⟺   σ_min = 0   ⟺   w = 0   ⟺   특이점
```

**참·거짓 판정으로는 셋이 완전히 같다.** M3-4 를 M3-3 의 극한으로 배치한 구성이 옳다.

**그런데 「얼마나 가까운가」를 재는 데는 다르다.** w 는 곱이라 두 원인이 섞인다.

| w 가 작다 | 원인 | 특이점인가 |
|---|---|---|
| 한 축이 죽어간다 | 차원이 무너지는 중 | **그렇다** |
| 세 축이 다 작다 | 그냥 느린 자세 | 아니다 |

**부피는 원인을 구분하지 못한다.** 이 로봇에서 실제로 찾은 반례가 프리셋으로 들어 있다.

| preset | σ (선속도) | **w** | cond | σ_min (6×6) |
|---|---|---|---|---|
| `elbow` | 0.929 / 0.755 / **0.012** | **0.00841** | 77.4 | 0.000050 |
| `slow` | 0.443 / 0.283 / **0.068** | **0.00851** | 6.5 | 0.0238 |

**부피가 1 % 이내로 같은데 조건수는 12 배, 6×6 `σ_min` 은 475 배 차이난다.**
`elbow` 는 특이점이고 `slow` 는 그냥 느린 자세다. 두 프리셋을 연달아 재생하면
이 대비가 한 화면에서 끝난다.

무작위 40,000 자세에서 `log w` 와 `log σ_min` 의 상관은 **0.6483** 에 그친다
(`log cond` 와는 −0.9465 였다). **`σ_min` 이 더 작은데 `w` 는 3 배 큰 자세 쌍**도 실재한다.

한 줄로 정리하면:

> **특이점은 부피가 0 이 되는 것이 아니라 차원이 하나 사라지는 것이다.
> 부피가 0 이 되는 것은 그 결과일 뿐이다.**

그래서 **판정은 최단 반축(`σ_min`)으로, `w` 는 「이 자세에 얼마나 여유가 있나」의 총량
지표로** 쓰는 것이 맞다. 특이점 *근처* 에서는 `w ≈ σ₁σ₂ · σ_min` 이라 둘이 함께 0 으로
가지만, 그 비례상수 `σ₁σ₂` 가 0.026 ~ 0.715 로 28 배나 흔들리므로 **고정된 `w` 임계값은
고정된 `σ_min` 임계값이 아니다.**

⚠ 실무적 함정 하나 더 — **`w` 는 σ 의 세제곱 차원**이라 기준점이나 단위가 바뀌면
세제곱으로 움직인다. 이 로봇에서 기준점을 플랜지 → TCP 로 옮겼을 뿐인데
`w` 가 0.0183 → 0.0569 로 **3.1 배**가 됐다 (σ 는 1.2 ~ 2 배).

### 무엇이 「표준」인가 — 셋이 병존한다

| 지표 | 쓰는 자리 |
|---|---|
| `w = √det(JJᵀ)` (manipulability) | 타원체의 부피. 설계 · 성능 지표 |
| `κ = σ_max/σ_min` (condition number) | 수치 조건, 등방성 설계, **실시간 감속 임계값** |
| `σ_min` | 특이점까지의 거리 |

**`σ_min` 과 `κ` 는 경쟁 관계가 아니다.** Eckart–Young 정리에 의해

- `σ_min(J)` = J 를 랭크 결손으로 만드는 최소 섭동의 크기 → 특이점까지의 **절대 거리**
- `1/κ = σ_min/σ_max` → 같은 것을 J 의 크기로 나눈 **상대 거리**

즉 **같은 질문의 절대판과 상대판**이다. 「어느 쪽이 표준이냐」가 아니라 「절대로 잴 것이냐
상대로 잴 것이냐」가 실제 선택이다. 이 로봇에서 둘의 상관이 −0.9465 로 나오는 것도
이것으로 설명된다 — `σ_max` 가 좁으면 절대 ≈ 상대 × 상수다.

> ⚠ 위 지표들의 **원 출처(Yoshikawa · Salisbury/Craig · Klein/Blake · Merlet 등)는
> 이 저장소에서 검증하지 않았다.** 강의 자료에 인용으로 올릴 때는 확인이 필요하다.
> Eckart–Young 관계와 이 문서의 측정값은 확인된 것이다.

### σ_min 과 cond 는 다른 것을 잰다

| | 재는 것 | 성질 |
|---|---|---|
| `σ_min` | **최악 방향의 속도 이득.** `‖q̇‖ ≥ ‖v‖ / σ_min` 이므로 관절 속도가 얼마나 폭발하는지 | **차원이 있다.** 로봇 크기와 단위에 따라 값이 달라진다 |
| `cond = σ_max/σ_min` | **비등방성.** 가장 빠른 방향과 가장 느린 방향의 비 | 같은 단위끼리의 비라 **무차원·스케일 불변** |

### ⚠ 6×6 Jacobian 을 그대로 쓰면 단위에 끌려다닌다

위 3행은 `m/s per rad/s`, 아래 3행은 무차원이다. **단위가 섞인 행렬의 특이값은 물리적
의미가 없다.** 길이 단위만 m → mm 로 바꿔 재보면:

| 자세 | | σ_min | cond |
|---|---|---|---|
| `good` | 6×6, m | 0.1094 | **16.4** |
| `good` | 6×6, mm | 0.2805 | **1829.6** |
| `good` | 선속도만, m | 0.1983 | **2.5889** |
| `good` | 선속도만, mm | 198.25 | **2.5889** |

**같은 로봇, 같은 자세인데 6×6 조건수가 112배 달라진다.** 선속도만 쓰면 cond 는 정확히
불변이고 σ_min 만 단위대로 1000배가 된다.

즉 6×6 의 절대값은 *"미터로 쟀다"* 는 약속 위에서만 의미가 있다. `moveit_servo` 의
17 · 30 도 그 약속 위의 숫자다. MoveIt 의 `KinematicsMetrics` 가 `translation = true`
플래그를 두고 있는 이유가 이것이다.

### 이 로봇에서는 두 지표가 사실상 같은 정보다

무작위 30,000 자세에서 선속도 Jacobian 의 **σ_max 범위가 0.293 ~ 0.721** 로 2.5배 안에
갇혀 있다. σ_max 가 거의 상수면 `cond ≈ 상수 / σ_min` 이 된다. 실제로
**`log σ_min` 과 `log cond` 의 상관계수가 −0.9465** 이고, 80,000 자세를 뒤져도
「σ_min 은 작은데 cond 는 작은」 자세도 「cond 는 큰데 σ_min 도 큰」 자세도 **0건**이다.

→ **이 팔에 한해서는 어느 쪽을 봐도 결론이 같다.** 다만 그것이 우연(σ_max 가 좁다)임을
알고 써야 한다. 다관절·가변 스케일 로봇에서는 갈린다.

### 어디서 자를 것인가 — MoveIt 이 실제로 실패하는 지점

무작위 400 자세(**전부 유효한 `q` 의 FK 이므로 해가 반드시 있다**)를 `/compute_ik` 에
넣고, 정답과 무관한 초기값을 주고, 충돌검사는 끄고 측정했다.

| `cond` (6×6, m) | 표본 | IK 성공률 |
|---|---|---|
| 10 ~ 17 | 56 | **100 %** |
| 17 ~ 30 | 98 | **100 %** |
| 30 ~ 100 | 148 | **100 %** |
| 100 ~ 1000 | 83 | 97.6 % |
| 1000 이상 | 15 | **66.7 %** |

| `σ_min` (6×6, m) | 표본 | IK 성공률 |
|---|---|---|
| < 0.001 | 11 | **81.8 %** |
| 0.001 ~ 0.01 | 58 | 91.4 % |
| 0.01 이상 | 331 | **100 %** |

**권고 기준 (이 로봇, 미터 단위):**

| `cond` (6×6) | `σ_min` (6×6) | 판정 | 근거 |
|---|---|---|---|
| < 100 | > 0.01 | **OK** | IK 100 % |
| 100 ~ 1000 | 0.001 ~ 0.01 | **주의** | IK 약 98 % — 가끔 실패한다 |
| > 1000 | < 0.001 | **특이점** | IK 약 67 % — 셋 중 하나가 실패한다 |

D1 의 숫자판 아래 판정 밴드가 이 기준을 그대로 쓴다.

### 임계값은 「잡기 나름」이 아니라 목적에서 유도된다

무엇을 막으려는지 정하면 숫자가 따라 나온다. 경로가 셋이고 **서로 다른 값을 낸다.**

**(1) 관절 속도 한계에서.** 최악 방향으로 손끝 속도 `v` 를 내려면 `‖q̇‖ ≥ v / σ_min` 이다.
이 로봇의 관절 한계가 5 rad/s (joint6 만 3) 이므로 임계값이 그대로 나온다 — `σ_min ≥ v/5`.

| 원하는 손끝 속도 | 필요한 `σ_min` (선속도) | 작업공간에서 미달인 자세 |
|---|---|---|
| 0.05 m/s | ≥ 0.010 | 0.3 % |
| **0.10 m/s** | **≥ 0.020** | **1.0 %** |
| 0.25 m/s | ≥ 0.050 | 4.8 % |
| 0.50 m/s | ≥ 0.100 | 21.4 % |

프리셋에 대보면 바로 읽힌다 (TCP 기준):

| preset | `σ_min` | 0.1 m/s 에 필요한 관절속도 | |
|---|---|---|---|
| `good` | 0.2373 | 0.42 rad/s | 여유 |
| `stretch` | 0.0851 | 1.18 rad/s | 여유 |
| **`elbow`** | **0.0120** | **8.34 rad/s** | **한계 5 초과 — 0.1 m/s 조차 불가** |

*"이 자세에서는 손끝을 0.1 m/s 로 못 움직인다"* 가 임의로 고른 임계값이 아니라
**로봇 제원에서 나온 결론**이 된다.

**(2) IK 수렴에서.** 위 측정대로 `cond > 1000` / `σ_min(6×6) < 0.001` 에서 성공률 67 %.

**(3) 정확도에서.** 관절 오차가 손끝 오차로 증폭되는 비가 `κ` 다.

**세 목적이 20배 차이 나는 숫자를 낸다** — 속도 목적의 `σ_min` 임계값은 0.02, IK 목적은
0.001 이다. 그래서 학생에게 줄 문장은 「기준은 잡기 나름」이 아니라

> **기준은 목적에서 나온다. 무엇을 막으려는지 먼저 정하고 그 목적의 식에서 숫자를 뽑아라.
> 셋을 섞지 마라.**

정말로 판단에 맡겨지는 것은 **안전 여유를 얼마나 둘지**와 **감속 시작(soft)과 정지(hard)를
어디에 둘지** 두 가지다.

### `moveit_servo` 의 17 · 30 은 다른 이야기다

`moveit_servo` 는 헤더에 *"Halt if condition(Jacobian) > hard_stop_singularity_threshold"*
라고 명시하고, 기본값은 **17에서 감속 · 30에서 정지**다. 위 표에서 보듯 그 구간의 IK
성공률은 **100 %** 다. 서보가 막는 것은 IK 실패가 아니라 **명령 속도의 폭발**이므로
기준이 훨씬 보수적인 것이 맞다. **두 임계값을 섞어 쓰면 안 된다.**

> ⚠ 그리고 17 · 30 은 **Panda 예제 설정값**이다. 이 로봇에 그대로 쓰면 무작위 자세의
> **84.8 % 가 17 을 넘고 60.8 % 가 30 을 넘는다** — 작업공간 대부분이 감속·정지 구간이
> 된다. 조건수 중앙값이 40.3 이기 때문이다. 임계값은 로봇마다 다시 잡아야 한다.

### MoveIt 의 이름이 헷갈린다

| API | 실제로 돌려주는 것 |
|---|---|
| `getManipulabilityIndex()` | `√det(JJᵀ)` — 타원체의 부피 |
| `getManipulability()` | **`σ_min/σ_max`** — 조건수의 **역수**인데 인자 이름은 `condition_number` 다 |
| 두 함수 공통 | `translation = true` 를 주면 선속도 3행만 쓴다 — 단위 문제를 피하려면 켤 것 |

## 확인된 수치

전부 이 레포의 URDF 와 MoveIt 설정으로 실측한 값이다.

### 기준점(TCP)은 플랜지가 아니다

D1 · D2 는 **손끝(TCP)** 을 기준으로 잰다. `link6`(플랜지) 기준이 기본값이 아닌 이유는
「닿는다」도 「잘 움직인다」도 손끝의 이야기이기 때문이다.

TCP 는 `link6` 프레임에서 **`(0, 0, 0.1358) m`** 이다. `link7` · `link8` 의 STL 을
`link6` 좌표계로 옮기면 손가락이 `z = 0.0593 ~ 0.1358` 을 차지하고, 두 손가락 사이의
파지 중심이 `z = 0.1358` 에 온다. 이 값은 `joint7` 원점과 같다.

D1 은 이 점을 **`tcp` 프레임**으로도 내보낸다 (`link6` 의 자식). rviz 의 TF 디스플레이에서
보이고 다른 노드가 `lookup_transform` 으로 가져다 쓸 수 있다.

`tcp_offset:="[0.0, 0.0, 0.0]"` 를 주면 플랜지 기준으로 되돌아간다.

기준점을 옮기면 **Jacobian 자체가 바뀐다** — 선속도 열이 `zᵢ × (pₑ − pᵢ)` 이고 `pₑ` 가
바로 그 점이기 때문이다. 팔을 편 자세에서 비교하면:

| 기준점 | σ (선속도) | cond | w |
|---|---|---|---|
| 플랜지 `link6` | 0.704 / 0.616 / **0.042** | 16.7 | 0.0183 |
| **TCP** | 0.895 / 0.747 / **0.085** | **10.5** | 0.0569 |

손끝으로 옮기면 `joint4` · `joint5` 의 지렛대가 길어져 (기여 0.044 → 0.109,
0.091 → 0.227) 타원체가 눈에 띄게 둥글어지고 부피는 3배가 된다.

`joint6` 의 선속도 기여는 **두 경우 모두 정확히 0** 이다. TCP 가 `joint6` 축 위에
있기 때문이다 — 대칭 그리퍼의 파지 중심은 손목 롤 축 위에 있으므로, 손목을 굴려도
파지점은 제자리에 있고 **자세만 바뀐다.**

### 특이점 세 자리 — 전부 존재한다

`sigma_min_6d` 는 6×6 Jacobian 의 최소 특이값, `cond_v` 는 선속도 타원체의 조건수다.
아래는 전부 **TCP 기준**이다.

| preset | 자세 | cond_v | σ_min (6D) | 읽는 법 |
|---|---|---|---|---|
| `good` | 기준 자세 | 2.8 | 0.104 | 타원체가 둥글다 |
| `stretch` | 팔을 편다 | 10.5 | 0.0075 | 타원체가 원반이 된다 |
| `elbow` | `joint3 = −2.85` + `joint5 = 0` | 77.4 | 0.000050 | 타원체가 선분이 된다 |
| `wrist` | `joint5 = 0` | **3.2** | 0.000057 | **타원체는 둥근데 6자유도는 죽어 있다** |
| `home` | 모든 관절 0 | 7.8 | 0.000061 | **이 로봇의 영자세가 곧 손목 특이점이다** |
| `shoulder` | 손목 중심이 `joint1` 축 위 | 5.1 | 0.000076 | 수평반경 0 |

- **손목**: `joint5 = 0` 이면 `z₄` 와 `z₆` 가 평행해진다 (`|z₄·z₆| = 1.0000`).
  **나머지 관절값과 무관하다** — 무작위 400자세에서 σ_min 최대 6.2e-5.
- **팔꿈치**: `joint3 = −2.831 rad` (−162.2°). 역시 나머지와 무관하며 σ_min 최대 8e-6.
  관절 한계 −2.967 보다 **안쪽**이라 실제로 지나갈 수 있다.
- **어깨**: 손목 중심이 `joint1` 축 위에 올 때. 손목·팔꿈치 특이점에서 떨어뜨린
  표본 2,283개에서 σ_min 최대 0.0076.

`wrist` 와 `elbow` 를 나란히 보이는 것이 이 데모의 핵심이다 — 둘 다 σ_min ≈ 0 인데
타원체는 하나는 둥글고 하나는 선분이다. **하나만 보면 놓친다.**

### 타원체가 얇아 보이는 이유 — 바늘이 아니라 원반이다

팔을 편 자세의 σ 는 `0.895 / 0.747 / 0.085` 다. 큰 축이 **둘** 이므로 이것은 바늘이
아니라 **원반**이고, 옆에서 보면 얇지만 위에서 보면 넓다.

주축의 방향을 보면 원인이 분명하다 (팔이 `+x` 로 뻗은 자세):

| | σ | 방향 |
|---|---|---|
| σ₁ | 0.895 | 수직 (`|u·ẑ| = 0.999`) |
| σ₂ | 0.747 | **접선** (`|u·t̂| = 1.000`) — `joint1` 이 만드는 방향 |
| σ₃ | 0.085 | **반경** (`|u·r̂| = 0.999`) — 팔이 뻗은 그 방향 |

`joint1` 이 좌우(접선) 운동을 만든다는 직관은 **맞다.** 그 기여는 정확히 반경 `r` 과
같고(뻗을수록 오히려 **커진다** — 0.391 → 0.615 → TCP 기준 0.739), σ₂ 가 바로 그 축이다.

얇아지는 것은 **다른 축**이다. 팔이 펴질수록 죽는 것은 **반경 방향**, 즉 팔이 뻗은 자기
방향으로 더 나가는 능력이다. 그것이 작업영역 경계이고 팔꿈치 특이점이다.

### 작업영역 (D2 기본 격자 `[11, 13, 13, 5, 5, 1]`, 46,475 자세, TCP 기준)

| | |
|---|---|
| 최대 수평 반경 | **0.761 m** |
| base_link 원점 기준 거리 | 0.010 ~ 0.885 m |
| 높이 | −0.393 ~ 0.884 m |
| 조건수 | 중앙값 4.4 / 90 % 지점 10.3 |
| 접근 방향(바닥 ±25°)까지 맞는 영역 | **닿는 영역의 4.3 %** (2 cm 복셀, 1,292 / 29,709) |
| 계산 시간 | 16 초 |

> 최대 수평 반경 **0.761 m** 는 강의 노트의 링크 길이 합 **0.764 m** 와 사실상 같다.
> 노트의 그 숫자는 **플랜지가 아니라 손끝 기준**이었다는 뜻이다 — 플랜지 기준으로
> 다시 재면 0.627 m 로, 노트 값과 0.14 m 차이가 난다.

#### ⚠ 이 비율을 인용할 때 반드시 붙일 것 두 가지

**① 「부피」로 세야 한다.** 자세의 개수를 세면 안 된다. 한 점을 여러 자세가 찍으므로
도달 자세가 많은 자리가 과대 반영된다. 같은 격자에서 자세로 세면 4.9 %, 부피로 세면
4.3 % 다. 강의에서 말하려는 것은 **영역의 크기**지 자세의 개수가 아니다.

**② 격자에 심하게 휘둘린다.** 특히 `joint4` 를 고정하면 접근 방향이 인위적으로 아래로
쏠린다.

| 격자 | 부피 기준 비율 |
|---|---|
| `joint4`·`joint6` 고정 (예전 기본값) | **15.7 %** |
| `joint4` 샘플링 (현재 기본값) | **4.3 %** |
| 손목을 더 촘촘히 (`[9,13,13,7,7,1]`) | 6.4 % |

**이 값은 하한이다.** 손목을 촘촘히 샘플링할수록 올라간다 — 거친 격자는 「그 방향으로
닿을 수 있는데 못 찾은」 점을 놓치기 때문이다. 수렴값을 원한다면 손목 샘플을 늘려야 한다.

> `joint6` 은 샘플링할 필요가 없다. 자기 축 회전이라 **TCP 위치도 접근 방향도 바꾸지
> 않는다** (URDF 로 확인). 샘플 수만 늘리고 결과는 같다.

### 보간 두 종 (D5)

| | 손끝 이동 | 직선에서 최대 이탈 |
|---|---|---|
| 관절 보간 (OMPL) | 0.267 m (직선의 106.5 %) | **0.0397 m** |
| 손끝 직선 (`computeCartesianPath`) | 0.251 m | 0.0000 m |

작업영역 경계 쪽으로 직선을 요구하면 `fraction = 0.62` 에서 멈춘다.
반경 0.391 m 에서 출발해 **0.546 m 에서 멈추는데, 위치만 따지면 0.627 m 까지 갈 수 있다.**
직선 보간은 손끝 *방향*도 고정한 채 가기 때문이다.

---

## 이 패키지가 덮는 piper 설정 — 하나도 없다

`piper_description` 과 `piper_with_gripper_moveit` 을 손대지 않고 그대로 쓴다.

한때 IK 제한시간을 `0.005 s -> 0.05 s` 로 덮었으나 **재보니 아무 차이가 없어서 뺐다.**

| | 0.005 s (원본) | 0.05 s |
|---|---|---|
| 단일 IK 성공률 (400 자세) | 98.2 % | 98.2 % |
| D5 의 카테시안 `fraction` | 0.627 | 0.627 |
| 실제로 간 거리 | 0.1569 m | 0.1569 m |

소수점까지 같다. 「제한시간이 모자라 경로가 끊긴다」는 그럴듯한 추측이었을 뿐
이 로봇·이 데모에서는 사실이 아니었다.

## 레포에서 발견한 것

### `joint_limits.yaml` 에 가속도 한계가 없다

7개 관절 전부 `has_acceleration_limits: false` 다. 그 결과:

- **Pilz 플래너(PTP · LIN)가 전혀 동작하지 않는다.** 파이프라인은 정상 로드되고
  `Registered Algorithm [PTP] / [LIN] / [CIRC]` 까지 찍히지만, 계획을 요청하면
  `Exception caught: 'acceleration limit not set for group arm'` 로 실패한다.
- 시간 파라미터화가 매번 경고를 낸다 —
  `Joint acceleration limits are not defined. Using the default 1 rad/s^2`.

이 데모는 Pilz 를 쓰지 않으므로(직선은 `computeCartesianPath` 가 만든다) 문제가 되지
않지만, 학생이 PTP/LIN 을 쓰려 하면 여기서 막힌다. 가속도 한계를 채운
`joint_limits.yaml` 을 주면 PTP·LIN 모두 정상 동작하는 것까지 확인했다.

### `joint8` 이 `ros2_control` 에 빠져 있다

`move_group` 이 **1 초에 한 번씩** 이 경고를 낸다. 강의 내내 터미널이 이것으로 덮인다.

```
The complete state of the robot is not yet known.  Missing joint8
```

URDF 에는 `joint8`(그리퍼의 반대쪽 손가락)이 **`<mimic>` 없는 독립 prismatic 관절**로
들어 있는데, `piper.ros2_control.xacro` 는 `joint1` ~ `joint7` 만 등록한다. 그래서
`joint_state_broadcaster` 가 7 개만 내보내고, 8 관절 모델을 들고 있는 `move_group` 은
상태가 영영 채워지지 않는다. 계획은 되지만 경고가 멈추지 않고 `link8` 은 추종되지 않는다.

**고치면 없어지는 것까지 확인했다.** `piper.ros2_control.xacro` 에 `joint8` 블록을
(`joint7` 것과 같은 모양으로) 더하고 `initial_positions.yaml` 에 `joint8: 0` 을 넣으면,
`/joint_states` 에 `joint8` 이 나오고 경고는 **0 건**이 된다.

⚠ 다만 그것은 **piper 설정을 고치는 일**이다. 이 패키지는 piper 설정을 하나도 덮지
않는다는 원칙이라(아래 절) 여기서는 고치지 않고 적어만 둔다.

### `ompl_planning.yaml` 이 없다

그룹별 플래너 설정이 비어 있어 `setPlannerId("RRTConnectkConfigDefault")` 같은 지정이
무시된다 (`Cannot find planning configuration for group 'arm' ... Will use defaults`).
기본값이 RRTConnect 라 결과는 같지만, 데모 코드는 없는 이름을 부르지 않는다.

### `sensors_3d.yaml` 의 octomap updater 가 로드되지 않는다

`occupancy_map_monitor/DepthImageOctomapUpdater` 클래스가 없다는 에러가 move_group
기동 때마다 두 번 뜬다. 충돌 물체를 손으로 얹는 D3 에는 영향이 없다.

---

## 파일

```
piper_lecture_demo/
├── piper_lecture_demo/kinematics.py   URDF -> FK · Jacobian (numpy 뿐)
├── scripts/
│   ├── d1_manipulability.py           D1 본체
│   ├── d1_preset.py                   D1 자세 드라이버 (프리셋 자세를 내보낸다)
│   ├── d1_goto.py                     D1 자세 전환 명령 (별도 실행)
│   ├── d2_workspace.py                D2 본체
│   ├── d3_obstacle.py                 D3 충돌 물체 spawn/despawn
│   ├── d3_tcp_offset.py               D3 link6 vs 손끝 표시 (TF 만 읽는다)
│   ├── d6_ik_branches.py              D6 IK 해 전수 탐색 (상주 · 서비스)
│   ├── piper_real_adapter.py          MoveIt 스택을 실물 팔에 물린다
│   ├── piper_driver_stub.py           드라이버 대역 (실물 없이 시험용)
│   └── lecture_panel.py               강의용 버튼 창 (PyQt5)
├── src/
│   └── d5_interp_compare.cpp          D5
├── launch/                            스택이 둘이라 런치도 둘이다
│   ├── d1_manipulability.launch.py    D1 · D2 · D6 + 패널 (URDF 만)
│   └── moveit_demo.launch.py          piper 런치 + 우리 rviz 설정 + D6 + 패널
└── config/
    ├── d1_manipulability.rviz         타원체 · 숫자판 · IK 해 · 작업영역
    └── moveit_demo.rviz               piper 의 moveit.rviz + Marker 디스플레이
                                       (piper 런치에 rviz_config 로 넘긴다)
```

## 빌드

```bash
cd ~/dev/ws && colcon build --packages-select piper_lecture_demo --symlink-install
```

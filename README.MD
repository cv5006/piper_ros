# piper 로봇팔 ROS(humble)

![ubuntu](https://img.shields.io/badge/Ubuntu-22.04-orange.svg)
![bash](https://img.shields.io/badge/bash-green.svg)

[EN](README(EN).MD)

python_version >= "3.10"

Test:

|PYTHON |STATE|
|---|---|
|![humble](https://img.shields.io/badge/ros-humble-blue.svg)|![Pass](https://img.shields.io/badge/Pass-blue.svg)|

```shell
git clone https://github.com/agilexrobotics/piper_ros.git
```

```shell
cd piper_ros
git checkout humble
```

|설명 |문서|
|---|---|
|Moveit|[Moveit README](src/piper_moveit/README.md)|
|시뮬레이션|[시뮬레이션 README](src/piper_sim/README.md)|

## 0 URDF 버전 주의사항

S-V1.6-3 펌웨어 버전 이전의 DH 파라미터는 각 관절이 리밋(limit)에 접촉한 위치를 초기 좌표로 하여 구한 값입니다.

S-V1.6-3 펌웨어 버전 이후의 DH 파라미터는 j2와 j3의 좌표계를 2도 오프셋하였습니다.

현재 기본 URDF는 후자입니다.

|firmware version |urdf|
|---|---|
|current version < S-V1.6-3|`piper_description_old.urdf`|
|current version >= S-V1.6-3|`piper_description.urdf`|

## 1 설치 방법

### 1.1 의존성 설치

주의: python-can 버전은 4.3.1 이상이어야 합니다.

```shell
pip3 install python-can
pip3 install scipy
```

```shell
pip3 install piper_sdk
```

```shell
sudo apt install ros-$ROS_DISTRO-ros2-control
sudo apt install ros-$ROS_DISTRO-ros2-controllers
sudo apt install ros-$ROS_DISTRO-controller-manager
```

## 2 CAN 모듈 사용

여기서 말하는 CAN 모듈은 로봇팔에 기본 제공되는 CAN 모듈만 지원하며, 다른 CAN 모듈은 지원하지 않습니다.

CAN 도구 설치

```shell
sudo apt update && sudo apt install can-utils ethtool
```

이 두 도구는 CAN 모듈을 설정하는 데 사용됩니다.

bash 스크립트를 실행할 때 `ip: command not found`가 나타나면 ip 명령을 설치하세요. 보통 `sudo apt-get install iproute2`로 설치합니다.

### 2.1 CAN 모듈 찾기

실행

```bash
bash find_all_can_port.sh
```

비밀번호를 입력한 후, CAN 모듈이 컴퓨터에 연결되어 있고 컴퓨터가 이를 감지하면 다음과 유사하게 출력됩니다:

```bash
Both ethtool and can-utils are installed.
Interface can0 is connected to USB port 3-1.4:1.0
```

여러 개가 있으면 다음과 유사하게 출력됩니다:

```bash
Both ethtool and can-utils are installed.
Interface can0 is connected to USB port 3-1.4:1.0
Interface can1 is connected to USB port 3-1.1:1.0
```

CAN 모듈의 개수만큼 `Interface can1 is connected to USB port 3-1.1:1.0`와 유사한 줄이 출력됩니다.

여기서 `can1`은 시스템이 찾은 CAN 모듈의 이름이고, `3-1.1:1.0`은 해당 CAN 모듈이 연결된 USB 포트입니다.

이전에 이미 CAN 모듈을 활성화하여 다른 이름을 가진 경우, 여기서는 이름이 `can_piper`라고 가정하면 다음과 같이 출력됩니다:

```bash
Both ethtool and can-utils are installed.
Interface can_piper is connected to USB port 3-1.4:1.0
Interface can0 is connected to USB port 3-1.1:1.0
```

CAN 모듈이 감지되지 않으면 다음만 출력됩니다:

```bash
Both ethtool and can-utils are installed.
```

### 2.2 단일 CAN 모듈 활성화, **여기서는 `can_activate.sh` 스크립트 사용**

단일 CAN 모듈을 활성화하는 경우는 두 가지로 나뉩니다. 하나는 컴퓨터에 CAN 모듈이 하나만 연결된 경우이고, 다른 하나는 여러 CAN 모듈이 컴퓨터에 연결되어 있지만 그중 하나만 활성화하는 경우입니다.

#### 2.2.1 PC에 USB-CAN 변환 모듈을 하나만 연결한 경우

바로 실행

```bash
bash can_activate.sh can0 1000000
```

여기서 `can0`은 임의의 이름으로 변경할 수 있으며, `1000000`은 보드레이트(baud rate)로 로봇팔을 사용하는 경우 변경할 수 없습니다.

#### 2.2.2 PC에 여러 USB-CAN 변환 모듈을 연결했지만, 매번 하나의 CAN 모듈만 활성화하는 경우

주의: 이 경우는 로봇팔과 섀시(chassis)를 동시에 사용하는 상황에 해당합니다.

(1) CAN 모듈이 연결된 USB 포트의 하드웨어 주소를 확인합니다. 모든 CAN 모듈을 뽑고, 로봇팔에 연결된 CAN 모듈만 PC에 연결한 후 실행

```shell
bash find_all_can_port.sh
```

그리고 `USB port` 값을 기록합니다. 예: `3-1.4:1.0`

(2) CAN 장치를 활성화합니다. 위의 `USB port` 값이 `3-1.4:1.0`이라고 가정하고 실행:

```bash
bash can_activate.sh can_piper 1000000 "3-1.4:1.0"
```

설명: **3-1.4:1.0 하드웨어로 인코딩된 USB 포트에 연결된 CAN 장치의 이름을 can_piper로 변경하고, 보드레이트를 1000000으로 설정하여 활성화합니다.**

(3) 활성화 성공 여부 확인

`ifconfig`를 실행하여 `can_piper`가 있는지 확인합니다. 있으면 CAN 모듈 설정이 성공한 것입니다.

### 2.3 여러 CAN 모듈 동시 활성화, **여기서는 `can_muti_activate.sh` 스크립트 사용**

먼저 컴퓨터에 몇 개의 정품 CAN 모듈이 연결되어 있는지 확인합니다. 여기서는 2개라고 가정합니다.

주의: **현재 컴퓨터에 5개의 CAN 모듈이 연결되어 있어도, 지정한 CAN 모듈만 활성화할 수 있습니다.**

#### 2.3.1 각 CAN 모듈에 대응하는 USB 포트 하드웨어 주소 기록

CAN 모듈을 하나씩 뽑았다 꽂으면서 각 모듈에 대응하는 USB 포트 하드웨어 주소를 하나씩 기록합니다.

`can_muti_activate.sh`에서 `USB_PORTS` 파라미터 내 요소의 개수는 활성화할 CAN 모듈의 개수이며, 여기서는 2로 가정합니다.

(1) CAN 모듈 중 하나를 PC에 단독으로 연결한 후 실행

```shell
bash find_all_can_port.sh
```

그리고 `USB port` 값을 기록합니다. 예: `3-1.4:1.0`

(2) 이어서 다음 CAN 모듈을 연결합니다. 주의: 이전 CAN 모듈을 연결했던 USB 포트와 **같으면 안 됩니다.** 그런 다음 실행

```shell
bash find_all_can_port.sh
```

두 번째 CAN 모듈의 `USB port` 값을 기록합니다. 예: `3-1.1:1.0`

주의: **이전에 활성화한 적이 없으면, 처음 연결한 CAN 모듈이 기본적으로 can0이 되고 두 번째가 can1이 됩니다. 이전에 활성화한 적이 있으면 이전에 활성화했던 이름이 됩니다.**

#### 2.3.2 USB 포트, 대상 인터페이스 이름 및 비트레이트 사전 정의

위 작업에서 기록한 `USB port` 값이 각각 `3-1.4:1.0`, `3-1.1:1.0`이라고 가정하면, 아래 `USB_PORTS["1-9:1.0"]="can_left:1000000"`의 대괄호 안 큰따옴표 내부 파라미터를 `3-1.4:1.0`과 `3-1.1:1.0`으로 교체합니다.

최종 결과는 다음과 같습니다:

```bash
USB_PORTS["3-1.4:1.0"]="can_left:1000000"
USB_PORTS["3-1.1:1.0"]="can_right:1000000"
```

설명: **3-1.4:1.0 하드웨어로 인코딩된 USB 포트에 연결된 CAN 장치의 이름을 can_left로 변경하고, 보드레이트를 1000000으로 설정하여 활성화합니다.**

#### 2.3.3 여러 CAN 모듈 활성화

`bash can_muti_activate.sh`를 실행합니다.

#### 2.3.4 여러 CAN 모듈이 정상 설정되었는지 확인

`ifconfig`를 실행하여 `can_left`와 `can_right`가 있는지 확인합니다.

## 3 노드 실행

빌드

```shell
cd piper_ros
colcon build
source install/setup.bash
```

### 3.1 단일 로봇팔 제어 시작

노드 이름 `piper_single_ctrl`

param

```shell
can_port:열려는 can 라우트 이름
auto_enable:자동 활성화(enable) 여부. True이면 프로그램 시작 시 자동으로 활성화됨
# 이 값을 False로 설정하면, 프로그램을 중단한 뒤 다시 노드를 시작해도 로봇팔은 지난번 프로그램 실행 시의 상태를 유지함
# 지난번 프로그램 실행 시 로봇팔 상태가 활성화(enable)였다면, 중단 후 다시 노드를 시작해도 로봇팔은 여전히 활성화 상태임
# 지난번 프로그램 실행 시 로봇팔 상태가 비활성화(disable)였다면, 중단 후 다시 노드를 시작해도 로봇팔은 여전히 비활성화 상태임
gripper_exist:엔드 이펙터 그리퍼(gripper) 유무. True이면 엔드 그리퍼가 있다는 의미로 그리퍼 제어가 켜짐
rviz_ctrl_flag:rviz를 사용하여 관절 각도 메시지를 전송할지 여부. True이면 rviz가 전송하는 관절 각도 메시지를 수신함
gripper_val_mutiple:그리퍼 제어 배수 설정
# rviz의 joint7 범위는 [0,0.04]인 반면 실제 그리퍼 행정(travel)은 0.08m이므로, rviz로 실제 그리퍼를 제어하려면 그리퍼를 2배로 설정해야 함
```

단일 팔을 시작하는 launch 파일은 `start_single_piper_rviz.launch.py`와 `start_single_piper.launch.py` 두 개가 있습니다.

전자는 rviz를 함께 시작할 수 있으며, 슬라이더를 드래그하여 로봇팔을 제어할 수 있습니다.

제어 노드를 시작합니다. 아래 방식 중 하나를 선택하여 실행하면 됩니다.

ros2 run 방식

```shell
# 노드 시작
ros2 run piper piper_single_ctrl --ros-args -p can_port:=can0 -p auto_enable:=false -p gripper_exist:=true -p gripper_val_mutiple:=2
# 로그 레벨 설정 --log-level DEBUG/INFO/WARN/ERROR
ros2 run piper piper_single_ctrl --ros-args -p can_port:=can0 -p auto_enable:=false -p gripper_exist:=true -p gripper_val_mutiple:=2 --log-level WARN
```

`ros2 topic list`

```bash
/arm_status #로봇팔 상태, 자세한 내용은 아래 참조
/enable_flag #활성화 플래그. 노드로 전송하며, true를 보내면 활성화됨
/end_pose #로봇팔 엔드 이펙터 위치/자세 상태 피드백
/end_pose_stamped #타임스탬프가 포함된 endpose 메시지
/joint_ctrl #로봇팔 제어 메시지 관련 피드백
/joint_ctrl_single #관절 메시지 구독. 이 메시지에 관절 위치를 보내면 로봇팔을 움직일 수 있음
/joint_states_single #로봇팔 관절 상태 피드백
/joint_states_feedback #위와 동일. topic 이름만 변경한 것
/pos_cmd #엔드 이펙터 제어 메시지. xyzrpy, 그리퍼 단위는 m, mode1과 mode2 파라미터는 기본값 0으로 두면 됨
```

ros2 launch 방식

launch에서 `/joint_ctrl_single` topic을 remap하여 `/joint_states`가 로봇팔을 제어하는 메시지가 되도록 함으로써, rviz와 연동하여 rviz가 실제 로봇팔을 제어할 수 있게 합니다.

```shell
# 노드 시작
ros2 run piper piper_single_ctrl --ros-args -p can_port:=can0 -p auto_enable:=false -p gripper_exist:=true -p gripper_val_mutiple:=2
# 로그 레벨 설정 --log-level DEBUG/INFO/WARN/ERROR
ros2 run piper piper_single_ctrl --ros-args -p can_port:=can0 -p auto_enable:=false -p gripper_exist:=true -p gripper_val_mutiple:=2 --log-level WARN
# launch 노드를 사용할 수도 있음
ros2 launch piper start_single_piper.launch.py can_port:=can0 auto_enable:=false gripper_exist:=false gripper_val_mutiple:=2
# log_level 설정 추가
ros2 launch piper start_single_piper.launch.py can_port:=can0 auto_enable:=false gripper_exist:=false gripper_val_mutiple:=2 log_level:=warn
# 또는, 기본 파라미터로 실행됨
ros2 launch piper start_single_piper.launch.py
# rviz로 제어를 시작할 수도 있음. 변경할 파라미터는 위와 같음
ros2 launch piper start_single_piper_rviz.launch.py
```

`ros2 topic list`

```shell
/arm_status #로봇팔 상태, 자세한 내용은 아래 참조
/enable_flag #활성화 플래그. 노드로 전송하며, true를 보내면 활성화됨
/end_pose #로봇팔 엔드 이펙터 위치/자세 상태 피드백
/end_pose_stamped #타임스탬프가 포함된 endpose 메시지
/joint_ctrl #로봇팔 제어 메시지 관련 피드백
/joint_states #관절 메시지 구독. 이 메시지에 관절 위치를 보내면 로봇팔을 움직일 수 있음
/joint_states_single #로봇팔 관절 상태 피드백
/joint_states_feedback #위와 동일. topic 이름만 변경한 것
/pos_cmd #엔드 이펙터 제어 메시지. xyzrpy, 그리퍼 단위는 m, mode1과 mode2 파라미터는 기본값 0으로 두면 됨
```

ros2 service list

```shell
/enable_srv #로봇팔 활성화 서비스 서버
```

로봇팔 활성화

```shell
# 서비스 서버 호출(call)
ros2 service call /enable_srv piper_msgs/srv/Enable enable_request:\ true\ 
# pub topic
ros2 topic pub /enable_flag std_msgs/msg/Bool data:\ true\ 
```

로봇팔 비활성화

```shell
# 서비스 서버 호출(call)
ros2 service call /enable_srv piper_msgs/srv/Enable enable_request:\ false\ 
# pub topic
ros2 topic pub /enable_flag std_msgs/msg/Bool data:\ false\ 
```

관절 메시지 발행(publish)

주의: 로봇팔이 들어 올려지므로, 로봇팔 작업 범위 내에 장애물이 없는지 확인하세요.

로봇팔은 기본 속도의 10%로 움직이며, 그리퍼 토크는 0.5N으로 설정됩니다.

속도 범위는 1~100으로 제한되며, 1보다 작으면 1로 간주됩니다.

위치는 다음과 같습니다.

```shell
ros2 topic pub /joint_states sensor_msgs/msg/JointState "{header: {stamp: {sec: 0, nanosec: 0}, frame_id: 'piper_single'}, name: ['joint1', 'joint2','joint3','joint4','joint5','joint6','joint7'], position: [0.2,0.2,-0.2,0.3,-0.2,0.5,0.01], velocity: [0,0,0,0,0,0,10], effort: [0,0,0,0,0,0,0.5]}"

```

### 3.2 단일 로봇팔에서 관절 메시지를 읽어 rviz에서 팔로우(follow) 표시

관절 데이터 읽기 노드 시작

```bash
# 그리퍼 있음
ros2 run piper piper_read_slave_joint --ros-args -p can_port:=can0 -p gripper_exist:=true
```

```bash
# 그리퍼 없음
ros2 run piper piper_read_slave_joint --ros-args -p can_port:=can0 -p gripper_exist:=false
```

실행 후에는 로봇팔 메시지 피드백용으로 `/joint_states` 하나만 존재합니다.

그중 그리퍼를 켜고 시작한 경우 메시지 내용에 `joint7`, `joint8` 데이터가 포함되는데, 이는 rviz에서 팔로우 표시 기능을 사용할 때 필요한 것입니다.

rviz 표시 팔로우 launch 시작

```bash
# 그리퍼 있음
ros2 launch piper_description  display_urdf_follow.launch.py
```

```bash
# 그리퍼 없음
ros2 launch piper_description  display_no_gripper_urdf_follow.launch.py
```

note:**rviz 표시만 시작하고 관절 데이터 읽기를 시작하지 않으면 /joint_states 데이터가 발행되지 않아, rviz 내 모델의 모든 link가 원점에 모여 정상적으로 표시되지 않습니다.**

실제 로봇팔 데이터 읽기 노드를 시작하고 싶지 않은 경우, rviz 표시 launch만 시작한 다음 다른 프로그램이나 터미널에서 `joint_states` topic을 발행할 수 있습니다. 예시는 다음과 같습니다.

```bash
# 그리퍼 있음
ros2 topic pub /joint_states sensor_msgs/msg/JointState "header:
  stamp:
    sec: 1757669668
    nanosec: 196177482
  frame_id: ''
name: ['joint1','joint2','joint3','joint4','joint5','joint6','gripper','joint7','joint8']
position: [0,0,0,0,0,0,0,0,0]
velocity: [0,0,0,0,0,0,0,0,0]
effort: [0,0,0,0,0,0,0,0,0]" 
```

```bash
# 그리퍼 없음
ros2 topic pub /joint_states sensor_msgs/msg/JointState "header:
  stamp:
    sec: 1757669668
    nanosec: 196177482
  frame_id: ''
name: ['joint1','joint2','joint3','joint4','joint5','joint6']
position: [0,0,0,0,0,0]
velocity: [0,0,0,0,0,0]
effort: [0,0,0,0,0,0]" 
```

주의: **header의 타임스탬프는 필수입니다. 그렇지 않으면 rviz에 표시되는 모델이 여전히 정상적으로 표시되지 않습니다.**

### 3.3 양팔 동시 제어

각 팔마다 CAN 모듈을 하나씩 연결한 다음, 두 개의 CAN 모듈을 활성화해야 합니다.

launch에서 읽어들이는 두 CAN 모듈 이름은 모두 `can0`이므로, 시스템에서의 좌/우 팔 이름으로 직접 수정해야 합니다. 확인 방법은 다음과 같습니다.

```bash
ifconfig -a | grep can
```

양팔 시작 실행

```bash
# 기본값은 두 노드 모두 can0 메시지를 읽도록 시작함
ros2 launch piper start_two_piper.launch.py auto_enable:=false
# 읽어들일 can 포트 이름을 사용자 지정
ros2 launch piper start_two_piper.launch.py can_left_port:=can_left can_right_port:=can_right auto_enable:=true girpper_exist:=true
```

- **launch 내부에서 topic의 remap을 수행합니다.**

```python
# piper_left_node
remappings=[
            # 제어
            ('pos_cmd', '/pos_cmd_left'),
            ('joint_ctrl_single', '/joint_ctrl_cmd_left'),
            # 피드백
            ('joint_states_single', '/joint_states_left'),
            ('joint_states_feedback', '/joint_left'),
            ('joint_ctrl', '/joint_states_ctrl_left'),
            ('arm_status', '/arm_status_left'),
            ('end_pose', '/end_pose_left'),
            ('end_pose_stamped', '/end_pose_stamped_left'),
        ]
# piper_right_node
remappings=[
            # 제어
            ('pos_cmd', '/pos_cmd_right'),
            ('joint_ctrl_single', '/joint_ctrl_cmd_right'),
            # 피드백
            ('joint_states_single', '/joint_states_right'),
            ('joint_states_feedback', '/joint_right'),
            ('joint_ctrl', '/joint_states_ctrl_right'),
            ('arm_status', '/arm_status_right'),
            ('end_pose', '/end_pose_right'),
            ('end_pose_stamped', '/end_pose_stamped_right'),
        ]
```

위의 단일 노드 topic 설명을 참조하여 remap 후 topic의 의미를 대조하세요.

```bash
/arm_status_left
/arm_status_right
/enable_flag
/end_pose_left
/end_pose_right
/end_pose_stamped_left
/end_pose_stamped_right
/joint_ctrl_cmd_left
/joint_ctrl_cmd_right
/joint_left
/joint_right
/joint_states_ctrl_left
/joint_states_ctrl_right
/joint_states_left
/joint_states_right
/pos_cmd_left
/pos_cmd_right
```

### 3.4 piper 사용자 정의 메시지

ros 패키지 `piper_msgs`

로봇팔 자체 상태 피드백 메시지로, CAN 프로토콜에서 `id=0x2A1`에 해당하는 피드백 메시지입니다.

`PiperStatusMsg.msg`

```c
uint8 ctrl_mode
/*
0x00 대기 모드
0x01 CAN 명령 제어 모드
0x02 교시(teaching) 모드
0x03 이더넷 제어 모드
0x04 wifi 제어 모드
0x05 리모컨 제어 모드
0x06 연동 교시 입력 모드
0x07 오프라인 궤적 모드*/
uint8 arm_status
/*
0x00 정상
0x01 비상 정지
0x02 해 없음(무해)
0x03 특이점(singularity)
0x04 목표 각도 리밋 초과
0x05 관절 통신 이상
0x06 관절 브레이크 미해제
0x07 로봇팔 충돌 발생
0x08 드래그 교시 중 과속
0x09 관절 상태 이상
0x0A 기타 이상
0x0B 교시 기록
0x0C 교시 실행
0x0D 교시 일시정지
0x0E 메인 컨트롤러 NTC 과열
0x0F 방전 저항 NTC 과열*/
uint8 mode_feedback
/*
0x00 MOVE P
0x01 MOVE J
0x02 MOVE L
0x03 MOVE C*/
uint8 teach_status
/*
0x00 종료
0x01 교시 기록 시작(드래그 교시 모드 진입)
0x02 교시 기록 종료(드래그 교시 모드 종료)
0x03 교시 궤적 실행(드래그 교시 궤적 재현)
0x04 실행 일시정지
0x05 실행 계속(궤적 재현 계속)
0x06 실행 중단
0x07 궤적 시작점으로 이동*/
uint8 motion_status
/*
0x00 지정 위치 도달
0x01 지정 위치 미도달*/
uint8 trajectory_num
/*0~255 (오프라인 궤적 모드에서 피드백)*/
int64 err_code//고장 코드
bool joint_1_angle_limit//1번 관절 통신 이상(0: 정상 1: 이상)
bool joint_2_angle_limit//2번 관절 통신 이상(0: 정상 1: 이상)
bool joint_3_angle_limit//3번 관절 통신 이상(0: 정상 1: 이상)
bool joint_4_angle_limit//4번 관절 통신 이상(0: 정상 1: 이상)
bool joint_5_angle_limit//5번 관절 통신 이상(0: 정상 1: 이상)
bool joint_6_angle_limit//6번 관절 통신 이상(0: 정상 1: 이상)
bool communication_status_joint_1//1번 관절 각도 리밋 초과(0: 정상 1: 이상)
bool communication_status_joint_2//2번 관절 각도 리밋 초과(0: 정상 1: 이상)
bool communication_status_joint_3//3번 관절 각도 리밋 초과(0: 정상 1: 이상)
bool communication_status_joint_4//4번 관절 각도 리밋 초과(0: 정상 1: 이상)
bool communication_status_joint_5//5번 관절 각도 리밋 초과(0: 정상 1: 이상)
bool communication_status_joint_6//6번 관절 각도 리밋 초과(0: 정상 1: 이상)
```

로봇팔 엔드 이펙터 위치/자세 제어. 주의: 일부 특이점은 도달할 수 없습니다.

`PosCmd.msg`

```c
float64 x
float64 y
float64 z
float64 roll
float64 pitch
float64 yaw
float64 gripper
int32 mode1
int32 mode2
```

## 주의사항

- 로봇팔 메시지를 읽거나 로봇팔을 제어하려면 먼저 CAN 장치를 활성화하고 올바른 보드레이트를 설정해야 합니다.
- 다음이 나타나는 경우

  ```shell
  사용 가능 상태(enable): False
  <class 'can.exceptions.CanError'> Message NOT sent
  <class 'can.exceptions.CanError'> Message NOT sent
  ```

  로봇팔이 CAN 모듈과 연결되지 않았다는 의미입니다. USB를 뽑았다 꽂은 후 로봇팔을 재시작하고, CAN 모듈을 다시 활성화한 다음 노드를 재시작해 보세요.

- 자동 활성화를 켠 경우, 5초 동안 활성화를 시도해도 성공하지 못하면 프로그램이 자동으로 종료됩니다.

## Q&A

1. 다음이 나타나는 경우

```shell
colcon build
Starting >>> piper   
Starting >>> piper_control
Starting >>> piper_description
Starting >>> piper_msgs
Finished <<< piper [0.31s]                                                                                                              
--- stderr: piper_description                                                                                            
CMake Error at CMakeLists.txt:5 (find_package):
  By not providing "Findament_cmake.cmake" in CMAKE_MODULE_PATH this project
  has asked CMake to find a package configuration file provided by
  "ament_cmake", but CMake did not find one.

  Could not find a package configuration file provided by "ament_cmake" with
  any of the following names:

    ament_cmakeConfig.cmake
    ament_cmake-config.cmake

  Add the installation prefix of "ament_cmake" to CMAKE_PREFIX_PATH or set
  "ament_cmake_DIR" to a directory containing one of the above files.  If
  "ament_cmake" provides a separate development package or SDK, be sure it
  has been installed.


---
Failed   <<< piper_description [0.45s, exited with code 1]
Aborted  <<< piper_control [0.45s]
Aborted  <<< piper_msgs [0.45s]

Summary: 1 package finished [0.55s]
  1 package failed: piper_description
  2 packages aborted: piper_control piper_msgs
  3 packages had stderr output: piper_control piper_description piper_msgs
```

```shell
source /opt/ros/$ROS_DISTRO/setup.bash
```

2. 

```shell
Starting >>> piper   
Starting >>> piper_control
Starting >>> piper_description
Starting >>> piper_msgs
Finished <<< piper [0.33s]                                                                                                              
Finished <<< piper_description [0.42s]                                                                                     
--- stderr: piper_control                                                               
CMake Error at CMakeLists.txt:7 (find_package):
  By not providing "Findcontroller_manager.cmake" in CMAKE_MODULE_PATH this
  project has asked CMake to find a package configuration file provided by
  "controller_manager", but CMake did not find one.

  Could not find a package configuration file provided by
  "controller_manager" with any of the following names:

    controller_managerConfig.cmake
    controller_manager-config.cmake

  Add the installation prefix of "controller_manager" to CMAKE_PREFIX_PATH or
  set "controller_manager_DIR" to a directory containing one of the above
  files.  If "controller_manager" provides a separate development package or
  SDK, be sure it has been installed.


---
Failed   <<< piper_control [0.80s, exited with code 1]
Aborted  <<< piper_msgs [3.28s]                               

Summary: 2 packages finished [3.38s]
  1 package failed: piper_control
  1 package aborted: piper_msgs
  1 package had stderr output: piper_control
```

```shell
sudo apt install ros-$ROS_DISTRO-ros2-control
sudo apt install ros-$ROS_DISTRO-ros2-controllers
sudo apt install ros-$ROS_DISTRO-controller-manager
```

3. 

다음이 나타나는 경우:

```shell
 "package 'joint_state_publisher_gui' not found
```

설치:

```shell
sudo apt install ros-$ROS_DISTRO-joint-state-publisher-gui
```

4. 

```shell
sudo apt install ros-$ROS_DISTRO-robot-state-publisher
```

5. 

```shell
sudo apt install ros-$ROS_DISTRO-xacro
```

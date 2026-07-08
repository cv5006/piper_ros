# Piper_Moveit2

[EN](README(EN).md)

![ubuntu](https://img.shields.io/badge/Ubuntu-22.04-orange.svg)

|PYTHON |STATE|
|---|---|
|![humble](https://img.shields.io/badge/ros-humble-blue.svg)|![Pass](https://img.shields.io/badge/Pass-blue.svg)|

> 주의: 설치 및 사용 과정에서 문제가 발생하면 5장을 참고하세요.

## 1 Moveit2 설치

1) 바이너리 설치, [참고 링크](https://moveit.ai/install-moveit2/binary/)

```bash
sudo apt install ros-humble-moveit*
```

2) 소스 컴파일 방법, [참고 링크](https://moveit.ai/install-moveit2/source/)

## 2 사용 환경

Moveit2 설치 후 몇 가지 의존성을 설치해야 합니다.

```bash
sudo apt-get install ros-humble-control* ros-humble-joint-trajectory-controller ros-humble-joint-state-* ros-humble-gripper-controllers ros-humble-trajectory-msgs
```

시스템 언어 로케일이 영어 로케일이 아닌 경우 다음을 설정해야 합니다.

```bash
echo "export LC_NUMERIC=en_US.UTF-8" >> ~/.bashrc
source ~/.bashrc
```

## 3 moveit으로 실제 로봇팔 제어

### 3.1 piper_ros 실행

[piper_ros](../../README.MD#1-안내-방법) 설정을 완료한 후

```bash
cd ~/piper_ros
source install/setup.bash
bash can_activate.sh can0 1000000
```

제어 노드 실행

```bash
ros2 launch piper start_single_piper.launch.py gripper_val_mutiple:=2
```

### 3.2 moveit2 제어

moveit2 실행

```bash
cd ~/piper_ros
conda deactivate # conda 환경이 없으면 이 줄은 제거하세요
source install/setup.bash
```

#### 3.2.1 그리퍼 없이 실행

```bash
ros2 launch piper_no_gripper_moveit demo.launch.py
```

#### 3.2.2 그리퍼 있게 실행

```bash
ros2 launch piper_with_gripper_moveit demo.launch.py
```

![piper_moveit](../../asserts/pictures/piper_moveit.png)

로봇팔 말단의 화살표를 직접 드래그하여 로봇팔을 제어할 수 있습니다.

위치를 조정한 후 왼쪽 MotionPlanning의 Planning에서 Plan&Execute를 클릭하면 계획 및 동작이 시작됩니다.

## 4 moveit으로 시뮬레이션 로봇팔 제어

### 4.1 gazebo

#### 4.1.1 gazebo 실행

[piper_gazebo](../piper_sim/README.md#1-gazebo-시뮬레이션) 참고

#### 5.1.2 moveit 제어

```bash
cd ~/piper_ros
source install/setup.bash
```

주의: **아래의 launch는 실제 로봇팔을 제어하는 demo.launch.py가 아니며, gazebo를 실행한 뒤에 실행해야 합니다. 그렇지 않으면 로봇팔 모델이 나타나지 않습니다.**

그리퍼 있게 실행

```bash
ros2 launch piper_with_gripper_moveit piper_moveit.launch.py
```

그리퍼 없이 실행

```bash
ros2 launch piper_no_gripper_moveit piper_moveit.launch.py
```

### 5.2 mujoco

#### 5.2.1 moveit 제어(먼저 moveit 실행)

[3.2 moveit2 제어](#32-moveit2-제어)와 동일

#### 5.2.2 mujoco 실행

[piper_mujoco](../piper_sim/README.md#2-mujoco-시뮬레이션) 참고

주의: **종료하려면 ctrl+C+\\를 사용하면 됩니다.**

## 5 발생할 수 있는 문제

### 5.1 gazebo를 열 때 urdf가 로드되지 않았다는 오류가 발생하여, 시뮬레이션 환경에서 로봇팔 말단과 베이스가 겹쳐 보이는 경우

1 컴파일 후 install 아래의 piper_description에 config가 있는지, 그리고 그 config에 src/piper/piper_description의 config 파일이 포함되어 있는지 확인하세요.

install에 urdf가 없는 경우도 마찬가지입니다.

2 src/piper/piper_description/urdf/piper_description_gazebo.xacro의 644번째 줄 경로가 올바른지 확인하세요. 확인 후에도 문제가 계속되면 경로를 절대 경로로 변경하세요.

### 5.2 demo.launch.py 실행 시 오류 발생

오류: 파라미터에 double이 필요한데 string이 제공됨
해결 방법:
터미널에서 실행

```bash
echo "export LC_NUMERIC=en_US.UTF-8" >> ~/.bashrc
source ~/.bashrc
```

또는 launch를 실행하기 전에 LC_NUMERIC=en_US.UTF-8을 붙입니다.
예를 들어

```bash
LC_NUMERIC=en_US.UTF-8 ros2 launch piper_moveit_config demo.launch.py
```

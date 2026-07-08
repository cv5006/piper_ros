# piper_sim

[EN](README(EN).md)

![ubuntu](https://img.shields.io/badge/Ubuntu-22.04-orange.svg)

|ROS |STATE|
|---|---|
|![ros](https://img.shields.io/badge/ROS-humble-blue.svg)|![Pass](https://img.shields.io/badge/Pass-blue.svg)|

## 1 gazebo 시뮬레이션

### 0 환경 설정

```bash
sudo apt update
sudo apt install gazebo ros-humble-gazebo-ros-pkgs ros-humble-gazebo-ros2-control ros-humble-ros2-control ros-humble-ros2-controllers
```

### 1.1 piper gazebo 시뮬레이션(그리퍼 있음)

gazebo 시뮬레이션 실행

```bash
cd piper_ros
source install/setup.bash
```

```bash
ros2 launch piper_gazebo piper_gazebo.launch.py
```

### 1.2 piper gazebo 시뮬레이션(그리퍼 없음)

```bash
ros2 launch piper_gazebo piper_no_gripper_gazebo.launch.py
```

주의: **moveit으로 제어할 경우 먼저 gazebo를 실행한 뒤 moveit을 실행해야 하며, demo.launch.py가 아닌 piper_moveit.launch.py를 사용해야 합니다.**

## 2 mujoco 시뮬레이션

### 2.1 mujoco210과 mujoco-py 설치

#### 2.1.1 mujoco 설치

1、[mujoco210 다운로드](https://github.com/google-deepmind/mujoco/releases/download/2.1.0/mujoco210-linux-x86_64.tar.gz)

2、압축 해제

```bash
mkdir ~/.mujoco
cd (압축 파일이 있는 디렉터리)
tar -zxvf mujoco210-linux-x86_64.tar.gz -C ~/.mujoco
```

3、환경 변수 추가

```bash
echo "export LD_LIBRARY_PATH=~/.mujoco/mujoco210/bin:\$LD_LIBRARY_PATH" >> ~/.bashrc
source ~/.bashrc
```

4、테스트

```bash
cd ~/.mujoco/mujoco210/bin
./simulate ../model/humanoid.xml
```

#### 2.1.2 mujoco-py 설치

1、소스 코드 다운로드

```bash
git clone https://github.com/openai/mujoco-py.git
```

2、설치(이 단계는 conda 환경에서 진행할 수 있습니다)

```bash
cd mujoco-py
pip3 install -U 'mujoco-py<2.2,>=2.1'
pip3 install -r requirements.txt
pip3 install -r requirements.dev.txt
python3 setup.py install
sudo apt install libosmesa6-dev
sudo apt install patchelf
```

3、환경 변수 추가

```bash
echo "export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/lib/nvidia" >> ~/.bashrc
source ~/.bashrc
```

4、테스트

주의: **mujoco_py를 임포트할 때 오류가 발생하는 경우가 있는데, 오류 메시지의 안내에 따라 numpy, cmake 등의 버전을 업데이트하면 됩니다.**

python 실행

```python
import mujoco_py
import os
mj_path = mujoco_py.utils.discover_mujoco()
xml_path = os.path.join(mj_path, 'model', 'humanoid.xml')
model = mujoco_py.load_model_from_path(xml_path)
sim = mujoco_py.MjSim(model)
print(sim.data.qpos)
sim.step()
print(sim.data.qpos)
```

### 2.2 piper mujoco 시뮬레이션(그리퍼 있음)

mujoco 시뮬레이션 실행

```bash
cd piper_ros
source install/setup.bash
```

```bash
ros2 run piper_mujoco piper_mujoco_ctrl.py
```

주의: **mujoco 로봇팔이 제어되지 않으면 mujoco를 다시 실행하세요.**

rviz_gui로 그리퍼 있는 로봇팔 제어(새 터미널에서 실행)

```bash
cd piper_ros
source install/setup.bash
```

```bash
ros2 launch piper_description display_urdf.launch.py
```

#### 2.3 piper mujoco 시뮬레이션(그리퍼 없음)

mujoco 시뮬레이션 실행

```bash
cd piper_ros
source install/setup.bash
```

```bash
ros2 run piper_mujoco piper_no_gripper_mujoco_ctrl.py
```

rviz_gui로 그리퍼 없는 로봇팔 제어(새 터미널에서 실행)

```bash
cd piper_ros
source install/setup.bash
```

```bash
ros2 launch piper_description display_no_gripper_urdf.launch.py
```

주의: **제어가 되지 않으면 rviz_gui를 실행한 후에 mujoco를 실행하세요.**

#### 제어 파라미터 소개

[그리퍼 있는 버전 제어 파라미터](../piper_description/mujoco_model/piper_description.xml)

[그리퍼 없는 버전 제어 파라미터](../piper_description/mujoco_model/piper_no_gripper_description.xml)

- damping="100 관절 댐핑(damping) 변경

- kp="10000" 관절 제어 게인(gain) 변경

- forcerange="-100 100" 관절 제어 토크(torque) 변경

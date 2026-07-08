#!/usr/bin/env python3
# coding=utf-8

import sys
print(sys.executable)

import rclpy
from rclpy.node import Node
import mujoco_py
import os
import time
import glfw
from mujoco_py import MjSim, MjViewer
from mujoco_py import GlfwContext
from sensor_msgs.msg import JointState
from ament_index_python.packages import get_package_share_directory

class MujocoModel(Node):
    def __init__(self):
        super().__init__("mujoco_joint_controller")
        self.create_subscription(JointState, "/joint_states", self.joint_state_callback, 10)

        # joint_targets 딕셔너리 초기화
        self.joint_targets = {}

        pkg_share_dir = get_package_share_directory('piper_description')
        model_path = os.path.join(pkg_share_dir, 'mujoco_model', 'piper_description.xml')
        model_path = os.path.abspath(model_path)

        self.get_logger().info(f"The model path is: {model_path}")

        model = mujoco_py.load_model_from_path(model_path)
        self.sim = MjSim(model)
        self.viewer = MjViewer(self.sim)

        self.timer = self.create_timer(0.01, self.control_loop)  # 100Hz 제어 루프
        self.tolerance = 0.05  # 각도 오차 허용치

    def joint_state_callback(self, msg):
        """ ROS 2 /joint_states 토픽에서 관절 각도 가져오기 """
        for i, name in enumerate(msg.name):
            self.joint_targets[name] = msg.position[i]

        # joint8이 joint7의 음수 값이 되도록 보장
        if "joint7" in self.joint_targets:
            self.joint_targets["joint8"] = -self.joint_targets["joint7"]

    def pos_ctrl(self, joint_name, target_angle):
        """ MuJoCo 관절 각도 제어 """
        if joint_name not in self.sim.model.joint_names:
            self.get_logger().warn(f"Joint {joint_name} not found in Mujoco model.")
            return
        try:
            joint_id = self.sim.model.get_joint_qpos_addr(joint_name)
            actuator_id = self.sim.model.actuator_name2id(joint_name)
            self.sim.data.ctrl[actuator_id] = target_angle
        except Exception as e:
            self.get_logger().error(f"Error controlling joint {joint_name}: {e}")

    def control_loop(self):
        """ MuJoCo 로봇 팔이 ROS 관절 상태를 따르도록 함 """
        all_reached = True
        for joint, target_angle in self.joint_targets.items():
            if joint in self.sim.model.joint_names:
                joint_id = self.sim.model.get_joint_qpos_addr(joint)
                current_angle = self.sim.data.qpos[joint_id]
                if abs(current_angle - target_angle) > self.tolerance:
                    all_reached = False
                self.pos_ctrl(joint, target_angle)
        self.sim.step()
        self.viewer.render()


def main():
    rclpy.init()
    mujoco_node = MujocoModel()
    # mujoco_node.declare_parameter('use_sim_time', True)
    rclpy.spin(mujoco_node)
    mujoco_node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":  
    main()
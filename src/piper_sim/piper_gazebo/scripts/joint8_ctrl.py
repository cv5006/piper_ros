#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from control_msgs.msg import JointTrajectoryControllerState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

class GripperMirrorController(Node):
    def __init__(self):
        super().__init__('gripper_mirror_controller')

        # joint7 상태 구독
        self.subscription = self.create_subscription(
            JointTrajectoryControllerState,
            '/gripper_controller/controller_state',
            self.joint_state_callback,
            10
        )

        # joint8 제어 명령 발행
        self.publisher = self.create_publisher(
            JointTrajectory,
            '/gripper8_controller/joint_trajectory',
            10
        )

        # 타이머, 초당 발행 빈도 제어
        self.timer = self.create_timer(0.02, self.publish_joint8_command)

        self.joint7_position = None  # joint7 위치 저장용

    def joint_state_callback(self, msg):
        try:
            # joint7 인덱스 찾기
            joint_index = msg.joint_names.index("joint7")
            self.joint7_position = msg.reference.positions[joint_index]

        except ValueError:
            self.get_logger().warn("joint7 not found in /gripper_controller/state")

    def publish_joint8_command(self):
        if self.joint7_position is not None:
            # 반대 값 계산
            joint8_position = -self.joint7_position

            # JointTrajectory 메시지 생성
            traj_msg = JointTrajectory()
            traj_msg.joint_names = ["joint8"]

            # 궤적 지점 설정
            point = JointTrajectoryPoint()
            point.positions = [joint8_position]

            traj_msg.points.append(point)

            # gripper8_controller로 발행
            self.publisher.publish(traj_msg)

def main(args=None):
    rclpy.init(args=args)
    node = GripperMirrorController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()

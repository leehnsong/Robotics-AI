import os
import re
import time
import tempfile

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSHistoryPolicy

from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import Point, Pose
from gazebo_msgs.srv import SetEntityState
from gazebo_msgs.msg import EntityState

from vla_interfaces.action import ExecuteAction

# 손(panda_hand) 프레임 → 손가락 끝(TCP) 대략 거리 [m]
TCP_OFFSET = 0.10
# 접근/파지/들어올리기 높이(물체 윗면 기준) [m]
APPROACH_CLEARANCE = 0.12
LIFT_HEIGHT = 0.22
# home 자세(관절 1~7)
HOME = [0.0, -0.4, 0.0, -1.8, 0.0, 1.5, 0.7]


class VLAControlActionServer(Node):
    def __init__(self):
        super().__init__('vla_control_action_server')

        self.arm_pub = self.create_publisher(
            JointTrajectory,
            '/arm_controller/joint_trajectory',
            10
        )

        self.gripper_pub = self.create_publisher(
            JointTrajectory,
            '/gripper_controller/joint_trajectory',
            10
        )

        self.set_entity_client = self.create_client(
            SetEntityState,
            '/gazebo/set_entity_state'
        )

        self.arm_joints = [
            'panda_joint1',
            'panda_joint2',
            'panda_joint3',
            'panda_joint4',
            'panda_joint5',
            'panda_joint6',
            'panda_joint7',
        ]

        self.gripper_joints = [
            'panda_finger_joint1',
            'panda_finger_joint2',
        ]

        self.held_object = None

        # scene.world 기준 기본 좌표(perception이 position을 안 줄 때만 사용)
        self.default_positions = {
            'red_block': Point(x=0.45, y=0.15, z=0.43),
            'blue_plate': Point(x=0.55, y=-0.20, z=0.41),
            'place_area': Point(x=0.55, y=-0.20, z=0.41),
        }

        # ── IK 체인 준비 (robot_description → ikpy) ──────────
        self.chain = None
        self._ik_guess = None
        self._setup_ik()

        self.action_server = ActionServer(
            self,
            ExecuteAction,
            '/execute_action',
            self.execute_callback
        )

        self.get_logger().info('vla_control_action_server ready')
        self.get_logger().info('Action server: /execute_action')

    # ── IK ────────────────────────────────────────
    def _setup_ik(self):
        """/robot_description(latched)에서 URDF를 받아 ikpy 체인 구성.
        panda_link0 → panda_hand, 관절 1~7만 active. 실패해도 노드는 살아있되
        IK 없이 home 자세만 가능(move_to_xyz에서 경고)."""
        urdf = self._wait_robot_description(timeout=15.0)
        if not urdf:
            self.get_logger().error('robot_description 수신 실패 → IK 비활성(프리셋 fallback)')
            return
        try:
            from ikpy.chain import Chain
            # 손가락 링크/조인트 제거(체인이 가지치기 없이 panda_hand에서 끝나도록)
            for nm in ['panda_finger_joint1', 'panda_finger_joint2',
                       'panda_leftfinger', 'panda_rightfinger']:
                urdf = re.sub(r'<joint name="%s".*?</joint>' % nm, '', urdf, flags=re.S)
                urdf = re.sub(r'<link name="%s".*?</link>' % nm, '', urdf, flags=re.S)
            fd, path = tempfile.mkstemp(suffix='.urdf')
            with os.fdopen(fd, 'w') as f:
                f.write(urdf)
            chain = Chain.from_urdf_file(path, base_elements=['panda_link0'])
            # 관절 1~7만 active(나머지 fixed)
            mask = [False] + [True] * 7 + [False] * (len(chain.links) - 8)
            chain.active_links_mask = mask
            self.chain = chain
            n = len(chain.links)
            self._ik_guess = np.zeros(n)
            self._ik_guess[1:8] = HOME
            self.get_logger().info(f'IK 체인 구성 완료 (links={n}, active joints=7)')
        except Exception as e:
            self.get_logger().error(f'IK 체인 구성 실패: {e} → 프리셋 fallback')
            self.chain = None

    def _wait_robot_description(self, timeout):
        qos = QoSProfile(depth=1)
        qos.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
        qos.history = QoSHistoryPolicy.KEEP_LAST
        box = {'urdf': None}

        def cb(msg):
            box['urdf'] = msg.data

        sub = self.create_subscription(String, '/robot_description', cb, qos)
        t0 = time.time()
        while box['urdf'] is None and time.time() - t0 < timeout:
            rclpy.spin_once(self, timeout_sec=0.2)
        self.destroy_subscription(sub)
        return box['urdf']

    def solve_ik(self, x, y, z):
        """손(panda_hand)을 (x,y,z)에, top-down(손 Z축 ↓) 자세로. 관절 1~7 리스트 반환."""
        R = np.array([[1, 0, 0], [0, -1, 0], [0, 0, -1]])  # tool Z → world -Z
        q = self.chain.inverse_kinematics(
            target_position=[x, y, z],
            target_orientation=R,
            orientation_mode='all',
            initial_position=self._ik_guess,
        )
        self._ik_guess = q  # 다음 해의 초기값(연속성)
        return list(q[1:8])

    def move_to_xyz(self, x, y, z_obj, clearance=0.0, sec=2.0):
        """물체 좌표(z_obj=물체 중심/윗면) 기준으로 손가락 끝이 그 위 clearance 에 오도록 이동."""
        if self.chain is None:
            self.get_logger().warn('IK 없음 → 이동 생략(home 유지)')
            return
        hand_z = z_obj + TCP_OFFSET + clearance
        try:
            joints = self.solve_ik(x, y, hand_z)
        except Exception as e:
            self.get_logger().error(f'IK 실패({x:.2f},{y:.2f},{hand_z:.2f}): {e}')
            return
        self.get_logger().info(
            f'Move IK → ({x:.2f}, {y:.2f}, {z_obj:.2f})+clr{clearance:.2f}')
        self.send_arm(joints, sec)

    def execute_callback(self, goal_handle):
        action = goal_handle.request.action
        target = goal_handle.request.target
        position = goal_handle.request.position

        self.feedback(goal_handle, f'Received: action={action}, target={target}')

        result = ExecuteAction.Result()

        try:
            if action == 'move_home':
                self.move_home(goal_handle)

            elif action == 'open_gripper':
                self.open_gripper(goal_handle)

            elif action == 'close_gripper':
                self.close_gripper(goal_handle)

            elif action == 'pick':
                self.pick(target, position, goal_handle)

            elif action == 'place':
                self.place(target, position, goal_handle)

            else:
                result.success = False
                result.message = f'Unknown action: {action}'
                goal_handle.abort()
                return result

            result.success = True
            result.message = f'{action} completed'
            goal_handle.succeed()
            return result

        except Exception as e:
            result.success = False
            result.message = f'Failed: {str(e)}'
            self.get_logger().error(result.message)
            goal_handle.abort()
            return result

    def feedback(self, goal_handle, msg):
        fb = ExecuteAction.Feedback()
        fb.status = msg
        goal_handle.publish_feedback(fb)
        self.get_logger().info(msg)

    def pick(self, target, position, goal_handle):
        self.feedback(goal_handle, f'Pick start: {target}')

        p = self.resolve_position(target, position)

        self.open_gripper(goal_handle)
        self.move_to_xyz(p.x, p.y, p.z, clearance=APPROACH_CLEARANCE)  # 접근
        self.move_to_xyz(p.x, p.y, p.z, clearance=0.0)                 # 파지 위치
        self.close_gripper(goal_handle)

        self.held_object = target

        # simulated grasp: 잡힌 것처럼 물체를 손과 함께 올림
        self.set_object_pose(target, p.x, p.y, p.z + LIFT_HEIGHT)
        self.move_to_xyz(p.x, p.y, p.z, clearance=LIFT_HEIGHT)         # 들어올리기

        self.feedback(goal_handle, f'Pick done: {target}')

    def place(self, target, position, goal_handle):
        self.feedback(goal_handle, f'Place start: {target}')

        p = self.resolve_position(target, position)

        self.move_to_xyz(p.x, p.y, p.z, clearance=LIFT_HEIGHT)        # 위로 접근
        if self.held_object:
            self.set_object_pose(self.held_object, p.x, p.y, p.z + LIFT_HEIGHT)

        self.move_to_xyz(p.x, p.y, p.z, clearance=0.05)              # 내려놓기

        if self.held_object:
            self.set_object_pose(self.held_object, p.x, p.y, p.z)

        self.open_gripper(goal_handle)
        self.held_object = None
        self.move_to_xyz(p.x, p.y, p.z, clearance=APPROACH_CLEARANCE)  # 후퇴
        self.move_home(goal_handle)

        self.feedback(goal_handle, f'Place done: {target}')

    def open_gripper(self, goal_handle=None):
        if goal_handle:
            self.feedback(goal_handle, 'Open gripper')
        self.send_gripper([0.04, 0.04], 1.0)

    def close_gripper(self, goal_handle=None):
        if goal_handle:
            self.feedback(goal_handle, 'Close gripper')
        self.send_gripper([0.00, 0.00], 1.0)

    def move_home(self, goal_handle=None):
        if goal_handle:
            self.feedback(goal_handle, 'Move home')
        self.send_arm(HOME, 2.0)
        if self.chain is not None:
            self._ik_guess[1:8] = HOME  # 초기 추정값 리셋

    def send_arm(self, positions, sec):
        msg = JointTrajectory()
        msg.joint_names = self.arm_joints

        point = JointTrajectoryPoint()
        point.positions = [float(v) for v in positions]
        point.time_from_start.sec = int(sec)

        msg.points.append(point)
        self.arm_pub.publish(msg)

        self.sleep(sec + 0.5)

    def send_gripper(self, positions, sec):
        msg = JointTrajectory()
        msg.joint_names = self.gripper_joints

        point = JointTrajectoryPoint()
        point.positions = positions
        point.time_from_start.sec = int(sec)

        msg.points.append(point)
        self.gripper_pub.publish(msg)

        self.sleep(sec + 0.3)

    def set_object_pose(self, name, x, y, z):
        # wait_for_service 는 execute_callback(=비spin 구간)에서 그래프 캐시를 못 갱신해
        # 서비스가 있어도 False 를 주므로 게이트하지 않고 바로 비동기 전송한다.
        req = SetEntityState.Request()

        state = EntityState()
        state.name = name
        state.reference_frame = 'world'

        pose = Pose()
        pose.position.x = float(x)
        pose.position.y = float(y)
        pose.position.z = float(z)
        pose.orientation.w = 1.0

        state.pose = pose
        req.state = state

        # 주의: execute_callback 안에서 spin 하면 다른 goal 콜백이 재진입(인터리빙)되므로
        # 응답을 기다리지 않고 비동기 전송만 한다(텔레포트는 best-effort).
        self.set_entity_client.call_async(req)
        time.sleep(0.3)

        self.get_logger().info(f'Set object pose: {name} -> {x}, {y}, {z}')

    def resolve_position(self, target, position):
        # orchestrator/perception이 position을 주면 그 좌표 우선 사용
        if abs(position.x) > 0.001 or abs(position.y) > 0.001 or abs(position.z) > 0.001:
            return position

        # position이 없으면 데모용 기본 좌표 사용
        return self.default_positions.get(
            target,
            Point(x=0.45, y=0.0, z=0.43)
        )

    def sleep(self, sec):
        # 단순 블로킹 sleep. execute_callback 안에서 spin_once 로 재진입하면
        # pick/place goal 이 인터리빙되어 팔이 두 목표를 오가므로 spin 하지 않는다.
        time.sleep(sec)


def main():
    rclpy.init()
    node = VLAControlActionServer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

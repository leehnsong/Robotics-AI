import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer

from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import Point, Pose
from gazebo_msgs.srv import SetEntityState
from gazebo_msgs.msg import EntityState

from vla_interfaces.action import ExecuteAction


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

        self.default_positions = {
            'red_block': Point(x=0.55, y=0.00, z=0.78),
            'blue_plate': Point(x=0.25, y=-0.35, z=0.78),
            'place_area': Point(x=0.25, y=-0.35, z=0.78),
        }

        self.action_server = ActionServer(
            self,
            ExecuteAction,
            '/execute_action',
            self.execute_callback
        )

        self.get_logger().info('vla_control_action_server ready')
        self.get_logger().info('Action server: /execute_action')

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
        self.move_to_pose('pre_grasp')
        self.move_to_pose('grasp')
        self.close_gripper(goal_handle)

        self.held_object = target

        # simulated grasp: 잡힌 것처럼 물체를 위로 올림
        self.set_object_pose(target, p.x, p.y, p.z + 0.25)

        self.move_to_pose('lift')

        self.feedback(goal_handle, f'Pick done: {target}')

    def place(self, target, position, goal_handle):
        self.feedback(goal_handle, f'Place start: {target}')

        p = self.resolve_position(target, position)

        self.move_to_pose('pre_place')

        if self.held_object:
            self.set_object_pose(self.held_object, p.x, p.y, p.z + 0.25)

        self.move_to_pose('place')

        if self.held_object:
            self.set_object_pose(self.held_object, p.x, p.y, p.z)

        self.open_gripper(goal_handle)
        self.held_object = None
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
        self.send_arm([0.0, -0.4, 0.0, -1.8, 0.0, 1.5, 0.7], 2.0)

    def move_to_pose(self, label):
        # MoveIt/pymoveit2 대체부
        # 추후 이 함수 내부만 pymoveit2 pose goal로 교체 가능

        presets = {
            'pre_grasp': [0.0, 0.1, 0.0, -1.7, 0.0, 1.8, 0.8],
            'grasp': [0.0, 0.25, 0.0, -1.95, 0.0, 2.1, 0.8],
            'lift': [0.0, -0.25, 0.0, -1.7, 0.0, 1.9, 0.8],
            'pre_place': [-0.65, -0.25, 0.0, -1.7, 0.0, 1.9, 0.8],
            'place': [-0.65, 0.15, 0.0, -1.9, 0.0, 2.1, 0.8],
        }

        self.get_logger().info(f'Move to pose demo: {label}')
        self.send_arm(presets[label], 2.0)

    def send_arm(self, positions, sec):
        msg = JointTrajectory()
        msg.joint_names = self.arm_joints

        point = JointTrajectoryPoint()
        point.positions = positions
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
        if not self.set_entity_client.wait_for_service(timeout_sec=0.5):
            self.get_logger().warn('/gazebo/set_entity_state service not available')
            return

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

        future = self.set_entity_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=1.0)

        self.get_logger().info(f'Set object pose: {name} -> {x}, {y}, {z}')

    def resolve_position(self, target, position):
        # orchestrator/perception이 position을 주면 그 좌표 우선 사용
        if abs(position.x) > 0.001 or abs(position.y) > 0.001 or abs(position.z) > 0.001:
            return position

        # position이 없으면 데모용 기본 좌표 사용
        return self.default_positions.get(
            target,
            Point(x=0.45, y=0.0, z=0.78)
        )

    def sleep(self, sec):
        start = self.get_clock().now().nanoseconds
        duration = int(sec * 1e9)

        while self.get_clock().now().nanoseconds - start < duration:
            rclpy.spin_once(self, timeout_sec=0.1)


def main():
    rclpy.init()
    node = VLAControlActionServer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from std_msgs.msg import String
from geometry_msgs.msg import Point
from vla_interfaces.srv import DetectObjects, PlanTask
from vla_interfaces.action import ExecuteAction
from .state_machine import StateMachine, State
from .mock import MOCK_OBJECTS, MOCK_PLAN
import json
import math

MAX_RETRY = 3
PANDA_REACH = 0.85  # panda_link0 기준 도달 범위 (m)

class OrchestratorNode(Node):
    def __init__(self):
        super().__init__('vla_orchestrator')
        self.sm = StateMachine(self.get_logger())
        self.current_command = None
        self.retry_count = 0

        self.create_subscription(String, '/vla/command', self.command_cb, 10)
        self.detect_client = self.create_client(DetectObjects, '/vla/detect_objects')
        self.plan_client = self.create_client(PlanTask, '/vla/plan_task')
        self.execute_client = ActionClient(self, ExecuteAction, '/vla/execute_action')
        self.status_pub = self.create_publisher(String, '/vla/status', 10)

        self.get_logger().info('Orchestrator started!')

    # ── 명령 수신 ──────────────────────────────
    def command_cb(self, msg):
        if not self.sm.is_idle():
            self.get_logger().warn(f'{self.sm.state.value} 중 — 명령 무시')
            return
        self.current_command = msg.data
        self.retry_count = 0
        self.get_logger().info(f'명령: {msg.data}')
        self.sm.transition(State.PERCEIVING)
        self.publish_status(self.sm.state.value)
        self.call_perception()

    # ── Perception ─────────────────────────────
    def call_perception(self):
        if not self.detect_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().warn('perception 없음 → mock')
            self.sm.transition(State.PLANNING)
            self.call_planner(MOCK_OBJECTS)
            return
        future = self.detect_client.call_async(DetectObjects.Request())
        future.add_done_callback(self.perception_done_cb)

    def perception_done_cb(self, future):
        try:
            res = future.result()
            objects = [
                {"name": o.name,
                 "position": {"x": o.position.x, "y": o.position.y, "z": o.position.z}}
                for o in res.objects
            ]
            self.sm.transition(State.PLANNING)
            self.call_planner(objects)
        except Exception as e:
            self.get_logger().error(f'perception 실패: {e}')
            self.handle_error()

    # ── Planner ────────────────────────────────
    def call_planner(self, objects):
        if not self.plan_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().warn('planner 없음 → mock')
            self.sm.transition(State.EXECUTING)
            self.execute_plan(MOCK_PLAN, objects)
            return
        req = PlanTask.Request()
        req.instruction = self.current_command
        req.world_state_json = json.dumps(objects)
        req.robot_state_json = json.dumps({"arm": "ready"})
        future = self.plan_client.call_async(req)
        future.add_done_callback(lambda f: self.planner_done_cb(f, objects))

    def planner_done_cb(self, future, objects):
        try:
            res = future.result()
            if res.success:
                self.sm.transition(State.EXECUTING)
                self.execute_plan(json.loads(res.plan_json), objects)
            else:
                self.handle_error()
        except Exception as e:
            self.get_logger().error(f'planner 실패: {e}')
            self.handle_error()

    # ── Execute ────────────────────────────────
    def execute_plan(self, plan, objects):
        obj_map = {o['name']: o['position'] for o in objects}

        for step in plan['plan']:
            action = step['action']
            target = step['target']
            pos = obj_map.get(target)

            if pos and not self.is_reachable(pos):
                self.get_logger().error(f'{target} 도달 범위 밖')
                self.handle_error()
                return

            if not self.execute_client.wait_for_server(timeout_sec=2.0):
                self.get_logger().warn(f'control 없음 → mock: {action} {target}')
                continue

            goal = ExecuteAction.Goal()
            goal.action = action
            goal.target = target
            if pos:
                goal.position = Point(x=pos['x'], y=pos['y'], z=pos['z'])
            self.execute_client.send_goal_async(goal)

        self.sm.transition(State.DONE)
        self.sm.transition(State.IDLE)
        self.publish_status('IDLE')

    # ── 유틸 ───────────────────────────────────
    def is_reachable(self, pos):
        dist = math.sqrt(pos['x']**2 + pos['y']**2 + pos['z']**2)
        return dist < PANDA_REACH

    def handle_error(self):
        self.retry_count += 1
        if self.retry_count <= MAX_RETRY:
            self.get_logger().warn(f'재시도 {self.retry_count}/{MAX_RETRY}')
            self.sm.transition(State.PERCEIVING)
            self.call_perception()
        else:
            self.get_logger().error('최대 재시도 초과 → IDLE')
            self.sm.transition(State.ERROR)
            self.sm.transition(State.IDLE)
            self.publish_status('IDLE')

    def publish_status(self, status):
        msg = String()
        msg.data = status
        self.status_pub.publish(msg)

def main():
    rclpy.init()
    node = OrchestratorNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
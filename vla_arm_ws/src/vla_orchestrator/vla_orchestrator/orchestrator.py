import json
import math
from geometry_msgs.msg import Point
from .state_machine import StateMachine, State
from .mock import MOCK_OBJECTS, MOCK_PLAN, MOCK_ROBOT_STATE
from vla_interfaces.srv import DetectObjects, PlanTask
from vla_interfaces.action import ExecuteAction

MAX_RETRY = 3
PANDA_REACH = 0.85  # panda_link0 기준 도달 범위 (m)

class Orchestrator:
    def __init__(self, logger, detect_client, plan_client, execute_client, status_cb):
        self.sm = StateMachine(logger)
        self.logger = logger
        self.detect_client = detect_client
        self.plan_client = plan_client
        self.execute_client = execute_client
        self.status_cb = status_cb
        self.current_command = None
        self.retry_count = 0

    def handle_command(self, command):
        if not self.sm.is_idle():
            self.logger.warn(f'{self.sm.state.value} 중 — 명령 무시')
            return
        self.current_command = command
        self.retry_count = 0
        self.logger.info(f'명령: {command}')
        self.sm.transition(State.PERCEIVING)
        self.status_cb(self.sm.state.value)
        self.call_perception()

    # ── Perception ─────────────────────────────
    def call_perception(self):
        if not self.detect_client.wait_for_service(timeout_sec=2.0):
            self.logger.warn('perception 없음 → mock')
            self.sm.transition(State.PLANNING)
            self.call_planner(MOCK_OBJECTS)
            return
        req = DetectObjects.Request()
        req.target = ""
        future = self.detect_client.call_async(req)
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
            self.logger.error(f'perception 실패: {e}')
            self.handle_error()

    # ── Planner ────────────────────────────────
    def call_planner(self, objects):
        if not self.plan_client.wait_for_service(timeout_sec=2.0):
            self.logger.warn('planner 없음 → mock')
            self.sm.transition(State.EXECUTING)
            self.execute_plan(MOCK_PLAN, objects)
            return
        req = PlanTask.Request()
        req.instruction = self.current_command
        req.world_state_json = json.dumps(objects)
        req.robot_state_json = json.dumps(MOCK_ROBOT_STATE)
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
            self.logger.error(f'planner 실패: {e}')
            self.handle_error()

    # ── Execute ────────────────────────────────
    def execute_plan(self, plan, objects):
        obj_map = {o['name']: o['position'] for o in objects}
        for step in plan['plan']:
            action = step['action']
            target = step.get('target') or ''   # open/close_gripper 등은 target=null → '' 로 보정 (str 필드)
            pos = obj_map.get(target)
            if pos and not self.is_reachable(pos):
                self.logger.error(f'{target} 도달 범위 밖')
                self.handle_error()
                return
            if not self.execute_client.wait_for_server(timeout_sec=2.0):
                self.logger.warn(f'control 없음 → mock: {action} {target}')
                continue
            goal = ExecuteAction.Goal()
            goal.action = action
            goal.target = target
            if pos:
                goal.position = Point(x=pos['x'], y=pos['y'], z=pos['z'])
            self.execute_client.send_goal_async(goal)
        self.sm.transition(State.DONE)
        self.sm.transition(State.IDLE)
        self.status_cb('IDLE')

    # ── 유틸 ───────────────────────────────────
    def is_reachable(self, pos):
        dist = math.sqrt(pos['x']**2 + pos['y']**2 + pos['z']**2)
        return dist < PANDA_REACH

    def handle_error(self):
        self.retry_count += 1
        if self.retry_count <= MAX_RETRY:
            self.logger.warn(f'재시도 {self.retry_count}/{MAX_RETRY}')
            self.sm.transition(State.PERCEIVING)
            self.call_perception()
        else:
            self.logger.error('최대 재시도 초과 → IDLE')
            self.sm.transition(State.ERROR)
            self.sm.transition(State.IDLE)
            self.status_cb('IDLE')
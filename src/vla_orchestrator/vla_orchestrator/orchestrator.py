import json
import math
from .state_machine import StateMachine, State
from .mock import MOCK_OBJECTS, MOCK_PLAN

MAX_RETRY = 3
PANDA_REACH = 0.85  # panda_link0 기준 도달 범위 (m)

class Orchestrator:
    def __init__(self, logger, detect_client, plan_client, execute_client, status_cb):
        self.sm = StateMachine(logger)
        self.logger = logger
        self.detect_client = detect_client
        self.plan_client = plan_client
        self.execute_client = execute_client
        self.status_cb = status_cb  # 상태 발행 콜백
        self.current_command = None
        self.retry_count = 0

    def handle_command(self, command):
        if not self.sm.is_idle():
            self.logger.warn(f'{self.sm.state.value} ...')
            return
        self.current_command = command
        self.retry_count = 0
        self.logger.info(f'명령: {command}')
        self.sm.transition(State.PERCEIVING)
        self.status_cb(self.sm.state.value)
        self.call_perception()

    def call_perception(self):
        if not self.detect_client.wait_for_service(timeout_sec=2.0):
            self.logger.warn('perception 없음 → mock')
            self.sm.transition(State.PLANNING)
            self.call_planner(MOCK_OBJECTS)
            return
        future = self.detect_client.call_async_detect()
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

    def call_planner(self, objects):
        if not self.plan_client.wait_for_service(timeout_sec=2.0):
            self.logger.warn('planner 없음 → mock')
            self.sm.transition(State.EXECUTING)
            self.execute_plan(MOCK_PLAN, objects)
            return
        future = self.plan_client.call_async_plan(self.current_command, objects)
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

    def execute_plan(self, plan, objects):
        obj_map = {o['name']: o['position'] for o in objects}
        for step in plan['plan']:
            action = step['action']
            target = step['target']
            pos = obj_map.get(target)
            if pos and not self.is_reachable(pos):
                self.logger.error(f'{target} 도달 범위 밖')
                self.handle_error()
                return
            if not self.execute_client.wait_for_server(timeout_sec=2.0):
                self.logger.warn(f'control 없음 → mock: {action} {target}')
                continue
            self.execute_client.send_goal(action, target, pos)
        self.sm.transition(State.DONE)
        self.sm.transition(State.IDLE)
        self.status_cb('IDLE')

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
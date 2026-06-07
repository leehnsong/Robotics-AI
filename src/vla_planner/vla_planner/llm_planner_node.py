#!/usr/bin/env python3
import json
import os
import rclpy
from rclpy.node import Node
from google import genai
from google.genai import types
from vla_interfaces.srv import PlanTask

GEMINI_MODEL = "gemini-2.5-flash"
VALID_ACTIONS = {"pick", "place", "open_gripper", "close_gripper", "move_home"}
ROBOT_API = """사용 가능한 액션(이 목록의 action만 사용):
- pick(target): 지정한 물체를 집는다
- place(target): 들고 있는 물체를 지정한 위치/물체 위에 놓는다
- open_gripper() / close_gripper()
- move_home(): 초기 자세 복귀"""

PLAN_SCHEMA = {
    "type": "object",
    "properties": {
        "reasoning": {"type": "string"},
        "plan": {"type": "array", "items": {
            "type": "object",
            "properties": {
                "action": {"type": "string", "enum": list(VALID_ACTIONS)},
                "target": {"type": "string"}},
            "required": ["action"]}}},
    "required": ["plan"]}


def make_plan(client, instruction, world_state_json, robot_state_json):
    system = (
        "너는 로봇 팔의 태스크 플래너다. 사용자 명령을 아래 로봇 API의 "
        "액션들로만 구성된 실행 가능한 순서로 분해하라.\n"
        f"{ROBOT_API}\n[현재 장면의 물체] {world_state_json}\n"
        f"[로봇 상태] {robot_state_json}\n"
        "규칙: 장면에 없는 물체 금지. place 전에 그 물체를 pick 했는지 확인.")
    r = client.models.generate_content(
        model=GEMINI_MODEL,
        contents=[system + "\n\n명령: " + instruction],
        config=types.GenerateContentConfig(
            response_mime_type="application/json", response_schema=PLAN_SCHEMA))
    return r.text


def validate_plan(raw_json, known_objects):
    try:
        data = json.loads(raw_json)
    except (json.JSONDecodeError, TypeError):
        return None, "JSON 파싱 실패"
    plan = data.get("plan")
    if not isinstance(plan, list) or not plan:
        return None, "빈/잘못된 plan"
    safe = []
    for step in plan:
        act, tgt = step.get("action"), step.get("target")
        if act not in VALID_ACTIONS:                      # 환각 액션 차단
            continue
        if act in ("pick", "place") and tgt not in known_objects:  # 환각 물체 차단
            continue
        safe.append({"action": act, "target": tgt})
    return (safe, None) if safe else (None, "유효 액션 없음")


class PlannerNode(Node):
    def __init__(self):
        super().__init__("llm_planner_node")
        
        # 안전한 API KEY 로드 (안내 로그 추가)
        api_key = os.environ.get("GEMINI_API_KEY")
        if not api_key:
            self.get_logger().error("환경변수 'GEMINI_API_KEY'가 설정되지 않았습니다! .bashrc를 확인하세요.")
        
        self.client = genai.Client(api_key=api_key)
        self.create_service(PlanTask, "/plan_task", self._on_plan)
        self.get_logger().info("llm_planner_node 준비 완료 (/plan_task 서비스 대기 중...)")

    def _on_plan(self, req, resp):
        try:
            raw = make_plan(self.client, req.instruction,
                            req.world_state_json, req.robot_state_json)
            try:
                known = [o["name"] for o in json.loads(req.world_state_json)]
            except Exception:
                known = []
            safe, err = validate_plan(raw, known)
            resp.success = safe is not None
            resp.plan_json = json.dumps({"plan": safe or []}, ensure_ascii=False)
            self.get_logger().info(f"계획 생성 성공! plan: {resp.plan_json} ({err or 'ok'})")
        except Exception as e:                            # 타임아웃/API 오류 방어
            self.get_logger().warn(f"planner 오류(안전 실패): {e}")
            resp.success = False
            resp.plan_json = json.dumps({"plan": []})
        return resp


def main():
    rclpy.init(); n = PlannerNode()
    rclpy.spin(n); n.destroy_node(); rclpy.shutdown()


if __name__ == "__main__":
    main()

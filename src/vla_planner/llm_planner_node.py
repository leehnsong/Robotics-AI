#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import os
import json

# 구글 제미나이(Gemini) API 라이브러리 (Step 1에서 pip install google-genai로 설치됨)
from google import genai
from google.genai import types

# 팀 커스텀 인터페이스 (Step 4 참고)
from vla_interfaces.srv import PlanTask

class LlmPlannerNode(Node):
    def __init__(self):
        super().__init__('llm_planner_node')
        
        # 1. 환경변수에서 Gemini API 키 가져오기 (Step 1에서 export한 값)
        api_key = os.environ.get("GEMINI_API_KEY")
        if not api_key:
            self.get_logger().error("환경변수 'GEMINI_API_KEY'를 찾을 수 없습니다! .bashrc를 확인하거나 터미널에 export 하세요.")
            # 가이드 규칙에 따라 에러 시 시스템이 죽지 않도록 예외 처리용 mock 키 할당 또는 대기 가능
        
        # Gemini 클라이언트 초기화 (기본 모델로 가성비가 좋고 속도가 빠른 gemini-2.5-flash 사용)
        self.client = genai.Client(api_key=api_key)
        self.model_name = 'gemini-2.5-flash'
        
        # 2. ROS 2 서비스 서버 생성 (가이드에 명시된 서비스 이름과 타입 적용)
        self.srv = self.create_service(PlanTask, '/plan_task', self.plan_task_callback)
        self.get_logger().info('LLM 플래너 노드(두뇌)가 활성화되었습니다. /plan_task 서비스 대기 중...')

    def plan_task_callback(self, request, response):
        """
        총괄 노드(task_manager_node)로부터 계획 요청을 받는 콜백 함수
        request.instruction: 자연어 명령 (예: "빨간 블록을 파란 접시에 놓아줘")
        request.world_state_json: 인식단에서 넘겨준 물체 정보 JSON 문자열
        request.robot_state_json: 로봇 상태 JSON 문자열
        """
        self.get_logger().info(f"계획 요청 수신 완료!")
        self.get_logger().info(f" -> 명령어: '{request.instruction}'")
        self.get_logger().info(f" -> 환경 상태(JSON): {request.world_state_json}")

        # 규칙 2번 반영: 물체 이름을 하드코딩하지 않고, world_state와 instruction을 그대로 LLM에 주입
        
        # 3. 모델이 반드시 지켜야 할 JSON 출력 양식(Schema) 정의 (Step 8-3, Step 11 가이드 기준)
        # 반환할 plan_json은 [{"action": "pick", "target": "물체명"}, {"action": "place", "target": "물체명"}] 배열 형태여야 함
        action_schema = types.Schema(
            type=types.Type.OBJECT,
            properties={
                "action": types.Schema(
                    type=types.Type.STRING,
                    enum=["pick", "place"], # 로봇이 수행 가능한 기본 액션 목록 제한
                    description="로봇이 수행해야 할 행동 타입"
                ),
                "target": types.Schema(
                    type=types.Type.STRING,
                    description="행동의 대상이 되는 물체 이름. world_state에 존재하는 물체여야 함 (하드코딩 금지)"
                )
            },
            required=["action", "target"]
        )

        response_schema = types.Schema(
            type=types.Type.ARRAY,
            items=action_schema,
            description="유저의 명령을 완수하기 위한 로봇 행동 시퀀스 리스트"
        )

        # 4. 프롬프트 엔지니어링 (로봇 API 및 인프라 제약 조건 주입)
        system_instruction = (
            "당신은 7자유도 로봇 팔(Panda Arm)의 작업 계획을 수립하는 고성능 AI 두뇌입니다.\n"
            "사용자의 자연어 명령(instruction)과 현재 작업 공간의 물체 정보(world_state)를 기반으로,\n"
            "로봇이 순서대로 실행해야 할 pick 및 place 행동 시퀀스를 생성해야 합니다.\n\n"
            "주의 규칙:\n"
            "1. 절대 물체 이름을 임의로 지어내거나 하드코딩하지 마십시오. 오직 주어진 world_state에 명시된 name만 target으로 삼아야 합니다.\n"
            "2. 물체를 옮기려면 반드시 먼저 'pick'을 한 뒤에 원하는 목적지나 물체에 'place'를 해야 합니다.\n"
            "3. 주어진 world_state 정보 내에서 명령을 수행할 수 없는 경우, 빈 배열 []을 반환하십시오."
        )

        user_content = (
            f"User Instruction: {request.instruction}\n"
            f"Current World State: {request.world_state_json}\n"
            f"Robot State: {request.robot_state_json}"
        )

        try:
            # 5. Gemini API 구조화 응답(Structured Outputs) 호출
            self.get_logger().info("Gemini API 호출 중...")
            api_response = self.client.models.generate_content(
                model=self.model_name,
                contents=user_content,
                config=types.GenerateContentConfig(
                    system_instruction=system_instruction,
                    response_mime_type="application/json",
                    response_schema=response_schema,
                    temperature=0.0 # 계획의 일관성과 정확성을 위해 창의성을 0으로 제한
                ),
            )
            
            # API 결과 추출
            generated_plan = api_response.text
            self.get_logger().info(f"Gemini 생성 계획: {generated_plan}")

            # 6. validate_plan (환각/포맷 오류 검증 로직 - 가이드 Step 8)
            if self.validate_plan(generated_plan, request.world_state_json):
                response.plan_json = generated_plan
            else:
                self.get_logger().warn("생성된 계획이 유효성 검증(validate_plan)을 통과하지 못했습니다. 빈 계획을 전송합니다.")
                response.plan_json = "[]"

        except Exception as e:
            self.get_logger().error(f"Gemini API 호출 또는 처리 중 오류 발생: {str(e)}")
            response.plan_json = "[]" # 시스템 다운 방지를 위한 예외 처리(빈 배열 반환)

        return response

    def validate_plan(self, plan_json_str, world_state_json_str):
        """
        Gemini가 출력한 결과가 형식에 맞고 환각(존재하지 않는 물체 조작 등)이 없는지 검증
        """
        try:
            plan = json.loads(plan_json_str)
            world_state = json.loads(world_state_json_str)
            
            #  존재하는 물체 이름 리스트 생성
            existing_targets = [obj.get("name") for obj in world_state if "name" in obj]
            
            if not isinstance(plan, list):
                return False
                
            for step in plan:
                action = step.get("action")
                target = step.get("target")
                
                # 가이드 조건 검증: 올바른 액션인지 확인
                if action not in ["pick", "place"]:
                    return False
                
                # 규칙 2번 검증: 환각(텍스트에 없는 엉뚱한 물체 이름) 방지 검증
                if target not in existing_targets:
                    self.get_logger().warn(f"환각 감지: world_state에 존재하지 않는 물체 '{target}'이 계획에 포함됨.")
                    return False
                    
            return True
        except Exception:
            return False

def main(args=None):
    rclpy.init(args=args)
    node = LlmPlannerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

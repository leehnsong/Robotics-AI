# 🤖 VLA Arm — Vision-Language-Action 로봇 팔 제어 시스템

> ROS 2 Humble + Gazebo Classic 11 환경에서 Panda 로봇 팔을 자연어 명령으로 제어하는 VLA(Vision-Language-Action) 프로젝트

---

## 📌 프로젝트 개요

사용자가 **"빨간 블록을 파란 접시에 올려"** 같은 자연어 명령을 입력하면, 카메라로 장면을 인식하고 AI가 행동 계획을 세워 로봇 팔이 자율적으로 작업을 수행

- **환경**: Ubuntu 22.04 + ROS 2 Humble + Gazebo Classic 11
- **로봇**: Franka Panda 7-DOF 로봇 팔
- **AI 모델**: Google Gemini 2.5 Flash (Vision + Language)
- **모션 플래닝**: MoveIt 2 + pymoveit2
- **팀원**:
-   202135569 이현송
  - 202135829 정민기
  - 202235065 신민서
  - 202239881 최유리
  - 202334273 김예서
  - 202434741 김은수

---

## 📦 주요 패키지 요약

| 패키지 | 역할 | 브랜치 |
|------------------|------|--------|
| `vla_perception` | 카메라 + Gemini로 물체 3D 좌표 산출 | `feat/perception` |
| `vla_planner`    | 자연어 명령 → 행동 시퀀스 계획 | `feat/planner` |
| `vla_control` + `arm_bringup` | MoveIt2로 팔 실제 실행, 그리퍼 제어 | `feat/control` |
| `vla_orchestrator` | 전체 흐름 조율 + 안전 관리 | `feat/orchestrator` |
| `vla_interfaces` + `vla_bringup` | 인터페이스 정의 + 통합 런치 | `feat/bringup` |

---

## 🗂️ 프로젝트 진행 과정

### 0단계 — 사전 합의 (인터페이스 고정)
5명이 코드 작성 전 **공통 인터페이스를 먼저 확정**했습니다. 형식(모양)은 고정, 값(내용)은 런타임에 동적으로 흐르게 설계해 어떤 월드에서도 코드 수정 없이 동작합니다.

### 1단계 — 환경 설치 및 워크스페이스 구성
ROS 2 Humble, Gazebo Classic 11, MoveIt, pymoveit2, google-genai 등 의존성을 설치하고 `~/vla_arm_ws` 워크스페이스를 구성했습니다.

### 2단계 — 인터페이스 패키지 우선 빌드
모든 노드가 공유하는 메시지/서비스/액션 타입(`vla_interfaces`)을 가장 먼저 정의하고 빌드했습니다. 이 패키지가 빌드돼 있어야 나머지 노드들이 컴파일됩니다.

### 3단계 — 시뮬레이션 환경 구축 (arm_bringup)
Panda 팔 URDF에 `gazebo_ros2_control`과 RGB-D 카메라 센서를 추가하고, Gazebo 월드에 조작 대상 물체(블록, 접시)를 배치했습니다.

### 4단계 — MoveIt 2 연동 및 모션 검증
MoveIt Setup Assistant로 플래닝 그룹(`arm`, `gripper`)을 설정하고, pymoveit2로 좌표 이동 테스트를 진행해 시뮬레이션 팔의 실제 동작을 확인했습니다.

### 5단계 — 각 노드 개발 (병렬)
인터페이스가 고정된 뒤 5명이 각자 담당 패키지를 독립적으로 개발했습니다.
- **perception_node**: Gemini Vision으로 물체 감지 → 깊이 이미지로 3D 좌표 변환
- **llm_planner_node**: Gemini Text로 명령을 행동 시퀀스로 분해 + 환각/포맷 검증
- **arm_controller_node**: pymoveit2로 pick/place 실행, 충돌 회피, 비상정지
- **safety_node + task_manager_node**: 전체 흐름 조율 및 안전 감시

### 6단계 — 통합 및 검증
모든 노드를 연결해 end-to-end 테스트를 진행하고, `vla_bringup`의 통합 런치 파일로 단일 명령 실행을 완성했습니다.

### 7단계 — 견고성·안전 점검
LLM 예외 처리, 비상정지, 충돌 회피, 작업공간 범위 검사 등을 테스트했습니다.

---

## 🏗️ 시스템 아키텍처

### 노드 구성 한눈에 보기

```
┌──────────────────────────────────────────────────────────────────────┐
│                         vla_arm_ws                                   │
│                                                                      │
│  [Gazebo + Panda 팔]          [ROS 2 노드 레이어]                       │
│  ┌──────────────────┐                                                │
│  │ scene.world      │  /joint_states (Topic)                         │
│  │  - red_block     │ ◄────────────────────── joint_state_broadcaster│
│  │  - blue_plate    │                                                │
│  │  - table         │  /camera/image_raw        ┌─────────────────┐ │
│  │  - RGB-D camera  │ ─────────────────────────►│ perception_node │ │
│  │  - Panda arm     │  /camera/depth/image_raw  │  (vla_perception│ │
│  └──────────────────┘ ─────────────────────────►│   .py)          │ │
│                        /camera/camera_info      └────────┬────────┘ │
│                       ─────────────────────────►         │           │
│                                                 /detect_objects(srv) │
│  ┌─────────────────────────────────────────┐            │           │
│  │         task_manager_node               │◄───────────┘           │
│  │         (vla_orchestrator)              │                         │
│  │                                         │  /plan_task (srv)       │
│  │  /user_command ──► [상태머신]           │──────────────►┌───────┐│
│  │  Topic 구독       IDLE→DETECT           │               │planner││
│  │                   →PLAN→EXEC            │◄──────────────│ node  ││
│  └──────────────────┬──────────────────────┘  plan_json   └───────┘│
│                     │ /execute_action (Action)                       │
│                     ▼                                                │
│           ┌─────────────────┐   MoveGroup Action                    │
│           │arm_controller   │──────────────────► MoveIt2            │
│           │_node            │                    (move_group)        │
│           │(vla_control)    │◄── /emergency_stop (Topic, latched)   │
│           └─────────────────┘         ▲                             │
│                     │                 │                              │
│                     │        ┌────────┴──────┐                      │
│                     │        │ safety_node   │◄── /set_estop (srv)  │
│                     │        │(vla_orchestr) │                      │
│                     │        └───────────────┘                      │
│                     │ /arm_controller/joint_trajectory_controller    │
│                     ▼        /follow_joint_trajectory (Action)       │
│              [Gazebo 팔 물리 실행]                                   │
└──────────────────────────────────────────────────────────────────────┘
```

---

### 전체 데이터 흐름 (시퀀스)

```
사용자
  │
  │  ros2 topic pub /user_command  std_msgs/msg/String
  │  '{"data": "빨간 블록을 파란 접시에 놓아줘"}'
  ▼
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
 STEP 1. [task_manager_node]  /user_command 토픽 수신
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
  │
  │  Service Request: /detect_objects
  │  req.target = ""  (빈 문자열 → KNOWN_OBJECTS 전체 탐색)
  ▼
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
 STEP 2. [perception_node]  장면 인식
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
  │
  │  내부 처리:
  │  ① self.rgb  ← /camera/image_raw  (Topic 구독, 버퍼)
  │  ② self.depth← /camera/depth/image_raw (Topic 구독, 버퍼)
  │  ③ self.K   ← /camera/camera_info (Topic 구독, 버퍼)
  │
  │  ④ Gemini Vision API 호출
  │     입력: PIL 이미지 + 프롬프트
  │           "Return the 2D bounding box of the {target}...
  │            Output ONLY JSON like {"box_2d":[ymin,xmin,ymax,xmax]}"
  │     출력: {"box_2d": [210, 180, 290, 260]}  (0~1000 정규화)
  │
  │  ⑤ 바운딩박스 중심 픽셀 (u, v) 계산
  │     u = (xmin + xmax) / 2 / 1000 * W
  │     v = (ymin + ymax) / 2 / 1000 * H
  │
  │  ⑥ 깊이값 d = nanmedian(depth[v-2:v+3, u-2:u+3])  # 5×5 중앙값 (노이즈 제거)
  │
  │  ⑦ 카메라 좌표계 3D 역투영
  │     p.x = (u - cx) * d / fx
  │     p.y = (v - cy) * d / fy
  │     p.z = d
  │     frame_id = "camera_optical_frame"
  │
  │  ⑧ TF 변환: camera_optical_frame → panda_link0 (base_link)
  │     tf_buffer.lookup_transform(base, optical_frame, Time())
  │     → do_transform_point(p, tf)
  │
  │  Service Response: DetectedObject[]
  │  [{"name": "red block",  "position": {x:0.45, y:0.15, z:0.43}},
  │   {"name": "blue plate", "position": {x:0.55, y:-0.20, z:0.41}}]
  ▼
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
 STEP 3. [task_manager_node]  world_state_json 조립
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
  │
  │  objs = {"red block": Point(0.45,0.15,0.43), ...}
  │  world_state_json = '[{"name":"red block"},{"name":"blue plate"}]'
  │  robot_state_json = '{"gripper":"open","holding":null}'
  │
  │  Service Request: /plan_task
  │  req.instruction       = "빨간 블록을 파란 접시에 놓아줘"
  │  req.world_state_json  = world_state_json
  │  req.robot_state_json  = robot_state_json
  ▼
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
 STEP 4. [llm_planner_node]  행동 계획 생성
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
  │
  │  내부 처리:
  │  ① 시스템 프롬프트 구성 (make_plan)
  │     - ROBOT_API: 사용 가능한 액션 명세
  │       "pick(target) / place(target) / open_gripper() /
  │        close_gripper() / move_home()"
  │     - 현재 장면 물체 목록 (world_state_json 주입)
  │     - 로봇 현재 상태 (robot_state_json 주입)
  │     - 규칙: 장면에 없는 물체 금지, place 전 pick 확인
  │
  │  ② Gemini Text API 호출
  │     model: gemini-2.5-flash
  │     response_mime_type: "application/json"
  │     response_schema: PLAN_SCHEMA (JSON Schema 강제)
  │       → 환각 액션 원천 차단 (enum: VALID_ACTIONS)
  │
  │  ③ validate_plan() 검증
  │     - JSON 파싱 실패 → success=False 반환
  │     - 빈/잘못된 plan → success=False 반환
  │     - action이 VALID_ACTIONS 외 → 해당 스텝 제거
  │     - pick/place의 target이 known_objects 외 → 해당 스텝 제거
  │     - 유효 스텝 0개 → success=False 반환
  │
  │  Service Response:
  │  resp.success   = True
  │  resp.plan_json = '{"plan": [
  │    {"action": "open_gripper",  "target": ""},
  │    {"action": "pick",          "target": "red block"},
  │    {"action": "place",         "target": "blue plate"},
  │    {"action": "close_gripper", "target": ""},
  │    {"action": "move_home",     "target": ""}
  │  ]}'
  ▼
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
 STEP 5. [task_manager_node]  스텝별 순차 실행
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
  │
  │  plan = [open_gripper, pick, place, close_gripper, move_home]
  │
  │  for step in plan:
  │    goal.action   = step["action"]      # e.g. "pick"
  │    goal.target   = step["target"]      # e.g. "red block"
  │    goal.position = objs[goal.target]   # Point(0.45,0.15,0.43)
  │
  │    Action Goal: /execute_action  (ExecuteAction)
  ▼
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
 STEP 6. [arm_controller_node]  물리 동작 실행
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
  │
  │  ① 비상정지 확인
  │     if self.estop == True → goal.abort(), return
  │
  │  ② 작업공간 범위 검사 (pick/place만 해당)
  │     WS = x:(0.2~0.7), y:(-0.4~0.4), z:(0.0~0.6)
  │     if not in_ws(p) → goal.abort(), return
  │
  │  ③ 액션별 동작 시퀀스 (피드백 publish_feedback 포함)
  │
  │  [pick 시퀀스]
  │   feedback: "그리퍼 열기"  → gripper.open()
  │   feedback: "물체 위로"    → moveit2.move_to_pose(x, y, z+0.10)
  │   feedback: "하강"         → moveit2.move_to_pose(x, y, z)
  │   feedback: "잡기"         → gripper.close()
  │   feedback: "들어올리기"   → moveit2.move_to_pose(x, y, z+0.10)
  │
  │  [place 시퀀스]
  │   feedback: "놓을 곳 위로" → moveit2.move_to_pose(x, y, z+0.12)
  │   feedback: "하강"         → moveit2.move_to_pose(x, y, z+0.04)
  │   feedback: "놓기"         → gripper.open()
  │   feedback: "복귀"         → moveit2.move_to_pose(x, y, z+0.12)
  │
  │  ④ pymoveit2 → MoveIt2 move_group → arm_controller
  │     /follow_joint_trajectory (Action)
  │     → Gazebo 관절 위치 제어 → 팔 물리 이동
  │
  │  Action Result: success=True, message="완료"
  ▼
 task_manager_node: 다음 스텝으로 → 전체 plan 완료
```

---

### 안전 시스템 데이터 흐름

```
외부 트리거 (운영자 또는 자동화 시스템)
  │
  │  ros2 service call /set_estop std_srvs/srv/SetBool "{data: true}"
  ▼
┌─────────────────────────────────────────────────┐
│ safety_node (vla_orchestrator/safety_node.py)   │
│                                                 │
│  /set_estop 서비스 수신                         │
│  → self.pub.publish(Bool(data=True))            │
│                                                 │
│  발행: /emergency_stop  (Topic, TRANSIENT_LOCAL)│
│  ※ TRANSIENT_LOCAL = "latched" 토픽             │
│    → 나중에 구독하는 노드도 마지막 값을 즉시 받음│
└───────────────────────┬─────────────────────────┘
                        │
                        │ /emergency_stop: Bool(data=True)
                        ▼
┌─────────────────────────────────────────────────┐
│ arm_controller_node (vla_control)               │
│                                                 │
│  구독: /emergency_stop (QoS: TRANSIENT_LOCAL)   │
│  → self.estop = True                            │
│  → moveit2.cancel_execution()  # 진행중 모션 중단│
│  → 이후 모든 /execute_action 요청 → 즉시 abort  │
└─────────────────────────────────────────────────┘

해제:
  ros2 service call /set_estop std_srvs/srv/SetBool "{data: false}"
  → Bool(data=False) 발행 → self.estop = False → 정상 복귀
```

---

### 패키지 구성 및 파일별 역할

```
vla_arm_ws/src/
├── vla_interfaces/                      ← 공통 타입 정의 (P5 전담, ament_cmake)
│   ├── msg/DetectedObject.msg           # 물체 1개: name(string) + position(Point)
│   ├── srv/DetectObjects.srv            # 요청: target / 응답: DetectedObject[]
│   ├── srv/PlanTask.srv                 # 요청: instruction+world+robot / 응답: plan_json
│   ├── action/ExecuteAction.action      # 목표: action+target+position / 결과: success+msg
│   ├── CMakeLists.txt                   # rosidl_generate_interfaces 설정
│   └── package.xml                      # rosidl 의존성 선언
│
├── arm_bringup/                         ← 시뮬레이션 환경 (P3 전담)
│   ├── worlds/scene.world               # Gazebo SDF: 테이블+red_block+blue_plate+조명
│   ├── config/controllers.yaml          # joint_state_broadcaster + arm_controller 설정
│   │                                    #   joints: panda_joint1~7, position 제어
│   ├── urdf/panda_gazebo.urdf.xacro     # Panda URDF 확장:
│   │                                    #   (1) ros2_control 블록: 7관절 position I/F
│   │                                    #   (2) RGB-D 카메라: camera_link + depth sensor
│   │                                    #   (3) gazebo_ros2_control 플러그인 연결
│   └── launch/gazebo.launch.py          # Gazebo + 팔 스폰 + 컨트롤러 + robot_state_pub
│
├── arm_moveit_config/                   ← MoveIt 경로계획 설정 (Setup Assistant 생성)
│   └── launch/move_group.launch.py      # MoveIt move_group 노드 실행
│
├── vla_perception/
│   └── vla_perception/
│       └── perception_node.py           ← PerceptionNode 클래스
│                                        # [구독] /camera/image_raw → self.rgb 버퍼
│                                        # [구독] /camera/depth/image_raw → self.depth 버퍼
│                                        # [구독] /camera/camera_info → self.K (fx,fy,cx,cy)
│                                        # [서비스 서버] /detect_objects
│                                        #   └ _on_detect(): locate() 호출 → DetectedObject[] 반환
│                                        # [내부] _gemini_box(): Gemini Vision API → bbox JSON
│                                        # [내부] locate(): bbox→픽셀→깊이→3D→TF변환
│
├── vla_planner/
│   └── vla_planner/
│       └── llm_planner_node.py          ← PlannerNode 클래스
│                                        # [서비스 서버] /plan_task
│                                        #   └ _on_plan(): make_plan() + validate_plan()
│                                        # [외부함수] make_plan(): Gemini Text API 호출
│                                        #   시스템 프롬프트 = ROBOT_API + world_state + 규칙
│                                        #   response_schema = PLAN_SCHEMA (JSON Schema)
│                                        # [외부함수] validate_plan(): 환각/포맷 검증
│                                        #   - action ∉ VALID_ACTIONS → 스텝 제거
│                                        #   - target ∉ known_objects → 스텝 제거
│
├── vla_control/
│   └── vla_control/
│       └── arm_controller_node.py       ← ArmControllerNode 클래스
│                                        # [구독] /emergency_stop (TRANSIENT_LOCAL QoS)
│                                        #   └ _on_estop(): self.estop 플래그 + 모션 중단
│                                        # [액션 서버] /execute_action
│                                        #   └ _execute(): 비상정지·작업공간 검사 후 동작
│                                        #     pick:  open→위→하강→close→들어올리기
│                                        #     place: 위→하강→open→복귀
│                                        #     open/close_gripper: GripperInterface 직접
│                                        #     move_home: move_to_configuration(HOME_CONFIG)
│                                        # [내부] _add_collision_objects(): 테이블 충돌 등록
│                                        # [내부] _move(x,y,z): pymoveit2.move_to_pose()
│                                        # pymoveit2.MoveIt2 → MoveGroup → arm_controller
│
├── vla_orchestrator/
│   └── vla_orchestrator/
│       ├── task_manager_node.py         ← TaskManagerNode 클래스
│       │                                # [구독] /user_command (std_msgs/String)
│       │                                #   └ _on_cmd(): 전체 파이프라인 실행
│       │                                #     ① detect_cli.call_async(DetectObjects.Request())
│       │                                #     ② plan_cli.call_async(PlanTask.Request(...))
│       │                                #     ③ exec_cli.send_goal_async(ExecuteAction.Goal())
│       │                                #        × plan 스텝 수만큼 반복
│       │                                # [서비스 클라이언트] /detect_objects
│       │                                # [서비스 클라이언트] /plan_task
│       │                                # [액션 클라이언트]  /execute_action
│       │
│       └── safety_node.py              ← SafetyNode 클래스
│                                        # [발행] /emergency_stop (Bool, TRANSIENT_LOCAL)
│                                        #   → 시작 시 False 발행 (초기화)
│                                        # [서비스 서버] /set_estop (std_srvs/SetBool)
│                                        #   └ _set(): 수신값 그대로 /emergency_stop 발행
│
└── vla_bringup/
    └── launch/
        └── demo.launch.py               ← 통합 런치 파일
                                         # ① arm_bringup/gazebo.launch.py 포함 (Gazebo+팔)
                                         # ② arm_moveit_config/move_group.launch.py 포함
                                         # ③ TimerAction(8초 지연) 후 5개 노드 실행:
                                         #    perception_node, llm_planner_node,
                                         #    arm_controller_node, safety_node,
                                         #    task_manager_node
```

---

### ROS 통신 채널 전체 정리

#### Topic (발행/구독 — 단방향 연속 스트림)

| 토픽 이름 | 메시지 타입 | 발행자 | 구독자 | 용도 | QoS |
|-----------|-------------|--------|--------|------|-----|
| `/camera/image_raw` | `sensor_msgs/Image` | Gazebo 카메라 플러그인 | `perception_node` | RGB 영상 스트림 | 기본 |
| `/camera/depth/image_raw` | `sensor_msgs/Image` (32FC1) | Gazebo 카메라 플러그인 | `perception_node` | 깊이 이미지 스트림 (미터 단위) | 기본 |
| `/camera/camera_info` | `sensor_msgs/CameraInfo` | Gazebo 카메라 플러그인 | `perception_node` | 카메라 내부 파라미터 (fx, fy, cx, cy) | 기본 |
| `/joint_states` | `sensor_msgs/JointState` | `joint_state_broadcaster` | MoveIt, RViz | 관절 각도·속도 브로드캐스트 | 기본 |
| `/user_command` | `std_msgs/String` | 사용자 (CLI) | `task_manager_node` | 자연어 명령 입력 | 기본 |
| `/emergency_stop` | `std_msgs/Bool` | `safety_node` | `arm_controller_node` | 비상정지 신호 (True=정지) | **TRANSIENT_LOCAL** |

> **TRANSIENT_LOCAL(latched) 이유**: `arm_controller_node`가 `safety_node`보다 늦게 시작되어도 마지막 발행값(`False`)을 즉시 수신해 초기 상태를 올바르게 설정할 수 있도록 함. 비상정지 해제(`False`) 후 재시작한 컨트롤러도 즉시 정상 상태로 시작.

#### Service (요청/응답 — 동기식 단발 통신)

| 서비스 이름 | 타입 | 서버 | 클라이언트 | 요청 | 응답 |
|-------------|------|------|-----------|------|------|
| `/detect_objects` | `vla_interfaces/DetectObjects` | `perception_node` | `task_manager_node` | `target: ""` (전체) 또는 특정 물체명 | `DetectedObject[]` (name + position) |
| `/plan_task` | `vla_interfaces/PlanTask` | `llm_planner_node` | `task_manager_node` | instruction + world_state_json + robot_state_json | `success` + `plan_json` |
| `/set_estop` | `std_srvs/SetBool` | `safety_node` | 운영자 (CLI) | `data: true/false` | `success` + `message` |

> **Service를 선택한 이유**: 1회성 질의응답 패턴. 클라이언트가 결과를 받아야 다음 단계로 진행 가능하므로 동기식 통신이 적합. Topic은 응답 개념이 없고 Action은 오버헤드가 큼.

#### Action (목표 전송 + 피드백 + 결과 — 장시간 비동기 통신)

| 액션 이름 | 타입 | 서버 | 클라이언트 | 목표 | 피드백 | 결과 |
|-----------|------|------|-----------|------|--------|------|
| `/execute_action` | `vla_interfaces/ExecuteAction` | `arm_controller_node` | `task_manager_node` | action + target + position | `status` (현재 동작 단계) | `success` + `message` |
| `MoveGroup` | `moveit_msgs/MoveGroup` | MoveIt move_group | `arm_controller_node` (pymoveit2) | 목표 포즈 / 관절 설정 | 경로 계획 진행 상태 | 실행 성공 여부 |

> **Action을 선택한 이유**: pick/place는 수 초 이상 걸리는 장시간 작업. 실행 중 피드백(`"그리퍼 열기"`, `"하강"` 등)을 단계별로 받을 수 있고, 비상정지 시 `cancel_execution()`으로 중단 가능. Service는 응답 전까지 블로킹되고 취소 불가능.

#### TF (좌표 변환 트리)

```
world
  └── panda_link0  (= base_link, 모든 좌표의 기준)
        ├── panda_link1
        │    └── ... (panda_link2~7)
        │         └── panda_hand (엔드이펙터)
        └── camera_link  (고정 joint, xyz="0.4 0.0 0.9" rpy="0 1.1 0")
              └── camera_optical_frame  (고정 joint, rpy="-1.5708 0 -1.5708")
                  ※ 카메라 표준 축: z=전방, x=우측, y=하방
```

`perception_node`는 깊이 이미지로 얻은 3D 좌표(`camera_optical_frame` 기준)를 `tf_buffer.lookup_transform()`으로 `panda_link0` 기준으로 변환해 모든 노드가 공통 좌표계를 사용하도록 합니다.

---

### ROS 인터페이스 정의 (vla_interfaces)

```
# msg/DetectedObject.msg
string name                       # 물체 이름 (snake_case, 예: red_block)
geometry_msgs/Point position      # panda_link0 기준 3D 좌표 (미터)

# srv/DetectObjects.srv
string target                     # 찾을 물체명 ("" = KNOWN_OBJECTS 전체)
---
DetectedObject[] objects          # 감지된 물체 목록

# srv/PlanTask.srv
string instruction                # 사용자 자연어 명령
string world_state_json           # JSON 문자열: [{"name":"..."}]
string robot_state_json           # JSON 문자열: {"gripper":"open","holding":null}
---
bool success                      # 플랜 생성 성공 여부
string plan_json                  # JSON 문자열: {"plan":[{"action":"...","target":"..."}]}

# action/ExecuteAction.action
string action                     # pick|place|open_gripper|close_gripper|move_home
string target                     # 대상 물체명
geometry_msgs/Point position      # 목표 3D 좌표
---
bool success                      # 실행 성공 여부
string message                    # 결과 메시지 (오류 설명 포함)
---
string status                     # 피드백: 현재 실행 중인 단계명
```

---

### 데이터 포맷 약속

#### `world_state_json`
```json
[
  {"name": "red block",  "position": {"x": 0.45, "y":  0.15, "z": 0.43}},
  {"name": "blue plate", "position": {"x": 0.55, "y": -0.20, "z": 0.41}}
]
```
- **생산**: `perception_node` — Gemini가 실제로 본 물체만 포함, 하드코딩 없음
- **소비**: `task_manager_node` (world 구성) → `llm_planner_node` (플랜 생성 컨텍스트)

#### `plan_json`
```json
{"plan": [
  {"action": "open_gripper",  "target": ""},
  {"action": "pick",          "target": "red block"},
  {"action": "place",         "target": "blue plate"},
  {"action": "close_gripper", "target": ""},
  {"action": "move_home",     "target": ""}
]}
```
- **생산**: `llm_planner_node` — Gemini + validate_plan() 검증 통과 후
- **소비**: `task_manager_node` → 스텝별 `/execute_action` 목표로 변환

#### 물체 이름 규칙
- 소문자 `snake_case`, 띄어쓰기 금지 (단, 서비스 응답에서는 공백 허용)
- 권장 형식: `색_종류` (예: `red_block`, `green_cup`, `blue_plate`)
- 사용자 자연어("녹색 컵") → LLM이 `green_cup`으로 매칭

#### 좌표 기준
- 모든 `position`은 로봇 베이스 `panda_link0` 기준, 단위: **미터(m)**
- 작업 가능 범위: x(0.2~0.7m), y(-0.4~0.4m), z(0.0~0.6m)

#### 허용 액션 목록 (5개 고정)
```
pick  |  place  |  open_gripper  |  close_gripper  |  move_home
```
`llm_planner_node`의 `VALID_ACTIONS` 집합 및 `PLAN_SCHEMA`의 `enum`으로 강제되어 다른 단어(`grab` 등)는 생성·실행 불가.

---

## 📁 최종 워크스페이스 구조

```
vla_arm_ws/
├── src/
│   ├── pymoveit2/                          # 외부: Python MoveIt2 라이브러리
│   ├── vla_interfaces/
│   │   ├── msg/DetectedObject.msg
│   │   ├── srv/DetectObjects.srv
│   │   ├── srv/PlanTask.srv
│   │   ├── action/ExecuteAction.action
│   │   ├── CMakeLists.txt
│   │   └── package.xml
│   ├── arm_bringup/
│   │   ├── worlds/scene.world
│   │   ├── config/controllers.yaml
│   │   ├── launch/gazebo.launch.py
│   │   ├── urdf/panda_gazebo.urdf.xacro   # ros2_control + RGB-D 카메라 추가
│   │   ├── setup.py
│   │   └── package.xml
│   ├── arm_moveit_config/                  # MoveIt Setup Assistant 생성
│   ├── vla_perception/
│   │   └── vla_perception/perception_node.py
│   ├── vla_planner/
│   │   └── vla_planner/llm_planner_node.py
│   ├── vla_control/
│   │   └── vla_control/arm_controller_node.py
│   ├── vla_orchestrator/
│   │   └── vla_orchestrator/
│   │       ├── task_manager_node.py
│   │       └── safety_node.py
│   └── vla_bringup/
│       └── launch/demo.launch.py
├── build/    # colcon 자동 생성
├── install/  # colcon 자동 생성
└── log/      # colcon 자동 생성
```

---

## ⚙️ 설치 및 실행

### 1. 의존성 설치
```bash
source /opt/ros/humble/setup.bash
sudo apt update
sudo apt install -y \
  ros-humble-gazebo-ros-pkgs ros-humble-gazebo-ros \
  ros-humble-gazebo-ros2-control ros-humble-ros2-control \
  ros-humble-ros2-controllers ros-humble-moveit \
  ros-humble-cv-bridge ros-humble-tf2-ros \
  ros-humble-tf2-geometry-msgs ros-humble-xacro \
  python3-colcon-common-extensions

pip install google-genai pillow numpy
```

### 2. API 키 설정
```bash
export GEMINI_API_KEY="발급받은_키"
# 영구 적용:
echo 'export GEMINI_API_KEY="발급받은_키"' >> ~/.bashrc
```

### 3. 워크스페이스 구성
```bash
mkdir -p ~/vla_arm_ws/src
cd ~/vla_arm_ws/src
git clone https://github.com/AndrejOrsula/pymoveit2.git
git clone <이_저장소_URL>
```

### 4. Panda 팔 패키지 추가
```bash
cd /tmp
git clone --depth 1 -b humble https://github.com/ros-planning/moveit_resources.git
cp -r /tmp/moveit_resources/panda_description ~/vla_arm_ws/src/
cp -r /tmp/moveit_resources/panda_moveit_config ~/vla_arm_ws/src/
rm -rf /tmp/moveit_resources
```

### 5. 빌드
```bash
cd ~/vla_arm_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

### 6. 실행

**통합 런치 (권장)**
```bash
ros2 launch vla_bringup demo.launch.py
```

**명령 발행**
```bash
ros2 topic pub --once /user_command std_msgs/msg/String \
  '{"data": "빨간 블록을 파란 접시에 놓아줘"}'
```

---

## 🔁 개발 반복 패턴

코드를 수정할 때마다 아래 흐름을 따릅니다:

```bash
# 1. 파일 수정
# 2. (신규 노드라면) setup.py entry_points에 등록
# 3. 빌드
cd ~/vla_arm_ws && colcon build --symlink-install
# 4. 소싱 (새 터미널 열 때마다, 빌드 후마다)
source install/setup.bash
# 5. 실행/검증
ros2 run <패키지> <노드>
```

> `--symlink-install` 옵션을 사용하면 `.py` 파일만 수정했을 때 재빌드 없이 즉시 반영됩니다.

---

## 🛡️ 핵심 설계 원칙

1. **형식(모양) 고정, 값(내용) 동적** — 인터페이스 약속만 지키면 월드(물체 색·개수)가 바뀌어도 코드 수정 불필요
2. **물체를 코드에 하드코딩하지 않는다** — 모든 물체 정보는 런타임에 perception이 채움
3. **Open-vocabulary 인식** — 미리 정한 목록이 아니라 카메라에 보이는 물체를 모두 탐지
4. **단일 좌표계** — 모든 위치는 `panda_link0` 기준 미터(m)
5. **5개 액션만 허용** — `pick`, `place`, `open_gripper`, `close_gripper`, `move_home`

---

## 🚨 트러블슈팅

| 증상 | 원인 / 해결 |
|------|------------|
| `ros2 run`이 패키지를 못 찾음 | 해당 터미널에서 `source install/setup.bash` 누락 |
| 인터페이스 import 에러 | `vla_interfaces`를 먼저 빌드/소싱했는지 확인 |
| 팔이 안 움직임 | `ros2 control list_controllers`로 컨트롤러 active 여부 확인. MoveIt 설정과 `controllers.yaml` 이름 일치 여부 확인 |
| 좌표가 엉뚱함 | 카메라 광학 프레임 회전(`-1.5708, 0, -1.5708`), 깊이 단위(m), `panda_link0`까지 TF 연결 확인 |
| 카메라 토픽 없음 | URDF 카메라 블록 삽입 여부 확인. `ros2 topic list`로 실제 토픽명 확인 후 `perception_node` 수정 |
| Gemini 호출 실패 | `GEMINI_API_KEY` 환경변수 설정 확인 |
| 비상정지 트리거 | `ros2 service call /set_estop std_srvs/srv/SetBool "{data: true}"` → `{data: false}`로 해제 |

---

## 📋 단계별 진행 요약

```
설치 → 워크스페이스 → 패키지 7개 생성 → 인터페이스 정의/빌드
→ Gazebo 시뮬 환경 → MoveIt 연동 및 모션 검증
→ perception_node → llm_planner_node → arm_controller_node
→ safety_node → task_manager_node (통합)
→ 통합 런치 → 견고성·안전 점검 → 데모/문서
```

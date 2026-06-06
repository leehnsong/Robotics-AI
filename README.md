# VLA Robot Arm — Franka Panda in Gazebo (ROS 2 Humble)

> 자연어 명령으로 로봇 팔을 제어하는 **VLA(Vision-Language-Action)** 시스템 구축 프로젝트.
> 이 브랜치(`vla-arm-gazebo`)는 그 **토대(Step 5)** — Franka Panda 팔 + RGB-D 카메라가 있는
> Gazebo Classic 시뮬레이션 무대를 만들고, 관절을 ROS 2 컨트롤러로 실제로 움직이는 단계까지를 담는다.

---

## 0. 현재 상태 (무엇이 동작하는가)

| 항목 | 상태 |
|------|------|
| Gazebo Classic 에 Panda 팔 + 테이블/물체 스폰 | ✅ |
| 팔 7관절 위치 제어 (`arm_controller`, JointTrajectoryController) | ✅ |
| 관절 상태 발행 (`joint_state_broadcaster` → `/joint_states`) | ✅ |
| 궤적 명령으로 실제 팔 이동 (검증됨) | ✅ |
| RGB-D 카메라 (`/camera/image_raw`, `/camera/depth/image_raw`, `/camera/points`) | ✅ |
| 그리퍼(panda_finger) 제어 | ⏳ 미구현 (다음 단계) |
| MoveIt 경로계획 / VLA 노드(인식·계획·제어) | ⏳ 미구현 (다음 단계) |

---

## 1. 프로젝트 개요

최종 목표는 다음과 같은 흐름이다:

```
사람: "빨간 블록을 파란 접시에 올려"
        ↓
[vla_orchestrator] 총괄 지휘
   ├→ [vla_perception] 카메라로 물체 인식 (어디에 뭐가 있나)
   ├→ [vla_planner]    명령 → 행동 플랜 수립   (vla_interfaces/PlanTask 사용)
   └→ [vla_control]    플랜대로 팔 제어 → MoveIt/pymoveit2 → Gazebo 안 Panda 팔
```

이 브랜치는 위 그림의 **무대(Gazebo + 팔 + 카메라)** 를 만드는 부분까지 완성한 상태다.
인식/계획/제어 노드(`vla_*`)는 아직 빈 스텁이며 이후 채운다.

### 왜 Franka Panda + Gazebo Classic 인가
- 워크스페이스의 `pymoveit2` 라이브러리가 **panda 기준**이고, `moveit_resources` 의 panda 네이밍
  (`panda_joint1~7`, `panda_link0~8`, `panda_hand`, `panda_finger_joint1/2`)이 그대로 일치 → 이름 맞추기 작업 최소화.
- panda 는 그리퍼가 내장돼 있어 집기 데모로 확장하기 좋다.
- ROS 2 Humble 의 기본 시뮬레이터인 **Gazebo Classic 11** 사용 (Ignition/신형 Gazebo 아님).

---

## 2. 시스템 요구사항

- **Ubuntu 22.04** + **ROS 2 Humble**
- **Gazebo Classic 11** (`gazebo`, `gzserver`)
- 아래 ROS 패키지 (대부분 `ros-humble-desktop` 에 포함):

```bash
sudo apt install \
  ros-humble-gazebo-ros-pkgs ros-humble-gazebo-ros2-control \
  ros-humble-ros2-control ros-humble-ros2-controllers \
  ros-humble-joint-state-broadcaster ros-humble-joint-trajectory-controller \
  ros-humble-xacro ros-humble-robot-state-publisher \
  ros-humble-moveit   # (MoveIt 단계 대비)
```

---

## 3. 빠른 시작 (Quick Start)

### 3-1. 이 저장소 받기
```bash
git clone -b vla-arm-gazebo https://github.com/leehnsong/Robotics-AI.git
cd Robotics-AI/vla_arm_ws
```

### 3-2. 외부 팔 패키지 받기 (저작권상 이 repo 엔 미포함)
Panda 의 URDF/메쉬/MoveIt 설정은 `moveit_resources` 에서 받는다. **panda 네이밍이 pymoveit2 와 일치**하므로 이걸 쓴다.
```bash
cd src
git clone --depth 1 -b humble https://github.com/ros-planning/moveit_resources.git /tmp/mr
cp -r /tmp/mr/panda_description /tmp/mr/panda_moveit_config .   # src/ 로 복사
rm -rf /tmp/mr
# (미래 단계용 — MoveIt/pymoveit2 제어에 사용. 지금 단계 빌드엔 필수 아님)
git clone https://github.com/AndrejOrsula/pymoveit2.git
cd ..
```
> 폴더명은 `panda_description` 이지만 **패키지명은 `moveit_resources_panda_description`** 이다 (헷갈리지 말 것).

### 3-3. 빌드 (반드시 워크스페이스 루트 `vla_arm_ws/` 에서)
```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

### 3-4. 실행 (한 줄)
```bash
ros2 launch arm_bringup spawn_arm.launch.py
```
잠시 후 Gazebo 창에 **흰색 Panda 팔 + 테이블 + 빨간 블록 + 파란 접시**가 보이고,
런치 로그 끝에 다음이 떠야 정상:
```
Configured and activated joint_state_broadcaster
Configured and activated arm_controller
```
> GUI 없이(헤드리스) 돌리려면: `ros2 launch arm_bringup spawn_arm.launch.py gui:=false`

---

## 4. 동작 검증

새 터미널을 열고:
```bash
cd Robotics-AI/vla_arm_ws && source install/setup.bash

# 1) 컨트롤러 둘 다 active 인가
ros2 control list_controllers

# 2) 관절 상태가 흐르는가 (panda_joint1~7)
ros2 topic echo /joint_states

# 3) 카메라 토픽이 있는가
ros2 topic list | grep camera     # /camera/image_raw, /camera/depth/image_raw, /camera/points

# 4) 팔을 실제로 움직여 보기 (joint1→0.5, joint4→-1.5, joint6→1.0 으로 이동)
ros2 topic pub --once /arm_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory \
'{joint_names: [panda_joint1,panda_joint2,panda_joint3,panda_joint4,panda_joint5,panda_joint6,panda_joint7],
  points: [{positions: [0.5,0.0,0.0,-1.5,0.0,1.0,0.0], time_from_start: {sec: 2}}]}'

# 5) 카메라 영상 확인
ros2 run rqt_image_view rqt_image_view
```

---

## 5. 단계별 상세 (무엇을, 왜)

### Step 5-1. 벤더 팔 패키지 받기
임의의 팔 URDF 를 직접 짓지 않고 공식 패키지를 받는다 → `moveit_resources` 의
`panda_description`(URDF/메쉬/그리퍼) + `panda_moveit_config`(MoveIt 설정). (위 3-2)

### Step 5-2. Gazebo world — [`arm_bringup/worlds/scene.world`](vla_arm_ws/src/arm_bringup/worlds/scene.world)
pick-and-place 용 무대: 테이블(고정) + 빨간 블록(움직임, 집을 대상) + 파란 접시(고정, 목적지).
- 빨간 블록에 **마찰(`mu=1.0`)** 부여 → 그리퍼가 미끄러지지 않게.
- `world` 파일은 "무대에 뭐가 있는지"만 정의한다. **로봇이 무엇을 할지는 VLA 코드가 결정**하므로
  같은 무대에서 다양한 시연이 가능하다 (빨강→파랑 전용이 아님).

### Step 5-3. Gazebo 연결 URDF — [`arm_bringup/urdf/panda_gazebo.urdf.xacro`](vla_arm_ws/src/arm_bringup/urdf/panda_gazebo.urdf.xacro)
순정 panda 를 **수정하지 않고 include** 한 뒤, Gazebo 구동에 필요한 것들을 얹는 wrapper:
1. **`ros2_control` + `gazebo_ros2_control` 플러그인** — 7관절을 위치 제어로 노출 (`command/state interface`).
2. **RGB-D 카메라** — `camera_link` + 영상 표준축 `camera_optical_frame` + depth 센서 플러그인.
3. **world 고정 조인트** — 베이스를 world 에 고정 (안 하면 로봇이 떨어짐).
4. **관성 주입** — 원본 panda.urdf 는 질량/관성이 없어 Gazebo 가 관절을 날린다.
   [`scripts/generate_panda_inertia.py`](vla_arm_ws/src/arm_bringup/scripts/generate_panda_inertia.py) 로
   각 링크에 관성을 넣은 [`panda_inertia.urdf`](vla_arm_ws/src/arm_bringup/urdf/panda_inertia.urdf) 를 생성해 사용.
5. **시각 머티리얼** — visual 메쉬에 색이 없어 화면에 안 보이므로 `<gazebo><material>` 로 색 부여.

### Step 5-4. 컨트롤러 설정 — [`arm_bringup/config/controllers.yaml`](vla_arm_ws/src/arm_bringup/config/controllers.yaml)
- `joint_state_broadcaster` : 관절 상태를 `/joint_states` 로 발행 (읽기 전용).
- `arm_controller` (JointTrajectoryController) : 궤적 명령을 받아 7관절을 실제로 움직임.
- ⚠️ 여기 `joints` 목록은 URDF 의 `<ros2_control>` 관절 이름과 **정확히 일치**해야 한다 (`panda_joint1~7`).

### Step 5-5. 통합 런치 — [`arm_bringup/launch/spawn_arm.launch.py`](vla_arm_ws/src/arm_bringup/launch/spawn_arm.launch.py)
가이드의 "4개 터미널"을 하나로 묶고, 올바른 순서를 보장:
`xacro 처리 → Gazebo+world → robot_state_publisher → spawn_entity → (스폰 후) jsb → arm_controller`.

---

## 6. 패키지 구조

```
vla_arm_ws/src/
├── arm_bringup/            ⭐ 팔 + Gazebo 실행 (이 단계의 핵심)
│   ├── worlds/scene.world          # 무대
│   ├── urdf/panda_gazebo.urdf.xacro # 래퍼(본체 include + 제어 + 카메라 + world고정 + 색)
│   ├── urdf/panda_inertia.urdf      # 관성 주입된 panda (생성물)
│   ├── scripts/generate_panda_inertia.py  # 위 파일 생성 스크립트
│   ├── config/controllers.yaml      # 컨트롤러 설정
│   └── launch/spawn_arm.launch.py   # 통합 런치
├── vla_interfaces/         # 노드 간 데이터 정의 (srv/PlanTask, srv/DetectObjects, msg/DetectedObject, action/ExecuteAction)
├── vla_perception/         # 인식  (스텁)
├── vla_planner/            # 계획  (스텁)
├── vla_control/            # 제어  (스텁)
├── vla_orchestrator/       # 총괄  (스텁)
└── vla_bringup/            # 통합 launch (스텁)
```
외부 의존(직접 받음): `moveit_resources_panda_description`, `moveit_resources_panda_moveit_config`, `pymoveit2`.

---

## 7. 트러블슈팅 — 실제로 겪은 문제와 해결

Gazebo 에 팔을 띄우기까지 만난 문제들(가이드엔 없던 실전 이슈). 같은 증상 만나면 참고:

| # | 증상 | 원인 | 해결 |
|---|------|------|------|
| 1 | `controller_manager` 가 안 뜸 (`Couldn't parse parameter override rule`) | robot_description 의 **XML 주석**이 gazebo_ros2_control 의 CLI 파라미터 전달을 깨뜨림 ([gazebo_ros2_control#295](https://github.com/ros-controls/gazebo_ros2_control/issues/295)) | 런치에서 `xacro` 처리 후 **XML 주석 제거** |
| 2 | `FrameAttachedToGraph` SDF 에러 | 빈(질량 없는) 카메라 링크 | 카메라 링크에 최소 `<inertial>` 부여 |
| 3 | gzserver 멈춤 (`Waiting for model database update`) | `model://sun` 등이 **온라인 모델 DB** 접속 시도 | 런치에서 `GAZEBO_MODEL_DATABASE_URI=""` |
| 4 | **관절 7개 전부** `Skipping joint ... not in the gazebo model` | moveit_resources panda.urdf 에 **질량/관성이 0개** (MoveIt 전용) → Gazebo 가 관절 제거 | `generate_panda_inertia.py` 로 관성 주입 |
| 5 | 로봇이 자유낙하 | base 가 world 에 안 묶임 | `world` 링크 + 고정 조인트 추가 |
| 6 | 팔이 화면에 안 보임 (관절·물리는 정상) | visual `.dae` 메쉬에 **`<material>` 없음** → 렌더 안 됨 (collision 은 보임) | `<gazebo reference><material>` 로 각 링크 색 지정 |

> 진단 팁: 메쉬 렌더 문제는 `~/.gazebo/ogre.log` 에서 메쉬 리소스 로딩/에러를 확인할 수 있다.
> `ros2 control list_controllers` 가 `active` 면 로봇은 물리적으로 정상 — 안 보이면 보통 **표시(머티리얼/뷰모드)** 문제다.

---

## 8. 다음 단계 (Roadmap)

- [ ] 그리퍼(`panda_finger_joint1/2`) 제어 추가 → 집기 데모
- [ ] MoveIt(`panda_moveit_config` move_group) + `pymoveit2` 연동 → "좌표만 주면 경로계획"
- [ ] `vla_perception` — 카메라로 물체 인식
- [ ] `vla_planner` — 자연어 명령 → 행동 플랜 (`PlanTask`)
- [ ] `vla_control` / `vla_orchestrator` — 전체 파이프라인 통합

---

*ROS 2 Humble · Gazebo Classic 11 · Franka Panda*

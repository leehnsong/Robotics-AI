# vla_perception

VLA 파이프라인의 **인식(눈)** 패키지. 카메라 영상을 보고 **테이블 위 물체들의 이름 + 3D 위치**를 알아내
`/detect_objects` 서비스로 제공한다.

- VLM: **`gemini-robotics-er-1.6-preview`** (로봇 전용, 물체를 **점**으로 가리킴)
- **open-vocabulary**: 미리 정한 목록 없이 보이는 물체를 다 찾음 → 어떤 월드(색·개수 달라도)에서도 동작
- 좌표 기준: **`panda_link0`**, 이름: **snake_case `색_종류`** (팀 CONVENTIONS 준수)

---

## 동작 원리 (4단계)

```
RGB 이미지 ─▶ ① Gemini(robotics-er): "물체를 점으로 가리켜" ─▶ 픽셀 위치 [{name, (u,v)}]
Depth 이미지 ─┐
CameraInfo  ─┼▶ ② 핀홀 공식: 픽셀(u,v) + 깊이 d + 렌즈(fx,fy,cx,cy) ─▶ 카메라기준 3D
            │
TF          ─▶ ③ 카메라기준 → panda_link0 기준 변환
                                                        ─▶ ④ DetectedObject{name, position(x,y,z)}
```

| 입력 토픽 | 내용 |
|-----------|------|
| `/camera/image_raw` | 컬러 이미지 (Gemini 입력) |
| `/camera/depth/image_raw` | 깊이(거리) 이미지 |
| `/camera/camera_info` | 렌즈 정보 (fx,fy,cx,cy) |

| 출력 (서비스) | 내용 |
|---------------|------|
| `/detect_objects` (`vla_interfaces/srv/DetectObjects`) | `target` → `DetectedObject[] {name, position}` |

> `target` 비우면 모든 물체, 특정 이름 주면 그 물체만 필터.

---

## 사전 준비

```bash
# 1) 라이브러리
pip install google-genai
#   (cv_bridge, tf2_ros, tf2_geometry_msgs, Pillow, numpy 는 ROS humble desktop 에 포함)

# 2) Gemini API 키 (https://aistudio.google.com/apikey 에서 발급)
echo 'export GEMINI_API_KEY="<발급받은키>"' >> ~/.bashrc
source ~/.bashrc
```

## 빌드

```bash
cd ~/vla_arm_ws
colcon build --symlink-install --packages-select vla_interfaces vla_perception
source install/setup.bash
```

## 실행

```bash
# (터미널1) 시뮬 - 카메라/TF 제공
ros2 launch arm_bringup spawn_arm.launch.py

# (터미널2) perception 노드
ros2 run vla_perception perception_node

# (터미널3) 물체 검출 요청
ros2 service call /detect_objects vla_interfaces/srv/DetectObjects "{target: ''}"
```

기대 출력:
```
objects:
  - name: red_block,  position: {x: 0.45, y: 0.15, z: 0.45}
  - name: blue_plate, position: {x: 0.55, y: -0.20, z: 0.42}
```

---

## 파라미터

| 파라미터 | 기본값 | 설명 |
|----------|--------|------|
| `model` | `gemini-robotics-er-1.6-preview` | 사용할 VLM |
| `base_frame` | `panda_link0` | 출력 좌표 기준 프레임 |
| `use_mock` | `false` | true 면 API 없이 이미지 중앙에 가짜 물체 1개 (파이프라인 테스트용) |
| `rgb_topic` / `depth_topic` / `info_topic` | `/camera/...` | 카메라 토픽 이름 |

예) API 키 없이 좌표변환만 테스트:
```bash
ros2 run vla_perception perception_node --ros-args -p use_mock:=true
```

---

## 테스트 체크리스트

| # | 명령 | 기대 |
|---|------|------|
| 1 | `target: ''` | 모든 물체 + 좌표 |
| 2 | `target: 'red_block'` | 해당 물체만 |
| 3 | 좌표 vs scene.world | ±2~3cm |
| 4 | Gazebo에서 물체 드래그 후 재검출 | 좌표가 따라 바뀜 (진짜 인식 증명) |
| 5 | `use_mock:=true` | mock_object z≈0.4 |

---

## 다른 팀원에게 (인터페이스)

이 패키지를 쓰려면 **`/detect_objects` 서비스만** 호출하면 된다. 내부 구현(어떤 VLM, 점/박스 등)은
신경 쓸 필요 없다 — 출력은 항상 `DetectedObject{name(snake_case), position(panda_link0 기준 x,y,z)}`.

```python
# 예: 다른 노드에서 호출
client = self.create_client(DetectObjects, '/detect_objects')
req = DetectObjects.Request(target='')      # 또는 'red_block'
# ... call → resp.objects -> [{name, position}, ...]
```

---

## 트러블슈팅

| 증상 | 확인 |
|------|------|
| "이미지/깊이/카메라정보 아직 안 들어옴" | 시뮬(터미널1)이 떴나? `ros2 topic hz /camera/image_raw` |
| 응답이 비어있음 | 로그의 `VLM 원본 응답` 확인 (Gemini가 뭘 줬나) |
| TF 실패 | robot_state_publisher 떴나 (런치에 포함됨), `base_frame` 이 `panda_link0` 인지 |
| 좌표가 이상함 | `base_frame` 확인, 깊이 토픽 정상인지 |
| API 오류 | `echo $GEMINI_API_KEY` 로 키 확인 |

> 디버그: 노드 로그에 `VLM 원본 응답`과 `VLM 검출 물체 수`가 찍힌다. 검출 문제 진단의 핵심.

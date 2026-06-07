#!/usr/bin/env python3
"""
perception_node.py  (open-vocabulary 버전)

역할: 카메라 이미지를 VLM(Gemini)에게 보내 "테이블 위의 모든 물체"를 찾고(open-vocabulary),
      각 물체의 2D 박스를 깊이(depth)로 3D 좌표(panda_link0 기준)로 변환해
      DetectObjects 서비스로 제공한다.

규칙(팀 CONVENTIONS 준수):
  - 출력 좌표 기준: panda_link0 (파라미터 base_frame)
  - 물체 이름: 소문자 snake_case, 가능하면 color_type (red_block, green_cup ...)
  - 하드코딩 목록 없음 → 어떤 월드(색·개수 달라도)에서도 동작

파라미터:
  use_mock(bool, 기본 False): True 면 Gemini 대신 이미지 중앙에 가짜 물체 1개를 반환
                              → API 키 없이 깊이/TF/좌표변환 파이프라인 테스트용
  model, base_frame, rgb_topic, depth_topic, info_topic

실행 전: export GEMINI_API_KEY=...   (use_mock:=true 면 불필요)
"""
import json
import re
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PointStamped, Point
from cv_bridge import CvBridge
from tf2_ros import Buffer, TransformListener
from tf2_geometry_msgs import do_transform_point
from PIL import Image as PILImage
from vla_interfaces.srv import DetectObjects
from vla_interfaces.msg import DetectedObject


def to_snake(name: str) -> str:
    """'Red Block' / 'red-block' -> 'red_block' (소문자 snake_case)"""
    s = re.sub(r"[^a-z0-9]+", "_", name.strip().lower())
    return s.strip("_")


class PerceptionNode(Node):
    def __init__(self):
        super().__init__("perception_node")

        self.declare_parameter("model", "gemini-robotics-er-1.6-preview")
        self.declare_parameter("base_frame", "panda_link0")
        self.declare_parameter("rgb_topic", "/camera/image_raw")
        self.declare_parameter("depth_topic", "/camera/depth/image_raw")
        self.declare_parameter("info_topic", "/camera/camera_info")
        self.declare_parameter("use_mock", False)

        g = lambda k: self.get_parameter(k).value
        self.model = g("model")
        self.base_frame = g("base_frame")
        self.use_mock = g("use_mock")

        self.bridge = CvBridge()
        self.rgb = None
        self.depth = None
        self.K = None              # (fx, fy, cx, cy)
        self.optical_frame = None

        # VLM 클라이언트 (mock 모드면 생략)
        self.client = None
        if not self.use_mock:
            from google import genai   # GEMINI_API_KEY 환경변수 사용
            self.client = genai.Client()

        self.create_subscription(Image, g("rgb_topic"), self._rgb_cb, 10)
        self.create_subscription(Image, g("depth_topic"), self._depth_cb, 10)
        self.create_subscription(CameraInfo, g("info_topic"), self._info_cb, 10)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.create_service(DetectObjects, "/detect_objects", self._on_detect)
        mode = "MOCK" if self.use_mock else f"Gemini({self.model})"
        self.get_logger().info(f"perception_node 준비 완료 (mode={mode}, base={self.base_frame})")

    # ---- 콜백: 최신 이미지/깊이/카메라정보 저장 ----
    def _rgb_cb(self, m):
        self.rgb = self.bridge.imgmsg_to_cv2(m, "rgb8")

    def _depth_cb(self, m):
        self.depth = self.bridge.imgmsg_to_cv2(m, "32FC1")   # 미터 단위

    def _info_cb(self, m):
        self.K = (m.k[0], m.k[4], m.k[2], m.k[5])            # fx, fy, cx, cy
        self.optical_frame = m.header.frame_id

    # ---- VLM: 모든 물체 "점(point)"으로 검출 (open-vocabulary) ----
    def _detect_all(self):
        """[{'name': str, 'uv': (u, v)}, ...]  (u,v = 이미지 픽셀 좌표)
        gemini-robotics-er 는 물체를 점 [y, x](0~1000 정규화)으로 가리킨다."""
        H, W = self.depth.shape
        if self.use_mock:
            # API 키 없이 파이프라인 테스트: 이미지 중앙에 가짜 물체 1개
            return [{"name": "mock_object", "uv": (W // 2, H // 2)}]

        from google.genai import types
        pil = PILImage.fromarray(self.rgb)
        prompt = (
            "Point to every distinct object on the table in this image "
            "(blocks, plates, cups, bowls, bins — include flat objects like plates too). "
            'Output ONLY a JSON list: [{"point": [y, x], "label": "name"}] '
            "where point is [y, x] normalized to 0-1000. "
            "Use lowercase snake_case color_type labels (e.g. red_block, blue_plate, green_cup)."
        )
        cfg = types.GenerateContentConfig(response_mime_type="application/json")
        r = self.client.models.generate_content(
            model=self.model, contents=[pil, prompt], config=cfg)
        self.get_logger().info(f"VLM 원본 응답: {r.text[:400]}")   # 디버그: 실제로 뭘 줬나
        data = json.loads(r.text)
        # robotics-er 는 보통 최상위 리스트. 혹시 dict 면 안의 리스트를 찾는다.
        items = data if isinstance(data, list) else (
            data.get("points") or data.get("objects") or [])

        out = []
        for it in items:
            pt = it.get("point")
            name = it.get("label") or it.get("name") or ""
            if not (isinstance(pt, list) and len(pt) == 2 and name):
                continue
            y, x = pt
            u = min(max(int(x / 1000 * W), 0), W - 1)
            v = min(max(int(y / 1000 * H), 0), H - 1)
            out.append({"name": name, "uv": (u, v)})
        self.get_logger().info(f"VLM 검출 물체 수: {len(out)}")
        return out

    # ---- 픽셀(u,v) + 깊이 -> panda_link0 기준 3D 좌표 ----
    def _pixel_to_base(self, u, v):
        # 점 주변 5x5 깊이의 중앙값 (노이즈 완화)
        patch = self.depth[max(0, v - 2):v + 3, max(0, u - 2):u + 3].astype(float)
        d = float(np.nanmedian(patch))
        if not np.isfinite(d) or d <= 0:
            return None

        fx, fy, cx, cy = self.K
        p = PointStamped()
        p.header.frame_id = self.optical_frame
        p.point.x = (u - cx) * d / fx
        p.point.y = (v - cy) * d / fy
        p.point.z = d
        try:
            tf = self.tf_buffer.lookup_transform(
                self.base_frame, self.optical_frame, rclpy.time.Time())
        except Exception as e:
            self.get_logger().warn(f"TF 실패 ({self.base_frame}<-{self.optical_frame}): {e}")
            return None
        pb = do_transform_point(p, tf)
        return (pb.point.x, pb.point.y, pb.point.z)

    # ---- 서비스 콜백 ----
    def _on_detect(self, req, resp):
        if self.rgb is None or self.depth is None or self.K is None:
            self.get_logger().warn("이미지/깊이/카메라정보 아직 안 들어옴")
            return resp
        try:
            detections = self._detect_all()
        except Exception as e:
            self.get_logger().warn(f"VLM 검출 오류: {e}")
            return resp

        want = to_snake(req.target) if req.target else None
        for det in detections:
            name = to_snake(det.get("name", ""))
            if not name:
                continue
            if want and want not in name:     # 특정 target 요청 시 필터
                continue
            u, v = det["uv"]
            xyz = self._pixel_to_base(u, v)
            if xyz is None:
                self.get_logger().warn(f"  {name}: 픽셀→3D 변환 실패(깊이/TF) → 제외됨")
                continue
            resp.objects.append(DetectedObject(
                name=name, position=Point(x=xyz[0], y=xyz[1], z=xyz[2])))
            self.get_logger().info(f"  {name} -> ({xyz[0]:.3f}, {xyz[1]:.3f}, {xyz[2]:.3f})")
        self.get_logger().info(f"검출 {len(resp.objects)}개 반환")
        return resp


def main():
    rclpy.init()
    node = PerceptionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

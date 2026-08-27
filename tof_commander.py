"""
tof_commander.py -- ESP32 ToF 서보 제어 + 물체 크기 산출 (라즈베리파이/노트북 쪽)

[역할]
  OpenCV/YOLO가 잡은 물체의 화면 좌표를 각도로 바꿔서 ESP32에 "그 각도를 겨냥해라"
  라고 보내고, 돌아온 거리로 실제 크기를 계산한다.

[왜 이렇게 하나]
  ToF(VL53L0X)는 FoV가 약 25도인 '원뿔'로 측정한다. 1m 거리면 측정 스팟이 44cm라
  물체 테두리를 직접 겨냥해도 절반은 물체, 절반은 배경에 걸쳐 값이 튄다.
  반면 카메라는 1픽셀이 약 0.3도라 테두리 '각도'는 압도적으로 정확하다.
  => 각도는 카메라가, 거리는 ToF가 담당한다.

     폭 = 2 x 거리 x tan(각폭/2)

[사용 예]
    tof = TofCommander("192.168.137.55")
    tof.connect()
    res = tof.measure_object(x1=210, x2=390)     # YOLO bbox의 좌우 픽셀
    print(res)   # {'distance_mm':612, 'width_mm':358, 'angle_deg':-4.2, ...}
"""

import json
import math
import socket
import threading
import queue
import time

# ===============================
# 파라미터 -- robot_config 한 곳에서 관리
# ===============================
import robot_config as C

IMG_W = C.IMG_W
FOCAL_PX = C.focal_px()

CMD_PORT = C.CMD_PORT


def pixel_to_angle(cx_px, img_w=None):
    """이미지 x픽셀 -> 광축 기준 각도(도). 핀홀(atan) 모델."""
    return C.pixel_to_angle(cx_px, img_w)


def width_from_pixels(distance_mm, x1_px, x2_px, img_w=IMG_W):
    """좌/우 테두리 픽셀 + ToF 사거리 -> 실제 폭(mm).

    [주의] ToF 가 주는 건 '사거리'(광선 방향 직선거리)인데,
    크기 계산에 필요한 건 '깊이'(광축 방향 성분)다.
    물체가 화면 가운데서 벗어날수록 둘이 벌어져서,
    2*d*tan(각폭/2) 를 쓰면 최대 20% 넘게 과소평가된다.

        깊이 Z = 사거리 r x cos(중심각)
        폭     = 픽셀폭 x Z / f

    ※ 실제 계산은 robot_config.width_from_box() 에 위임한다. 같은 공식이
      robot_config / vision_object / tof_commander 세 곳에 중복돼 있던 것
      중 하나를 제거한 것 — robot_config 것만 남긴다 (수정지침.md §3-2).
    """
    return C.width_from_box(distance_mm, x1_px, x2_px, img_w=img_w)


class TofCommander:
    def __init__(self, esp32_ip, cmd_port=CMD_PORT, stream_queue=None):
        self.ip = esp32_ip
        self.port = cmd_port
        self.sock = None
        self._next_id = 1
        self._lock = threading.Lock()
        # ESP32 응답은 센서 스트림(9999)으로 돌아온다.
        # socket_receiver 가 그 스트림을 읽으면서 type이 aim/scan 인 것을
        # 이 큐에 넣어주면 여기서 받아 쓴다.
        self.responses = stream_queue if stream_queue is not None else queue.Queue()

    # ---------- 연결 ----------
    def connect(self, timeout=3.0):
        self.sock = socket.create_connection((self.ip, self.port), timeout=timeout)
        self.sock.settimeout(None)
        return True

    def close(self):
        if self.sock:
            try:
                self.sock.close()
            finally:
                self.sock = None

    def _send(self, obj):
        if self.sock is None:
            raise RuntimeError("connect() 를 먼저 호출해야 함")
        line = (json.dumps(obj, separators=(',', ':')) + "\n").encode()
        with self._lock:
            self.sock.sendall(line)

    def _new_id(self):
        self._next_id += 1
        return self._next_id

    # ---------- 기본 명령 ----------
    def sweep(self):
        """연속 스윕(지도 작성 모드)으로 복귀."""
        self._send({"cmd": "sweep"})

    def hold(self, angle_deg):
        """지정 각도에서 정지 유지.

        ★ 현재 .ino 는 "hold" 명령을 모른다 (수정지침.md §3-3, 아두이노 쪽
          미이식). 지금 호출하면 ESP32가 그냥 무시하고 응답도 안 온다.
          .ino 에 hold 처리가 추가되기 전까지는 쓰지 말 것.
        """
        self._send({"cmd": "hold", "angle": round(angle_deg, 1)})

    def aim(self, angle_deg, samples=5, timeout=2.0):
        """지정 각도를 겨냥해 samples회 측정하고 중앙값을 받는다.
        반환: dict 또는 None(응답 없음)
        ※ .ino 에 구현되어 있는 명령. n(samples)은 아직 무시되고
          AIM_SAMPLES 로 고정되어 있다 (수정지침.md §3-3)."""
        rid = self._new_id()
        self._send({"cmd": "aim", "id": rid,
                    "angle": round(angle_deg, 1), "n": samples})
        return self._wait_response(rid, "aim", timeout)

    def scan_range(self, from_deg, to_deg, step_deg=3.0, samples=3, timeout=8.0):
        """구간을 훑으며 각도마다 측정. 반환: [(각도, 거리mm), ...]
        테두리 '위치'를 거리 불연속으로 찾고 싶을 때 쓴다.

        ★ 현재 .ino 는 "scan" 명령을 모른다 (수정지침.md §3-3, 아두이노 쪽
          미이식). 지금은 응답이 절대 안 와서 timeout 초 뒤 빈 리스트만
          반환한다. .ino 에 scan 명령이 추가되기 전까지는 항상 빈 결과다.
          (find_edges_by_scan() 도 내부적으로 이 함수를 쓰므로 마찬가지.)
        """
        rid = self._new_id()
        self._send({"cmd": "scan", "id": rid,
                    "from": round(from_deg, 1), "to": round(to_deg, 1),
                    "step": round(step_deg, 1), "n": samples})
        out = []
        deadline = time.time() + timeout
        while time.time() < deadline:
            try:
                msg = self.responses.get(timeout=max(0.05, deadline - time.time()))
            except queue.Empty:
                break
            if msg.get("id") != rid:
                continue
            if msg.get("type") == "scan_end":
                break
            if msg.get("type") == "scan" and msg.get("ok"):
                out.append((float(msg["angle"]), int(msg["dist"])))
        return out

    def _wait_response(self, rid, want_type, timeout):
        deadline = time.time() + timeout
        while time.time() < deadline:
            try:
                msg = self.responses.get(timeout=max(0.05, deadline - time.time()))
            except queue.Empty:
                return None
            if msg.get("type") == want_type and msg.get("id") == rid:
                return msg
        return None

    # ---------- 물체 크기 측정 ----------
    def measure_object(self, x1, x2, samples=5, timeout=2.5):
        """카메라 bbox의 좌/우 픽셀로 물체 크기를 구한다.

        1) 좌/우 테두리 픽셀 -> 각도 (카메라가 정확)
        2) 중심 각도로 ToF 겨냥 -> 거리 1회 측정 (스팟이 물체 안에 온전히 들어감)
        3) 폭 = 2 x 거리 x tan(각폭/2)
        """
        ang_l = pixel_to_angle(min(x1, x2))
        ang_r = pixel_to_angle(max(x1, x2))
        ang_c = (ang_l + ang_r) / 2.0

        res = self.aim(ang_c, samples=samples, timeout=timeout)
        if res is None or not res.get("ok"):
            return {"ok": False, "reason": "no_response" if res is None else "no_valid_range",
                    "angle_deg": ang_c, "angle_left": ang_l, "angle_right": ang_r}

        dist = int(res["dist"])
        width = width_from_pixels(dist, x1, x2)
        return {
            "ok": True,
            "distance_mm": dist,
            "width_mm": round(width, 1),
            "angle_deg": round(ang_c, 2),
            "angle_left": round(ang_l, 2),
            "angle_right": round(ang_r, 2),
            "angular_width_deg": round(abs(ang_r - ang_l), 2),
            "yaw": res.get("yaw"),
            "samples": res.get("n"),
        }

    def find_edges_by_scan(self, from_deg, to_deg, step_deg=2.0, jump_mm=150):
        """ToF 스윕만으로 테두리를 찾는 보조 방법.
        테두리를 '직접 겨냥'하는 대신, 거리가 크게 튀는 지점(불연속)을 테두리로 본다.
        카메라를 못 쓸 때의 대안이며, ToF 원뿔 때문에 카메라보다 정밀도는 낮다."""
        pts = self.scan_range(from_deg, to_deg, step_deg)
        edges = []
        for i in range(1, len(pts)):
            if abs(pts[i][1] - pts[i-1][1]) >= jump_mm:
                edges.append((pts[i-1][0] + pts[i][0]) / 2.0)
        return edges, pts


# ===============================
# socket_receiver 쪽에서 쓸 헬퍼
# ===============================
def route_message(msg, sensor_queue, response_queue):
    """ESP32에서 온 JSON 한 줄을 종류별로 나눠 담는다.
    - stream : 지도용 스트림 -> sensor_queue (map_2D가 소비)
               ★ .ino 가 실제로 보내는 타입은 "scanpt"가 아니라 "stream"이다.
                 예전 코드가 "scanpt"만 걸러서, 지도용 센서 데이터가 전부
                 조용히 버려지고 있었다 (수정지침.md §3-4).
    - aim/scan/scan_end/pong : 명령 응답 -> response_queue (TofCommander가 소비)
    """
    t = msg.get("type")
    if t == "stream":
        sensor_queue.put({
            'distance_mm': float(msg.get('dist', 0)),
            'angle_deg': float(msg.get('servo', 0.0)),   # 로봇 정면 기준 서보각
            'absolute_angle': False,
            'st': int(msg.get('st', 0)),                 # v4 추가: ToF 원본 status
            'gas': float(msg['gas']) if msg.get('gas') is not None else None,
            'robot_angle': float(msg['yaw']) if msg.get('yaw') is not None else None,
            # robot_x/y 는 .ino 가 아직 안 보낸다(오도메트리 블로커 그 자체).
            # 여기에 "or 0" 같은 기본값을 주면 모든 관측이 원점(0,0)에 쌓여
            # 지도가 조용히 망가지므로, None 이면 None 그대로 넘긴다.
            # 소비 측(map_2D 등)에서 None 을 명시적으로 걸러낼 것.
            'robot_x': msg.get('robot_x'),
            'robot_y': msg.get('robot_y'),
            'valid': bool(msg.get('valid', 1)),
            't': msg.get('t'),
        })
    elif t in ("aim", "scan", "scan_end", "pong"):
        response_queue.put(msg)


if __name__ == "__main__":
    # 각도/크기 계산이 맞는지 하드웨어 없이 확인
    print(f"초점거리 {FOCAL_PX:.1f}px (핀홀 HFOV {2*C.half_fov():.1f}도, {IMG_W}px)")
    print(f"{'bbox':<14}{'좌각':<9}{'우각':<9}{'중심':<9}{'각폭':<9}"
          f"{'500mm폭':<11}{'1000mm폭'}")
    for x1, x2 in [(280, 360), (240, 400), (160, 480), (60, 220)]:
        al, ar = pixel_to_angle(x1), pixel_to_angle(x2)
        ac = (al + ar) / 2
        w500 = width_from_pixels(500, x1, x2)
        w1000 = width_from_pixels(1000, x1, x2)
        print(f"{f'{x1}~{x2}':<14}{al:<9.2f}{ar:<9.2f}{ac:<9.2f}{ar-al:<9.2f}"
              f"{w500:<11.0f}{w1000:.0f}")

    # 화면 가장자리 물체에서 옛 공식이 얼마나 틀리는지 확인
    print()
    print("=== 옛 공식(2*d*tan(각폭/2)) 과 비교 ===")
    print(f"{'bbox':<14}{'옛 공식':<12}{'현재 공식':<12}{'차이'}")
    for x1, x2 in [(280, 360), (160, 240), (40, 120)]:
        al, ar = pixel_to_angle(x1), pixel_to_angle(x2)
        old_w = 2 * 800 * math.tan(math.radians(abs(ar - al) / 2))
        new_w = width_from_pixels(800, x1, x2)
        print(f"{f'{x1}~{x2}':<14}{old_w:<12.0f}{new_w:<12.0f}"
              f"{100*(new_w-old_w)/new_w:+.0f}%")
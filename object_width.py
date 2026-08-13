"""
object_width.py — 물체 폭 계산 (카메라 각폭 + ToF 중심 1회)

배경 (정리문서 1-3):
    VL53L0X는 점이 아니라 FoV 25도 원뿔이다. 1m에서 측정 스팟 지름이 44cm라
    물체 가장자리를 겨냥하면 절반은 물체·절반은 배경에 걸쳐 값을 못 믿는다.
    → 각폭은 카메라(픽셀)로만, 거리는 물체 중심 방향 ToF 1회.

공식 선택:
    문서에 두 공식이 있었는데 서로 다르다.
      (A) 호   : width = 2 * d * tan((ang_r - ang_l) / 2)
      (B) 핀홀 : Z = d * cos(중심각);  width = 픽셀폭 * Z / f
    중심각 0에서만 일치하고, 중심각 40도에서 25%, 60도에서 55% 벌어진다.
    (A)는 좌우 끝이 카메라에서 같은 거리(=원통)라는 가정이라 중심각이
    아예 식에 안 들어간다. 평면 물체에는 틀린다.
    → 기본은 (B). 원통형인 걸 아는 경우만 (A)를 쓴다.

★★ 경고 — 이 모듈의 정확도는 CAMERA_HFOV 하나에 걸려 있다.
    현재 프로젝트에 값이 세 개 돌아다닌다:
        path_planner_sim2.py  CAMERA_FOV  = 135
        robot_config.py       CAMERA_HFOV = 120
        Camera Module 3 Wide 스펙          = 102  (대각선 화각일 가능성)
    같은 픽셀폭·같은 거리인데 135 vs 102는 폭이 1.95배 차이 난다.
    체스보드나 자로 실측해서 하나로 확정하기 전까지 이 모듈의 절대값은
    믿으면 안 된다. (상대 비교나 통신 검증에는 써도 됨)
"""

import math
import json

# ── 카메라 파라미터 ─────────────────────────────────────────
# ※ 실측 후 robot_config.py 하나로 통합할 것.
IMAGE_WIDTH_PX = 640
CAMERA_HFOV_DEG = 120.0      # ★ 미확정. 위 경고 참조.

# 조준 재시도
AIM_TOLERANCE_DEG = 2.0      # 응답 각도가 요청과 이만큼 넘게 다르면 이상
AIM_MAX_ITER = 3             # ok=0일 때 재시도 횟수
AIM_TIMEOUT_S = 3.0


def focal_length_px(image_width=IMAGE_WIDTH_PX, hfov_deg=CAMERA_HFOV_DEG):
    """
    수평 화각으로부터 초점거리(픽셀)를 구한다.
        tan(HFOV/2) = (W/2) / f
    체스보드 캘리브레이션을 하면 이 값이 직접 나오므로,
    나중에 캘리브레이션 결과가 있으면 이 함수 대신 그 값을 쓰는 게 정확하다.
    """
    return (image_width / 2.0) / math.tan(math.radians(hfov_deg / 2.0))


def angle_from_pixel(x_px, image_width=IMAGE_WIDTH_PX,
                     hfov_deg=CAMERA_HFOV_DEG):
    """
    픽셀 x좌표 → 광축 기준 오프셋각(도). 오른쪽이 +.
    ★ 선형 근사(x * HFOV / W)를 쓰면 안 된다. 광각일수록 가장자리에서
      크게 틀린다. 반드시 atan.
    """
    f = focal_length_px(image_width, hfov_deg)
    return math.degrees(math.atan((x_px - image_width / 2.0) / f))


def width_pinhole(dist_mm, x_l_px, x_r_px,
                  image_width=IMAGE_WIDTH_PX, hfov_deg=CAMERA_HFOV_DEG):
    """
    평면 가정 폭 계산 (기본).

    dist_mm : 물체 중심 방향으로 잰 ToF 거리 (시선 방향 사거리)
    x_l_px, x_r_px : YOLO 박스의 좌/우 픽셀 경계

    시선거리를 광축 방향 깊이 Z로 바꾼 뒤 픽셀폭에 곱한다.
        Z = dist * cos(중심각)
        width = (x_r - x_l) * Z / f
    """
    f = focal_length_px(image_width, hfov_deg)
    center_px = (x_l_px + x_r_px) / 2.0
    center_ang = math.atan((center_px - image_width / 2.0) / f)
    depth = dist_mm * math.cos(center_ang)
    return abs(x_r_px - x_l_px) * depth / f


def width_arc(dist_mm, x_l_px, x_r_px,
              image_width=IMAGE_WIDTH_PX, hfov_deg=CAMERA_HFOV_DEG):
    """
    원통 가정 폭 계산. 좌우 끝이 카메라에서 같은 거리일 때만 맞다.
    사람 다리, 기둥, 드럼통 등 단면이 원인 물체에만 쓸 것.
    """
    al = angle_from_pixel(x_l_px, image_width, hfov_deg)
    ar = angle_from_pixel(x_r_px, image_width, hfov_deg)
    return 2.0 * dist_mm * math.tan(math.radians(abs(ar - al) / 2.0))


def center_angle_deg(x_l_px, x_r_px,
                     image_width=IMAGE_WIDTH_PX, hfov_deg=CAMERA_HFOV_DEG):
    """YOLO 박스 중심의 오프셋각. 이 각도로 서보를 겨냥한다."""
    return angle_from_pixel((x_l_px + x_r_px) / 2.0, image_width, hfov_deg)


class WidthMeasurer:
    """
    ESP32에 조준을 요청하고 응답을 받아 폭을 계산한다.

    사용:
        rx = Esp32Receiver(); rx.start()
        wm = WidthMeasurer(rx, esp32_ip='192.168.137.51')
        res = wm.measure(x_l=220, x_r=340)
        print(res['width_mm'])
    """

    def __init__(self, receiver, esp32_ip, cmd_port=8890,
                 image_width=IMAGE_WIDTH_PX, hfov_deg=CAMERA_HFOV_DEG,
                 cylindrical=False):
        import socket
        self.rx = receiver
        self.image_width = image_width
        self.hfov_deg = hfov_deg
        self.cylindrical = cylindrical
        self._next_id = 1

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.settimeout(5.0)
        self.sock.connect((esp32_ip, cmd_port))

    def _send_aim(self, aim_id, angle_off):
        cmd = {"cmd": "aim", "id": aim_id, "angle": round(angle_off, 2)}
        # ★ 개행 필수. ESP32가 \n 단위로 자른다.
        self.sock.sendall((json.dumps(cmd) + "\n").encode())

    def measure(self, x_l, x_r):
        """
        YOLO 박스 좌우 픽셀을 받아 폭(mm)을 반환.
        반환: dict 또는 실패 시 None
        """
        ang_c = center_angle_deg(x_l, x_r, self.image_width, self.hfov_deg)

        res = None
        for attempt in range(AIM_MAX_ITER):
            aim_id = self._next_id
            self._next_id += 1
            self._send_aim(aim_id, ang_c)

            r = self.rx.wait_aim(aim_id, timeout=AIM_TIMEOUT_S)
            if r is None:
                print(f"[width] id={aim_id} 응답 타임아웃 "
                      f"({attempt + 1}/{AIM_MAX_ITER})")
                continue
            if not r.get('ok'):
                print(f"[width] id={aim_id} 측정 실패 "
                      f"({attempt + 1}/{AIM_MAX_ITER})")
                continue

            # 서보가 실제로 그 각도로 갔는지 확인.
            # 요청과 응답 각도가 크게 다르면 각도 규약이 어긋난 것.
            err = abs(r.get('angle', ang_c) - ang_c)
            if err > AIM_TOLERANCE_DEG:
                print(f"[width] 경고: 요청 {ang_c:.1f}도 vs 응답 "
                      f"{r.get('angle'):.1f}도 (오차 {err:.1f}도). "
                      f"SERVO_SIGN/SERVO_CENTER 확인 필요")
            res = r
            break

        if res is None:
            return None

        d = res['dist']
        fn = width_arc if self.cylindrical else width_pinhole
        w = fn(d, x_l, x_r, self.image_width, self.hfov_deg)

        return {
            'aim_id': res['id'],
            'center_angle_deg': ang_c,
            'dist_mm': d,
            'width_mm': w,
            'width_pinhole': width_pinhole(d, x_l, x_r,
                                           self.image_width, self.hfov_deg),
            'width_arc': width_arc(d, x_l, x_r,
                                   self.image_width, self.hfov_deg),
            'yaw': res.get('yaw'),
        }

    def close(self):
        try:
            self.sock.close()
        except OSError:
            pass


# ── 계산부만 단독 점검 (통신 없이) ──────────────────────────
if __name__ == '__main__':
    f = focal_length_px()
    print(f"HFOV {CAMERA_HFOV_DEG}도 → 초점거리 {f:.1f}px "
          f"(1픽셀 = {math.degrees(math.atan(1 / f)):.3f}도)\n")

    print("픽셀 → 각도 (선형 근사와 비교)")
    print(f"{'픽셀':>6} {'atan(정확)':>12} {'선형근사':>10} {'오차':>8}")
    for x in [0, 80, 160, 240, 320, 400, 480, 560, 640]:
        exact = angle_from_pixel(x)
        approx = (x - 320) * CAMERA_HFOV_DEG / 640
        print(f"{x:6d} {exact:12.2f} {approx:10.2f} {exact - approx:8.2f}")

    print("\n폭 계산 (거리 1000mm, 픽셀폭 100)")
    print(f"{'박스위치':>10} {'중심각':>8} {'핀홀mm':>10} {'호mm':>10}")
    for xl in [270, 370, 440, 500, 540]:
        xr = xl + 100
        c = center_angle_deg(xl, xr)
        print(f"{xl:4d}~{xr:<5d} {c:8.1f} "
              f"{width_pinhole(1000, xl, xr):10.1f} "
              f"{width_arc(1000, xl, xr):10.1f}")

    print("\n★ FOV 미확정으로 인한 불확실성 (픽셀폭 100, 깊이 1000mm)")
    for hfov in [135, 120, 102]:
        w = width_pinhole(1000, 270, 370, hfov_deg=hfov)
        print(f"   HFOV {hfov}도 → {w:.0f}mm")

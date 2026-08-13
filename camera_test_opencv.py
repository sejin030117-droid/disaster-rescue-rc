#	카메라 각도 검증 — Canny 윤곽선 기반, YOLO 없이 class-agnostic 인식
"""
camera_test_opencv.py -- Canny+findContours 윤곽선 + 각도 계산 + ToF 폭 측정

camera_test.py(YOLO-seg) 와 나란히 비교하기 위한 병렬 버전이다. 하는 일과
전체 구조(프레임 수신, CSV 로깅, ESP32 통신 스레드)는 동일하고, 물체 인식
방식만 다르다.

[camera_test.py 와 다른 점]
  - YOLO-seg 마스크 대신 Canny+findContours 로 윤곽선을 딴다 (클래스 무관).
    전시회 장애물을 직접 제작하고 재난 잔해 데이터셋을 확보할 방법이 없어서
    "이게 뭔지" 분류할 필요/방법이 없다 - 필요한 건 "여기부터 여기까지"뿐.
  - "신뢰도(confidence)" 개념이 없다. 대신 컨투어 넓이(area_px)를 그 자리에
    쓴다 - 크게 잡힌 물체일수록 노이즈가 아닐 가능성이 높다는 점에서 역할이
    비슷하다.
  - "클래스명" 대신 classify_shape() 이 낸 형태(rect/circle/triangle/unknown)
    를 로그에 쓴다.
  - ESP32 통신(소켓 연결, aim 명령 송신, 9999 응답 수신)을 이 파일 안에
    전부 자체 구현했다. camera_test.py 가 의존하던 socket_receiver.py 나
    tof_commander.py 를 가져다 쓰지 않는다 - 두 카메라 테스트 스크립트가
    서로 완전히 독립적으로 동작해야, 나중에 어느 한쪽만 갖고도 비교 실험을
    돌릴 수 있다.

[라파 스트리밍 명령 - camera_test.py 와 동일]
    rpicam-vid -t 0 --mode 2304:1296 --width 640 --height 360 \
               --inline --listen -o tcp://0.0.0.0:8888

  4:3(640x480)으로 찍으면 libcamera 가 크롭 센서모드를 골라 시야가 절반이
  되고 초점거리가 281 -> 567 로 바뀐다. robot_config.FOCAL_PX 가 무효가 된다.

[성능 참고]
  - 640x360 에서 중앙 1픽셀 = 0.204도. ToF 원뿔(25도)보다 122배 정밀
    (이 정밀도는 카메라가 제공하는 것이라 YOLO/Canny 무관하게 동일하다).
  - Canny 는 YOLO-seg 추론보다 훨씬 가볍다 - GPU 불필요, 프레임마다 돌려도
    보통 수 ms 수준. camera_test.py 의 INFER_EVERY/GPU 전환 로직이 여기엔
    없다.

q 로 종료.
"""

import csv
import json
import socket
import threading
import time
from datetime import datetime

import cv2
import numpy as np

import robot_config as C
import vision_object as vo

# ===============================
# 설정
# ===============================
STREAM_URL = f"tcp://{C.PI_IP}:{C.CAMERA_PORT}"

MIN_AREA_PX = 800        # 이보다 작은 컨투어는 노이즈로 버린다 (ROI 없이
                          # 전체 프레임을 보므로 test_opencv_edges.py 때보다
                          # 여유 있게 잡음 - 배경 잡음 컨투어 배제용)
MAX_CONTOURS = 5          # 프레임당 최대 인식 물체 수
CANNY_LOW = 60
CANNY_HIGH = 150

SEND_TO_ESP32 = True
SEND_INTERVAL = 1.0       # aim 처리(서보 이동+안정화+샘플링) 여유
AIM_SAMPLES = 3           # ESP32 가 평균낼 샘플 수

PRINT_HZ = 2.0
SHOW_TIMING = True
ANALYZE_EVERY = 1         # N프레임마다 1번만 분석. Canny 는 가벼워서 기본 1
                          # (YOLO 쪽 INFER_EVERY=2 와 다름 - 필요하면 올릴 것)


# ===============================
# 서보 현재 각도 추적 (camera_test.py 와 동일한 이유)
# ===============================
servo_angle_deg = 0.0
servo_lock = threading.Lock()


def get_servo_angle():
    with servo_lock:
        return servo_angle_deg


def set_servo_angle(a):
    global servo_angle_deg
    with servo_lock:
        servo_angle_deg = a


# ===============================
# ESP32 통신 - 이 파일 안에 전부 자체 구현 (다른 스크립트와 독립)
# ===============================

# ---- 송신 (8890, aim 명령) ----
esp32_sock = None
esp32_lock = threading.Lock()
aim_id_counter = 0
aim_id_lock = threading.Lock()


def next_aim_id():
    """★ 매번 새 id. 고정 id 를 쓰면 어느 요청의 응답인지 구분 불가."""
    global aim_id_counter
    with aim_id_lock:
        aim_id_counter += 1
        return aim_id_counter


def get_esp32_socket():
    """살아있는 소켓을 재사용한다. 매번 새로 열고 닫으면 ESP32 의 WiFiClient 가
    이전 연결 종료를 바로 인식하지 못해 다음 접속을 못 받고 timeout 이 잦아진다."""
    global esp32_sock
    if esp32_sock is not None:
        return esp32_sock
    try:
        esp32_sock = socket.create_connection((C.ESP32_IP, C.CMD_PORT), timeout=2.0)
        esp32_sock.settimeout(2.0)
        print(f"\n[ESP32] 연결됨 ({C.ESP32_IP}:{C.CMD_PORT})")
    except Exception as e:
        print(f"\n[ESP32 연결 실패] {e}")
        esp32_sock = None
    return esp32_sock


def send_aim(angle_deg, aim_id):
    """aim 명령 전송. 실패해도 카메라 루프는 계속 돈다."""
    global esp32_sock
    with esp32_lock:
        s = get_esp32_socket()
        if s is None:
            return False
        try:
            cmd = json.dumps({"cmd": "aim", "id": aim_id,
                              "angle": round(angle_deg, 2), "n": AIM_SAMPLES})
            s.sendall((cmd + "\n").encode())   # ★ 개행 필수
            return True
        except Exception as e:
            print(f"\n[ESP32 전송 실패] {e}")
            try:
                s.close()
            except Exception:
                pass
            esp32_sock = None
            return False


# ---- 수신 (9999, 센서 스트림 - aim 응답도 여기로 온다) ----
class Esp32Receiver:
    """9999 포트에서 ESP32 스트림을 받아 aim 응답을 id 로 매칭해준다.

    camera_test.py 가 의존하던 socket_receiver.Esp32Receiver 와 인터페이스
    (start/wait_aim/stat/stop) 는 맞췄지만, 이 파일 안에서 새로 구현했다 -
    두 카메라 테스트 스크립트가 서로 의존하지 않게 하기 위함."""

    def __init__(self, port=None, verbose=False):
        self.port = port or C.STREAM_PORT
        self.verbose = verbose
        self._server = None
        self._conn = None
        self._buf = b""
        self._responses = {}          # aim_id -> 응답 dict
        self._lock = threading.Lock()
        self._cv = threading.Condition(self._lock)
        self.running = True
        self.stat = {"lines": 0, "aim": 0, "stream": 0, "errors": 0}
        self._thread = threading.Thread(target=self._loop, daemon=True)

    def start(self):
        self._thread.start()

    def _loop(self):
        srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        try:
            srv.bind(("0.0.0.0", self.port))
            srv.listen(1)
        except OSError as e:
            print(f"[Esp32Receiver] 포트 {self.port} 바인드 실패: {e}")
            return
        srv.settimeout(1.0)
        self._server = srv

        while self.running:
            if self._conn is None:
                try:
                    conn, addr = srv.accept()
                    conn.settimeout(1.0)
                    self._conn = conn
                    if self.verbose:
                        print(f"[Esp32Receiver] 연결됨 {addr}")
                except socket.timeout:
                    continue
                except OSError:
                    break
            try:
                chunk = self._conn.recv(4096)
                if not chunk:
                    self._conn = None
                    continue
                self._buf += chunk
                while b"\n" in self._buf:
                    line, self._buf = self._buf.split(b"\n", 1)
                    self._handle_line(line)
            except socket.timeout:
                continue
            except OSError:
                self._conn = None

    def _handle_line(self, line):
        self.stat["lines"] += 1
        try:
            msg = json.loads(line.decode(errors="ignore").strip())
        except Exception:
            self.stat["errors"] += 1
            return
        t = msg.get("type")
        if t == "aim":
            self.stat["aim"] += 1
            with self._cv:
                self._responses[msg.get("id")] = msg
                self._cv.notify_all()
        elif t == "stream":
            self.stat["stream"] += 1
        # scan/scan_end/pong 등은 이 스크립트에서는 안 씀 - 필요해지면 여기에 분기 추가

    def wait_aim(self, aim_id, timeout=3.0):
        """aim_id 응답이 올 때까지 기다린다. 타임아웃되면 None."""
        deadline = time.time() + timeout
        with self._cv:
            while aim_id not in self._responses:
                remain = deadline - time.time()
                if remain <= 0:
                    return None
                self._cv.wait(timeout=remain)
            return self._responses.pop(aim_id)

    def stop(self):
        self.running = False
        try:
            if self._conn:
                self._conn.close()
        except Exception:
            pass
        try:
            if self._server:
                self._server.close()
        except Exception:
            pass


# ===============================
# 저지연 프레임 수신 (camera_test.py 와 동일)
# ===============================
class FrameGrabber:
    """계속 읽어서 버리고 최신 프레임 한 장만 유지한다."""

    def __init__(self, url):
        self.cap = cv2.VideoCapture(url)
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        self.frame = None
        self.lock = threading.Lock()
        self.running = True
        self.fail = 0
        self.t = threading.Thread(target=self._loop, daemon=True)

    def start(self):
        if not self.cap.isOpened():
            return False
        self.t.start()
        return True

    def _loop(self):
        while self.running:
            ok, f = self.cap.read()
            if not ok:
                self.fail += 1
                time.sleep(0.01)
                continue
            self.fail = 0
            with self.lock:
                self.frame = f

    def read(self):
        with self.lock:
            return None if self.frame is None else self.frame.copy()

    def stop(self):
        self.running = False
        time.sleep(0.1)
        self.cap.release()


# ===============================
# ESP32 수신기 기동 (9999) — aim 응답이 여기로 온다
# ===============================
rx = Esp32Receiver(verbose=False)
rx.start()
print(f"[수신기] {C.STREAM_PORT} 대기 중")

# ===============================
# 스트림 연결
# ===============================
grab = FrameGrabber(STREAM_URL)
if not grab.start():
    print("스트림 연결 실패:", STREAM_URL)
    print("확인: 라파에서 rpicam-vid 실행 중인지 / IP 가 맞는지")
    raise SystemExit(1)

t0 = time.time()
while grab.read() is None and time.time() - t0 < 10:
    time.sleep(0.05)
first = grab.read()
if first is None:
    print("프레임 수신 실패 (10초 대기)")
    grab.stop()
    raise SystemExit(1)

H, W = first.shape[:2]
print(f"스트림 해상도: {W} x {H}")
print(f"초점거리: {C.focal_px(W):.1f}px  "
      f"중앙 1픽셀 = {C.pixel_to_angle(W / 2 + 1, img_w=W):.3f}도")

if (W, H) != (C.IMG_W, C.IMG_H):
    print(f"!! 경고: robot_config 는 {C.IMG_W}x{C.IMG_H} 기준인데 "
          f"스트림이 {W}x{H} 입니다.")
    print(f"   16:9 가 아니면 센서 크롭이 걸려 초점거리가 완전히 달라집니다.")
    print(f"   라파에서 --mode 2304:1296 --width 640 --height 360 으로 재실행 권장")

# ===============================
# CSV
# ===============================
csv_name = f"detection_log_opencv_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
csv_file = open(csv_name, 'w', newline='', encoding='utf-8')
w_csv = csv.writer(csv_file)
w_csv.writerow(['timestamp', 'shape', 'area_px',
                'x1', 'y1', 'x2', 'y2',
                'left_x', 'left_y', 'right_x', 'right_y', 'contour_pts',
                'angle_left', 'angle_right', 'angle_center',
                'servo_angle', 'robot_angle_center'])
print(f"로그: {csv_name}")

# 폭 측정 결과는 별도 파일 (aim 응답이 비동기로 도착하므로 행 대응이 안 맞음)
width_csv_name = f"width_log_opencv_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
width_file = open(width_csv_name, 'w', newline='', encoding='utf-8')
w_width = csv.writer(width_file)
w_width.writerow(['timestamp', 'aim_id', 'shape', 'target_deg',
                  'reported_deg', 'dist_mm', 'px_width', 'width_mm', 'ok'])
print(f"폭 로그: {width_csv_name}")
print("q 로 종료\n")


# ===============================
# aim 요청 -> 응답 -> 폭 계산 (백그라운드, camera_test.py 와 동일한 구조)
# ===============================
def aim_and_measure(shape_label, target_deg, lx, rx_px, img_w):
    """
    별도 스레드에서 실행. 카메라 루프를 막지 않는다.
    ESP32 는 서보 이동 + 안정화 + 샘플링에 200~900ms 를 쓴다.
    """
    aim_id = next_aim_id()
    if not send_aim(target_deg, aim_id):
        return

    res = rx.wait_aim(aim_id, timeout=C.AIM_TIMEOUT_S)
    ts = datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]

    if res is None:
        print(f"\n[aim] id={aim_id} 응답 타임아웃")
        w_width.writerow([ts, aim_id, shape_label, f"{target_deg:.2f}",
                          '', '', abs(rx_px - lx), '', 0])
        width_file.flush()
        return

    reported = res.get('angle', target_deg)
    set_servo_angle(reported)          # ★ 서보 실제 위치 갱신

    if not res.get('ok'):
        print(f"\n[aim] id={aim_id} 측정 실패 (거리 무효)")
        w_width.writerow([ts, aim_id, shape_label, f"{target_deg:.2f}",
                          f"{reported:.2f}", res.get('dist', ''),
                          abs(rx_px - lx), '', 0])
        width_file.flush()
        return

    err = abs(reported - target_deg)
    if err > C.AIM_TOLERANCE_DEG:
        print(f"\n[aim] 경고: 요청 {target_deg:.1f}도 vs 응답 {reported:.1f}도 "
              f"(오차 {err:.1f}도). SERVO_SIGN/SERVO_CENTER 확인")

    dist = res['dist']
    width_mm = C.width_from_box(dist, lx, rx_px, img_w=img_w)

    print(f"\n[폭] {shape_label}: {width_mm:.0f}mm "
          f"(거리 {dist}mm, 각도 {reported:.1f}도, {abs(rx_px - lx)}px)")
    w_width.writerow([ts, aim_id, shape_label, f"{target_deg:.2f}",
                      f"{reported:.2f}", dist, abs(rx_px - lx),
                      f"{width_mm:.1f}", 1])
    width_file.flush()


# ===============================
# 메인 루프
# ===============================
fps_t, fps_n, fps = time.time(), 0, 0.0
last_print = 0.0
last_send = 0.0
frame_idx = 0
last_result = None          # [(contour, area), ...] 캐시 (ANALYZE_EVERY>1 일 때)
t_read_sum = t_analyze_sum = t_draw_sum = 0.0
t_sample_n = 0

try:
    while True:
        t0 = time.time()
        frame = grab.read()
        if frame is None:
            time.sleep(0.01)
            continue
        t_read = time.time() - t0

        frame_idx += 1
        t1 = time.time()
        if frame_idx % ANALYZE_EVERY == 0 or last_result is None:
            found = vo.contours_from_canny_roi(
                frame, (0, 0, W, H), canny_low=CANNY_LOW, canny_high=CANNY_HIGH,
                min_area_px=MIN_AREA_PX, max_contours=MAX_CONTOURS)
            last_result = found
        else:
            found = last_result
        t_analyze = time.time() - t1

        t2 = time.time()
        annotated = frame.copy()
        t_draw = time.time() - t2

        if SHOW_TIMING:
            t_read_sum += t_read; t_analyze_sum += t_analyze; t_draw_sum += t_draw
            t_sample_n += 1

        cur_servo = get_servo_angle()
        detected = []

        for cnt, area in found:
            info = vo.analyze_contour(cnt, img_w=W)
            if info is None:
                continue
            ts = datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]

            shape = info["shape"]
            x1, y1, x2, y2 = (int(v) for v in info["bbox_px"])
            lx, rx_px = x1, x2
            # analyze_contour 의 bbox_px 는 (x_min,y_min,x_max,y_max) 이므로
            # 좌/우 y 는 원 컨투어에서 x_min/x_max 지점의 y 를 다시 찾는다
            pts = cnt.reshape(-1, 2)
            ly = int(pts[pts[:, 0].argmin(), 1])
            ry = int(pts[pts[:, 0].argmax(), 1])
            npts = len(cnt)

            ang_l = info["angle_left"]
            ang_r = info["angle_right"]
            ang_c = info["angle_center"]

            # ★ 오프셋은 빼야 한다 (camera_test.py 와 동일한 부호 규약).
            #   robot_config.cam_angle_to_tof() 로 위임해 부호가 한 곳에서만
            #   정해지게 한다 - camera_test.py 는 이 식을 직접 풀어 썼지만
            #   (cur_servo + ang_c - C.CAM_TOF_OFFSET_DEG) 결과는 동일하다.
            robot_c = cur_servo + C.cam_angle_to_tof(ang_c)

            cv2.polylines(annotated, [cnt], True, (0, 255, 0), 2)
            cv2.circle(annotated, (lx, ly), 5, (255, 0, 0), -1)
            cv2.circle(annotated, (rx_px, ry), 5, (0, 0, 255), -1)
            cv2.putText(annotated, f"{ang_l:.1f}~{ang_r:.1f}deg",
                        (lx, max(20, ly - 28)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)
            cv2.rectangle(annotated, (x1, y1), (x2, y2), (0, 200, 0), 2)
            cv2.putText(annotated, f"{shape} {area:.0f}px", (x1, max(20, y1 - 8)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 200, 0), 2)

            w_csv.writerow([ts, shape, f"{area:.0f}", x1, y1, x2, y2,
                            lx, ly, rx_px, ry, npts,
                            f"{ang_l:.2f}", f"{ang_r:.2f}", f"{ang_c:.2f}",
                            f"{cur_servo:.1f}", f"{robot_c:.2f}"])
            csv_file.flush()
            detected.append((shape, area, ang_l, ang_r, ang_c, npts,
                             lx, rx_px, robot_c))

        # 가장 넓게 잡힌(=YOLO 의 '가장 높은 신뢰도'에 대응) 물체로 aim
        if SEND_TO_ESP32 and detected and time.time() - last_send >= SEND_INTERVAL:
            last_send = time.time()
            best = max(detected, key=lambda x: x[1])   # area 기준
            b_shape, _, _, _, _, _, b_lx, b_rx, b_robot = best
            threading.Thread(target=aim_and_measure,
                             args=(b_shape, b_robot, b_lx, b_rx, W),
                             daemon=True).start()

        if time.time() - last_print >= 1.0 / PRINT_HZ:
            last_print = time.time()
            if detected:
                parts = [f"{s}({a:.0f}px) {l:+.1f}~{r_:+.1f}도"
                         for s, a, l, r_, _, _, _, _, _ in detected]
                line = " | ".join(parts)
            else:
                line = "탐지 없음"
            print(f"\r{fps:4.1f}fps sv{cur_servo:+5.1f}  {line[:100]:<100}",
                  end="", flush=True)

        fps_n += 1
        if time.time() - fps_t >= 1.0:
            fps = fps_n / (time.time() - fps_t)
            fps_t, fps_n = time.time(), 0
            if SHOW_TIMING and t_sample_n > 0:
                print(f"\n[타이밍] 읽기 {t_read_sum/t_sample_n*1000:.1f}ms  "
                      f"분석(Canny) {t_analyze_sum/t_sample_n*1000:.1f}ms  "
                      f"그리기 {t_draw_sum/t_sample_n*1000:.1f}ms  "
                      f"(매 {ANALYZE_EVERY}프레임 분석)")
                t_read_sum = t_analyze_sum = t_draw_sum = 0.0
                t_sample_n = 0

        cv2.putText(annotated, f"FPS {fps:.1f}  servo {cur_servo:+.1f}",
                    (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

        cv2.imshow('Canny contour + angle (YOLO 없음)', annotated)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

except KeyboardInterrupt:
    pass
finally:
    grab.stop()
    cv2.destroyAllWindows()
    csv_file.close()
    width_file.close()
    rx.stop()
    if esp32_sock is not None:
        try:
            esp32_sock.close()
        except Exception:
            pass
    print(f"\n\n로그 저장: {csv_name} / {width_csv_name}")
    print(f"수신 통계: {rx.stat}")

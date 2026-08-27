#	카메라 각도 검증 — atan 기반 각도 계산, CAM_TOF_OFFSET_DEG 부호 수정
"""
camera_test.py -- YOLOv8-seg 윤곽선 + 각도 계산 + ToF 폭 측정

[하는 일]
  1. 라파 스트림에서 프레임 수신 (지연 최소화: 최신 프레임만 사용)
  2. YOLO-seg 로 물체 세그멘테이션 (GPU 가속)
  3. 마스크 -> 윤곽선 추출, 좌/우 극점 찾기
  4. 극점 픽셀 -> 각도 변환  (robot_config.pixel_to_angle)
  5. ESP32로 aim 명령 전송 (8890) + 응답 수신 (9999, socket_receiver 경유)
  6. 응답 거리로 물체 실제 폭 계산
  7. CSV 로그 + 화면 표시

[라파 스트리밍 명령 - 반드시 이걸로]
    rpicam-vid -t 0 --mode 2304:1296 --width 640 --height 360 \
               --inline --listen -o tcp://0.0.0.0:8888

  4:3(640x480)으로 찍으면 libcamera 가 크롭 센서모드를 골라 시야가 절반이
  되고 초점거리가 281 -> 567 로 바뀐다. robot_config.FOCAL_PX 가 무효가 된다.

[성능 참고]
  - 640x360 에서 중앙 1픽셀 = 0.204도. ToF 원뿔(25도)보다 122배 정밀.
  - 이 모델은 OpenVINO export 시 640x640 고정(static) 입력이라 imgsz 는 640 고정.
    ultralytics 가 내부에서 레터박스 처리하고 좌표를 원본 프레임으로 되돌려주므로
    입력이 640x360 이어도 마스크 좌표는 원본 기준이다. 직접 resize 하지 말 것.
  - GPU 첫 추론은 컴파일 때문에 1~2초 걸린다.

q 로 종료.
"""

import csv
import json
import logging
import os
import socket
import threading
import time
from datetime import datetime

import cv2
import numpy as np

# ultralytics 자체 로그 억제 (프레임마다 찍히는 것 방지)
logging.getLogger("ultralytics").setLevel(logging.ERROR)
os.environ.setdefault("YOLO_VERBOSE", "False")

from ultralytics import YOLO

import robot_config as C
from socket_receiver import Esp32Receiver

# ===============================
# 설정
# ===============================
STREAM_URL = f"tcp://{C.PI_IP}:{C.CAMERA_PORT}"
MODEL_DIR = "yolov8n-seg_openvino_model/"
MODEL_PT = "yolov8n-seg.pt"
CONF_MIN = 60.0          # %

SEND_TO_ESP32 = True
SEND_INTERVAL = 1.0      # aim 처리(서보 이동+안정화+샘플링) 여유
AIM_SAMPLES = 3          # ESP32 가 평균낼 샘플 수

PRINT_HZ = 2.0

# ---- 성능 설정 ----
INFER_DEVICE = "intel:gpu"   # 없으면 아래에서 자동으로 cpu 로 전환
INFER_IMGSZ = 640            # 이 모델은 640 고정(static) export
INFER_EVERY = 2              # N프레임마다 1번만 추론
SHOW_TIMING = True


# ===============================
# 서보 현재 각도 추적
# ===============================
# 카메라가 ToF 와 같은 서보에 달려 있으므로, 화면 중앙이 곧 로봇 정면이 아니다.
# 서보가 30도 돌아간 상태에서 화면 중앙에 보이는 물체는 로봇 기준 30도 방향이다.
#
# aim 응답의 'angle' 은 "측정 시점의 서보 오프셋각" 이므로 그걸로 갱신한다.
# (ESP32 가 유일한 진실이고, PC 가 명령한 각도로 추정하면 실패 시 어긋난다)
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
# ESP32 명령 전송 (소켓 재사용)
# ===============================
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


# ===============================
# 저지연 프레임 수신
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
# 모델 준비
# ===============================
print("YOLO-seg 로딩 중... (OpenVINO)")
if not os.path.exists(MODEL_DIR):
    print("OpenVINO 변환 중 (처음 한 번만, 몇 분 걸림)")
    YOLO(MODEL_PT).export(format="openvino")
ov_model = YOLO(MODEL_DIR)

# GPU 가 실제로 잡히는지 확인 (조용히 CPU 로 떨어지는 걸 방지)
try:
    import openvino as ov
    core = ov.Core()
    avail = core.available_devices
    print(f"OpenVINO 사용 가능 디바이스: {avail}")
    if not any(d.startswith("GPU") for d in avail):
        print("!! GPU 가 목록에 없음 -> cpu 로 전환")
        INFER_DEVICE = "cpu"
    else:
        print(f"-> {INFER_DEVICE} 로 추론")
except Exception as e:
    print(f"OpenVINO 디바이스 확인 실패({e}) -> cpu 로 진행")
    INFER_DEVICE = "cpu"

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

# ★ 해상도가 설정과 다르면 초점거리 비례 스케일이 걸리는데,
#   센서모드까지 바뀐 경우엔 비례가 성립하지 않는다. 경고를 띄운다.
if (W, H) != (C.IMG_W, C.IMG_H):
    print(f"!! 경고: robot_config 는 {C.IMG_W}x{C.IMG_H} 기준인데 "
          f"스트림이 {W}x{H} 입니다.")
    print(f"   16:9 가 아니면 센서 크롭이 걸려 초점거리가 완전히 달라집니다.")
    print(f"   라파에서 --mode 2304:1296 --width 640 --height 360 으로 재실행 권장")

# ===============================
# CSV
# ===============================
csv_name = f"detection_log_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
csv_file = open(csv_name, 'w', newline='', encoding='utf-8')
w_csv = csv.writer(csv_file)
w_csv.writerow(['timestamp', 'class', 'confidence',
                'x1', 'y1', 'x2', 'y2',
                'left_x', 'left_y', 'right_x', 'right_y', 'contour_pts',
                'angle_left', 'angle_right', 'angle_center',
                'servo_angle', 'robot_angle_center'])
print(f"로그: {csv_name}")

# 폭 측정 결과는 별도 파일 (aim 응답이 비동기로 도착하므로 행 대응이 안 맞음)
width_csv_name = f"width_log_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
width_file = open(width_csv_name, 'w', newline='', encoding='utf-8')
w_width = csv.writer(width_file)
w_width.writerow(['timestamp', 'aim_id', 'class', 'target_deg',
                  'reported_deg', 'dist_mm', 'px_width', 'width_mm', 'ok'])
print(f"폭 로그: {width_csv_name}")
print("q 로 종료\n")


# ===============================
# aim 요청 -> 응답 -> 폭 계산 (백그라운드)
# ===============================
def aim_and_measure(obj_name, target_deg, lx, rx_px, img_w):
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
        w_width.writerow([ts, aim_id, obj_name, f"{target_deg:.2f}",
                          '', '', abs(rx_px - lx), '', 0])
        width_file.flush()
        return

    reported = res.get('angle', target_deg)
    set_servo_angle(reported)          # ★ 서보 실제 위치 갱신

    if not res.get('ok'):
        print(f"\n[aim] id={aim_id} 측정 실패 (거리 무효)")
        w_width.writerow([ts, aim_id, obj_name, f"{target_deg:.2f}",
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

    print(f"\n[폭] {obj_name}: {width_mm:.0f}mm "
          f"(거리 {dist}mm, 각도 {reported:.1f}도, {abs(rx_px - lx)}px)")
    w_width.writerow([ts, aim_id, obj_name, f"{target_deg:.2f}",
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
last_result = None
t_read_sum = t_infer_sum = t_draw_sum = 0.0
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
        if frame_idx % INFER_EVERY == 0 or last_result is None:
            r = ov_model(frame, device=INFER_DEVICE, imgsz=INFER_IMGSZ,
                         conf=CONF_MIN / 100.0, verbose=False)[0]
            last_result = r
        else:
            r = last_result
        t_infer = time.time() - t1

        t2 = time.time()
        # r.plot() 은 전체 마스크 오버레이+라벨 배경까지 그려서 무겁다(20~30ms).
        # 실제 쓰는 건 윤곽선+각도 텍스트뿐이라 원본에 최소한만 그린다.
        annotated = frame.copy()
        t_draw = time.time() - t2

        if SHOW_TIMING:
            t_read_sum += t_read; t_infer_sum += t_infer; t_draw_sum += t_draw
            t_sample_n += 1

        cur_servo = get_servo_angle()
        detected = []

        for i, box in enumerate(r.boxes):
            conf = float(box.conf[0]) * 100
            if conf < CONF_MIN:
                continue
            name = ov_model.names[int(box.cls[0])]
            x1, y1, x2, y2 = map(int, box.xyxy[0])
            ts = datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]

            lx = ly = rx_px = ry = -1
            npts = 0
            ang_l = ang_r = ang_c = 0.0
            robot_c = 0.0

            if r.masks is not None and i < len(r.masks.data):
                poly = r.masks.xy[i]
                if len(poly) >= 3:
                    c = poly.astype(np.int32)
                    npts = len(c)
                    lx, ly = c[c[:, 0].argmin()]
                    rx_px, ry = c[c[:, 0].argmax()]
                    lx, ly, rx_px, ry = int(lx), int(ly), int(rx_px), int(ry)

                    ang_l = C.pixel_to_angle(float(lx), img_w=W)
                    ang_r = C.pixel_to_angle(float(rx_px), img_w=W)

                    # ★ 각도의 평균이 아니라 '중심 픽셀의 각도'.
                    #   atan 이 비선형이라 (ang_l+ang_r)/2 는 물체 중심이 아니다.
                    #   화면 가장자리에서 최대 1.8도 차이가 난다.
                    ang_c = C.pixel_to_angle((lx + rx_px) / 2.0, img_w=W)

                    # ★ 오프셋은 빼야 한다.
                    #   CAM_TOF_OFFSET_DEG = "화면 중앙에 물체를 놓았을 때
                    #   ToF 가 실제로 보는 각도". ToF 를 물체에 맞추려면
                    #   카메라를 그만큼 덜 돌려야 하므로 뺀다.
                    robot_c = cur_servo + ang_c - C.CAM_TOF_OFFSET_DEG

                    cv2.polylines(annotated, [c], True, (0, 255, 0), 2)
                    cv2.circle(annotated, (lx, ly), 5, (255, 0, 0), -1)
                    cv2.circle(annotated, (rx_px, ry), 5, (0, 0, 255), -1)
                    cv2.putText(annotated, f"{ang_l:.1f}~{ang_r:.1f}deg",
                                (lx, max(20, ly - 28)),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)

            cv2.rectangle(annotated, (x1, y1), (x2, y2), (0, 200, 0), 2)
            cv2.putText(annotated, f"{name} {conf:.0f}%", (x1, max(20, y1 - 8)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 200, 0), 2)

            w_csv.writerow([ts, name, f"{conf:.1f}", x1, y1, x2, y2,
                            lx, ly, rx_px, ry, npts,
                            f"{ang_l:.2f}", f"{ang_r:.2f}", f"{ang_c:.2f}",
                            f"{cur_servo:.1f}", f"{robot_c:.2f}"])
            csv_file.flush()
            detected.append((name, conf, ang_l, ang_r, ang_c, npts,
                             lx, rx_px, robot_c))

        if SEND_TO_ESP32 and detected and time.time() - last_send >= SEND_INTERVAL:
            last_send = time.time()
            best = max(detected, key=lambda x: x[1])
            b_name, _, _, _, _, _, b_lx, b_rx, b_robot = best
            if b_lx >= 0:   # 마스크가 있었던 경우만
                threading.Thread(target=aim_and_measure,
                                 args=(b_name, b_robot, b_lx, b_rx, W),
                                 daemon=True).start()

        if time.time() - last_print >= 1.0 / PRINT_HZ:
            last_print = time.time()
            if detected:
                parts = [f"{n}({c:.0f}%) {l:+.1f}~{r_:+.1f}도"
                         for n, c, l, r_, _, _, _, _, _ in detected]
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
                      f"추론 {t_infer_sum/t_sample_n*1000:.1f}ms  "
                      f"그리기 {t_draw_sum/t_sample_n*1000:.1f}ms  "
                      f"(디바이스={INFER_DEVICE}, imgsz={INFER_IMGSZ}, "
                      f"매 {INFER_EVERY}프레임 추론)")
                t_read_sum = t_infer_sum = t_draw_sum = 0.0
                t_sample_n = 0

        cv2.putText(annotated, f"FPS {fps:.1f}  servo {cur_servo:+.1f}",
                    (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

        cv2.imshow('YOLO-seg contour + angle', annotated)
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
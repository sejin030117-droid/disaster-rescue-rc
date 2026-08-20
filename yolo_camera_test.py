# best.pt(rescue-target) 인식 테스트 -- 라즈베리파이 카메라 스트림 + YOLO 추론만.
# ESP32 조준/CSV 로깅 없음. camera_test.py 와 동일한 프레임 수신 구조만 재사용.
"""
[라파 스트리밍 명령 - 반드시 이걸로]
    rpicam-vid -t 0 --mode 2304:1296 --width 640 --height 360 \
               --inline --listen -o tcp://0.0.0.0:8888

q 로 종료.
"""

import logging
import os
import threading
import time

import cv2

logging.getLogger("ultralytics").setLevel(logging.ERROR)
os.environ.setdefault("YOLO_VERBOSE", "False")

from ultralytics import YOLO

import robot_config as C

MODEL_PATH = "best_ver3.pt"
STREAM_URL = f"tcp://{C.PI_IP}:{C.CAMERA_PORT}"
CONF_MIN = 0.4


class FrameGrabber:
    """계속 읽어서 버리고 최신 프레임 한 장만 유지한다 (camera_test.py 와 동일)."""

    def __init__(self, url):
        self.cap = cv2.VideoCapture(url)
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        self.frame = None
        self.lock = threading.Lock()
        self.running = True
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
                time.sleep(0.01)
                continue
            with self.lock:
                self.frame = f

    def read(self):
        with self.lock:
            return None if self.frame is None else self.frame.copy()

    def stop(self):
        self.running = False
        time.sleep(0.1)
        self.cap.release()


def main():
    model = YOLO(MODEL_PATH)
    print(f"모델 로드: {MODEL_PATH}  클래스: {model.names}")

    grab = FrameGrabber(STREAM_URL)
    if not grab.start():
        print("스트림 연결 실패:", STREAM_URL)
        print("확인: 라파에서 rpicam-vid 실행 중인지 / robot_config.PI_IP 가 맞는지")
        return

    t0 = time.time()
    while grab.read() is None and time.time() - t0 < 10:
        time.sleep(0.05)
    if grab.read() is None:
        print("프레임 수신 실패 (10초 대기)")
        grab.stop()
        return

    print("연결됨. q 로 종료")
    fps_t, fps_n, fps = time.time(), 0, 0.0

    try:
        while True:
            frame = grab.read()
            if frame is None:
                time.sleep(0.01)
                continue

            result = model.predict(frame, conf=CONF_MIN, verbose=False)[0]
            annotated = result.plot()

            fps_n += 1
            if time.time() - fps_t >= 1.0:
                fps = fps_n / (time.time() - fps_t)
                fps_t, fps_n = time.time(), 0
            cv2.putText(annotated, f"FPS {fps:.1f}", (10, 25),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

            cv2.imshow("rescue-target YOLO test", annotated)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
    except KeyboardInterrupt:
        pass
    finally:
        grab.stop()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()

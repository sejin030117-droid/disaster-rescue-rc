"""
socket_receiver.py — ESP32 스트림 수신 (9999 포트, PC가 서버)

camera_test.py 와 object_width.py 가 의존하는 모듈. ESP32가 9999로 보내는
줄 단위 JSON(센서 스트림 + aim 명령 응답)을 받아, aim 응답은 id로 매칭해
wait_aim() 으로 돌려준다.

★ camera_test_opencv.py 는 이 클래스와 인터페이스(start/wait_aim/stat/stop)가
  동일한 사본을 자체 보유한다 - 두 카메라 테스트 스크립트를 서로 독립적으로
  돌리기 위한 의도적인 중복이다(그쪽 docstring 참고). 여기 인터페이스를
  바꾸면 그쪽도 맞춰 바꿀 것.
"""
import json
import socket
import threading
import time

import robot_config as C


class Esp32Receiver:
    """9999 포트에서 ESP32 스트림을 받아 aim 응답을 id 로 매칭해준다."""

    def __init__(self, port=None, verbose=False, on_message=None):
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
        # ★신규 - 파싱된 메시지마다(type 무관하게) 그대로 넘겨주는 훅.
        #   stream 타입은 원래 stat 만 올리고 페이로드를 버렸다(map_2d.py
        #   가 없었던 시절엔 받아갈 곳이 없었으니까). 기존 wait_aim()/stat
        #   경로는 하나도 안 건드렸다 - camera_test.py 등 기존 사용자는
        #   영향 없음. 콜백에서 예외가 나도 수신 스레드는 안 죽는다.
        self.on_message = on_message
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

        if self.on_message is not None:
            try:
                self.on_message(msg)
            except Exception as e:                    # noqa: BLE001
                if self.verbose:
                    print(f"[Esp32Receiver] on_message 콜백 오류(무시): {e}")

        t = msg.get("type")
        if t == "aim":
            self.stat["aim"] += 1
            with self._cv:
                self._responses[msg.get("id")] = msg
                self._cv.notify_all()
        elif t == "stream":
            self.stat["stream"] += 1
        # scan/scan_end/pong 등은 필요해지면 여기에 분기 추가

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

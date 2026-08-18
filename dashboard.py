"""
dashboard.py — 재난 정찰 로봇 관제 UI (PySide6)
(시뮬레이션 맵 + 라이브 카메라 하이브리드 버전)
"""
import os
os.environ["OPENCV_LOG_LEVEL"] = "ERROR"
os.environ["YOLO_VERBOSE"] = "False"

import sys
import math
import time
import threading
from dataclasses import dataclass, field

import numpy as np
import cv2
from PySide6.QtCore import Qt, QTimer, QSize
from PySide6.QtGui import QImage, QPixmap, QColor, QPainter, QFont, QPen
from PySide6.QtWidgets import (QApplication, QWidget, QLabel, QFrame,
                               QVBoxLayout, QHBoxLayout, QGridLayout,
                               QSizePolicy, QPushButton)
import pyqtgraph as pg
from map_canvas import MapCanvas

try:
    import robot_config as C
    STREAM_URL = f"tcp://{C.PI_IP}:{C.CAMERA_PORT}"
except ImportError:
    STREAM_URL = "tcp://0.0.0.0:8888" # 기본값
    C = None

# ── 위험 임계값 (§8-3 단일 출처화) ──────────────────────────────────
# "위험" 단계는 robot_config.FIRE_TEMP_C/GAS_PPM 을 그대로 따라간다 -
# 지도 화재원/가스구역, 실제 경로 회피와 같은 값이어야 "카드는 정상인데
# 왜 이 경로를 피하지?" 같은 혼란이 안 생긴다. "경계(주의)" 조기경고선은
# 그보다 낮게 별도로 잡는다.
# ★가스 danger 값이 기존 60 -> 40 으로 바뀐다(FIRE 는 기존 50 그대로,
#   변화 없음) - GAS_PPM(경로회피/지도표시 기준)과 다르게 놀던 걸 맞춘
#   결과다. 카드에서 "위험"이 이제 더 낮은 농도에서부터 뜬다.
FIRE_TEMP_C  = getattr(C, "FIRE_TEMP_C", 50.0)
GAS_PPM      = getattr(C, "GAS_PPM", 40.0)
TEMP_WARN_C  = getattr(C, "TEMP_WARN_C", 40.0)
GAS_WARN_PPM = getattr(C, "GAS_WARN_PPM", 30.0)

BG        = "#0d1117"      
PANEL     = "#161b22"      
BORDER    = "#232b36"
TEXT      = "#e6edf3"
TEXT_DIM  = "#8b949e"
ACCENT    = "#2f81f7"
OK        = "#3fb950"
WARN      = "#d29922"
DANGER    = "#f85149"

QSS = f"""
QWidget {{
    background: {BG};
    color: {TEXT};
    font-family: 'Malgun Gothic', 'NanumGothic', 'Noto Sans CJK KR', sans-serif;
    font-size: 12px;
}}
QFrame#card {{
    background: {PANEL};
    border: 1px solid {BORDER};
    border-radius: 10px;
}}
QLabel {{
    background: transparent;
}}
QLabel#cardTitle {{
    color: {TEXT_DIM};
    font-size: 11px;
    font-weight: 600;
    letter-spacing: 1px;
}}
QLabel#big {{
    font-size: 26px;
    font-weight: 700;
}}
QLabel#unit {{
    color: {TEXT_DIM};
    font-size: 11px;
}}
QLabel#topTitle {{
    font-size: 15px;
    font-weight: 700;
    letter-spacing: 1px;
}}
"""

GRID_N = 20


def sensor_level(v, warn, danger):
    if v is None:
        return "", TEXT_DIM
    if v >= danger:
        return "위험", DANGER
    if v >= warn:
        return "경계(주의)", WARN
    return "정상", OK


@dataclass
class RobotState:
    dist_mm:   float | None = None
    dist_conf: float = 0.0          
    temp_c:    float | None = None
    gas_ppm:   float | None = None
    humid_pct: float | None = None  

    frame: np.ndarray | None = None
    map_image: np.ndarray | None = None
    map_grid:  np.ndarray | None = None
    heading:   float = 0.0
    trail:     list = field(default_factory=list)
    plan_path: list = field(default_factory=list)
    hazards:   list = field(default_factory=list)
    robot_cell: tuple | None = None

    rescue_target:      tuple | None = None
    rescue_path_safe:   list = field(default_factory=list)
    rescue_path_risky:  list = field(default_factory=list)

    temp_grid: np.ndarray = field(default_factory=lambda: np.full((GRID_N, GRID_N), np.nan))
    gas_grid: np.ndarray = field(default_factory=lambda: np.full((GRID_N, GRID_N), np.nan))
    block_grid: np.ndarray = field(default_factory=lambda: np.zeros((GRID_N, GRID_N), dtype=np.int8))
    detections: list = field(default_factory=list)

    pos:        tuple = (0.0, 0.0)
    goal:       tuple | None = None
    remain_m:   float = 0.0
    eta_s:      float = 0.0
    path_ok:    bool = True

    phase: str = "-"
    step:  int = 0

    modules: dict = field(default_factory=lambda: {
        "ESP32": False, "Raspberry Pi": False, "카메라": False,
        "ToF 센서": False, "온도 센서": False, "가스 센서": False,
        "모터 드라이버": False,
    })
    battery_pct: int = 0
    online: bool = False

    hist_dist: list = field(default_factory=list)
    hist_temp: list = field(default_factory=list)
    hist_gas:  list = field(default_factory=list)

    def push_history(self, maxlen=120):
        for buf, v in ((self.hist_dist, self.dist_mm),
                       (self.hist_temp, self.temp_c),
                       (self.hist_gas, self.gas_ppm)):
            buf.append(v if v is not None else float('nan'))
            del buf[:-maxlen]

class FrameGrabber:
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
        if self.cap.isOpened():
            self.cap.release()

class SimSource:
    def __init__(self, state: RobotState, seed=11, live_cam=False):
        import sim_bridge as B
        self.B = B
        self.s = state
        self.t = 0.0
        self.rng = np.random.default_rng(seed)
        self.live_cam = live_cam 
        self.cam_running = True
        
        for k in self.s.modules:
            self.s.modules[k] = True
        self.s.online = True
        self.s.battery_pct = 92
        self.done = False
        self.mode = None

        try:
            import path_planner_sim2 as S2
        except Exception as e:                        
            print(f"[SimSource] path_planner_sim2 로드 실패({e}) -> MiniExplorer 로 대체")
            S2 = None

        if S2 is not None:
            self._start_sim2(S2, B, seed)
        else:
            self.mode = "mini"
            self.ex = B.MiniExplorer(seed=seed)
            self.path = None
            self.fr = []

        if self.live_cam:
            self.cam_thread = threading.Thread(target=self._cam_loop, daemon=True)
            self.cam_thread.start()

    def _start_sim2(self, S2, B, seed):
        import threading
        self.mode = "sim2"
        self.S2 = S2
        S2._Viz = B.make_dash_viz(self.s, grid_n=GRID_N)

        def _run():
            try:
                S2.run(seed, visualize=True, verbose=False)
            except Exception as e:                     
                print(f"[SimSource] path_planner_sim2 실행 중 예외: {type(e).__name__}: {e}")
            finally:
                self.done = True

        self._thread = threading.Thread(target=_run, daemon=True)
        self._thread.start()

    def _cam_loop(self):
        print(f"[카메라] 스트림 연결 시도: {STREAM_URL}")
        grab = FrameGrabber(STREAM_URL)
        if not grab.start():
            print("[카메라] 연결 실패. (지도 시뮬레이션만 작동합니다)")
        else:
            print("[카메라] 소켓 연결 성공! 첫 프레임 수신 대기 중... (약 3~5초 소요)")

        try:
            from ultralytics import YOLO
            import logging
            logging.getLogger("ultralytics").setLevel(logging.ERROR)
            model = YOLO("yolov8n-seg.pt")
            print("[카메라] YOLO 로드 완료")
        except Exception as e:
            print(f"[카메라] YOLO 로드 실패: {e}")
            model = None

        frame_count = 0
        first_frame_received = False
        
        while self.cam_running: 
            frame = grab.read()
            if frame is not None:
                if not first_frame_received:
                    print("[카메라] 첫 프레임 화면 표시 시작! 정상 송출 중입니다.")
                    first_frame_received = True

                frame_count += 1
                detections = []
                
                try:
                    if model is not None and frame_count % 2 == 0:
                        results = model(frame, verbose=False, conf=0.6)[0]
                        for box in results.boxes:
                            conf = float(box.conf[0])
                            name = model.names[int(box.cls[0])]
                            x1, y1, x2, y2 = map(int, box.xyxy[0].cpu().tolist())
                            w, h = x2 - x1, y2 - y1
                            detections.append({
                                "label": name.upper(),
                                "conf": conf,
                                "dist_m": 0.0, 
                                "angle": 0.0, 
                                "box": (x1, y1, w, h)
                            })
                except Exception as e:
                    print(f"[카메라] YOLO 추론 건너뜀 (에러: {e})")
                
                self.s.frame = frame
                if frame_count % 2 == 0 or not self.s.detections:
                    self.s.detections = detections
                    
            time.sleep(0.03) 
            
        grab.stop()
        print("[카메라] 백그라운드 스레드 정상 종료됨")

    def tick(self, dt=0.1):
        if self.mode == "sim2":
            self._tick_sim2(dt)
        else:
            self._tick_mini(dt)

    def _tick_sim2(self, dt):
        s = self.s
        self.t += dt
        if not self.live_cam:
            s.frame = self._fake_cam()
            s.detections = self._detect(s)
        s.battery_pct = max(5, 92 - int(self.t / 6))

    def _tick_mini(self, dt):
        B, s = self.B, self.s
        self.t += dt
        if not self.done:
            self.fr, self.path, self.done = self.ex.tick()
        ex = self.ex
        s.map_grid   = ex.grid
        s.robot_cell = (ex.x, ex.y)
        s.heading    = ex.heading
        s.trail      = ex.trail
        s.plan_path  = self.path or []
        s.hazards    = self._hazards()
        s.step = ex.step_i
        s.phase = "done" if self.done else "explore"
        s.dist_mm = self._front_dist() * 50.0
        s.dist_conf = 0.95
        cur = ex.trail[-1] if ex.trail else (ex.x, ex.y)
        s.temp_c = ex.measured_t.get(cur)
        s.gas_ppm = ex.measured_g.get(cur)
        s.humid_pct = None
        s.temp_grid = ex.f_temp.to_display_grid(ex.measured_t, GRID_N)
        s.gas_grid = ex.f_gas.to_display_grid(ex.measured_g, GRID_N)

        n = ex.n
        s.pos = ((ex.x - n / 2) * 0.05, (ex.y - n / 2) * 0.05)
        if self.path:
            tx, ty = self.path[-1]
            s.goal = ((tx - n / 2) * 0.05, (ty - n / 2) * 0.05)
            s.remain_m = len(self.path) * 0.05
            s.eta_s = s.remain_m / 0.15
        s.path_ok = True
        s.battery_pct = max(5, 92 - int(self.t / 6))
        
        if not self.live_cam:
            s.frame = self._fake_cam()
            s.detections = self._detect(s)
        s.push_history()

    def _hazards(self):
        ex = self.ex
        out = []
        for (x, y), v in ex.measured_t.items():
            if v > 50:
                out.append({"kind": "fire", "x": x, "y": y,
                            "r": 4 + (v - 50) / 6, "level": (v - 50) / 20})
        for (x, y), v in ex.measured_g.items():
            if v > 40:
                out.append({"kind": "gas", "x": x, "y": y,
                            "r": 4 + (v - 40) / 6, "level": (v - 40) / 25})
        merged = []
        for h in out:
            for m in merged:
                if m["kind"] == h["kind"] and \
                        math.hypot(m["x"] - h["x"], m["y"] - h["y"]) < 9:
                    m["r"] = max(m["r"], h["r"])
                    m["level"] = max(m["level"], h["level"])
                    break
            else:
                merged.append(dict(h))
        return merged

    def _front_dist(self):
        ex = self.ex
        ar = math.radians(ex.heading)
        ca, sa = math.cos(ar), math.sin(ar)
        for i in range(1, ex.scan_range + 1):
            x, y = int(ex.x + i * ca), int(ex.y + i * sa)
            if not (0 <= x < ex.n and 0 <= y < ex.n) or ex.truth[y][x]:
                return i
        return ex.scan_range

    def _detect(self, s):
        if s.temp_c is not None and s.temp_c > 45:
            conf = min(0.99, 0.5 + (s.temp_c - 45) / 40)
            heading = self.S2.robot_angle if self.mode == "sim2" else self.ex.heading
            return [{"label": "FIRE", "conf": conf,
                     "dist_m": (s.dist_mm or 0) / 1000,
                     "angle": heading % 360 - 180,
                     "box": (300, 60, 130, 150)}]
        return []

    def _fake_cam(self):
        h, w, t = 300, 520, self.t
        yy, xx = np.mgrid[0:h, 0:w]
        f = np.zeros((h, w, 3), np.uint8)
        f[..., 0] = (26 + 18 * np.sin(xx / 95 + t)).clip(0, 255)
        f[..., 1] = (30 + 14 * np.cos(yy / 75 - t * .6)).clip(0, 255)
        f[..., 2] = (36 + 12 * np.sin((xx + yy) / 120)).clip(0, 255)
        return (f + self.rng.normal(0, 3, f.shape)).clip(0, 255).astype(np.uint8)

    def stop(self):
        self.done = True
        self.cam_running = False

def card(title: str | None = None) -> tuple[QFrame, QVBoxLayout]:
    f = QFrame(); f.setObjectName("card")
    lay = QVBoxLayout(f); lay.setContentsMargins(12, 10, 12, 10); lay.setSpacing(8)
    if title:
        t = QLabel(title); t.setObjectName("cardTitle")
        t.setFixedHeight(16)
        lay.addWidget(t)
    return f, lay

class MetricCard(QFrame):
    def __init__(self, title, unit, color, none_text="측정 대기"):
        super().__init__()
        self.setObjectName("card")
        self.color = color
        self.none_text = none_text
        lay = QVBoxLayout(self); lay.setContentsMargins(14, 12, 14, 12); lay.setSpacing(4)

        t = QLabel(title); t.setObjectName("cardTitle")
        lay.addWidget(t)

        row = QHBoxLayout(); row.setSpacing(6)
        self.val = QLabel("--"); self.val.setObjectName("big")
        self.val.setStyleSheet(f"color:{color};")
        u = QLabel(unit); u.setObjectName("unit")
        u.setAlignment(Qt.AlignBottom)
        row.addWidget(self.val); row.addWidget(u); row.addStretch()
        lay.addLayout(row)

        self.sub = QLabel(""); self.sub.setObjectName("unit")
        lay.addWidget(self.sub)

        self.spark = pg.PlotWidget()
        self.spark.setFixedHeight(38)
        self.spark.setBackground(None)
        self.spark.hideAxis('bottom'); self.spark.hideAxis('left')
        self.spark.setMouseEnabled(False, False)
        self.spark.setMenuEnabled(False)
        self.curve = self.spark.plot(pen=pg.mkPen(color, width=2))
        lay.addWidget(self.spark)

    def update_value(self, v, fmt="{:.1f}", sub="", hist=None, level_color=None):
        if v is None:
            self.val.setText("--")
            self.val.setStyleSheet(f"color:{TEXT_DIM};")
            self.sub.setText(sub or self.none_text)
            self.sub.setStyleSheet(f"color:{TEXT_DIM}; font-size:11px;")
            return
        self.val.setText(fmt.format(v))
        c = level_color or self.color
        self.val.setStyleSheet(f"color:{c};")
        self.sub.setText(sub)
        self.sub.setStyleSheet(
            f"color:{c if level_color else TEXT_DIM}; font-size:11px;")
        if level_color:
            self.curve.setPen(pg.mkPen(level_color, width=2))
        if hist:
            arr = np.asarray(hist, float)
            self.curve.setData(np.arange(len(arr)), arr)

class HeatGrid(QWidget):
    STOPS = [(0.00, (33, 82, 168)),
             (0.25, (38, 150, 170)),
             (0.50, (60, 170, 90)),
             (0.75, (215, 175, 55)),
             (1.00, (225, 70, 55))]
    C_BLOCK_OBST = QColor("#11151d")
    C_BLOCK_PINK = QColor("#a1547a")

    def __init__(self, vmin, vmax, fmt="{:.0f}"):
        super().__init__()
        self.vmin, self.vmax, self.fmt = vmin, vmax, fmt
        self.grid = np.full((GRID_N, GRID_N), np.nan)
        self.block = np.zeros((GRID_N, GRID_N), dtype=np.int8)
        self.setMinimumHeight(150)
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)

    def update_grid(self, g, block=None):
        self.grid = g
        if block is not None:
            self.block = block
        self.update()

    def _color(self, t):
        t = min(max(t, 0.0), 1.0)
        for i in range(len(self.STOPS) - 1):
            p0, c0 = self.STOPS[i]
            p1, c1 = self.STOPS[i + 1]
            if p0 <= t <= p1:
                k = (t - p0) / (p1 - p0) if p1 > p0 else 0
                return QColor(*[int(a + (b - a) * k) for a, b in zip(c0, c1)])
        return QColor(*self.STOPS[-1][1])

    def paintEvent(self, ev):
        g = self.grid
        n = g.shape[0]
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing, False)

        W, H = self.width(), self.height() - 14
        # ★정사각형 셀 유지 - 폭/높이를 따로 나누면(cw!=ch) 카드가 정사각형이
        # 아닐 때 셀이 직사각형으로 늘어난다. MapCanvas 와 동일하게 작은
        # 쪽(min(W,H))에 맞춰 정사각형 격자를 만들고, 남는 공간은 가운데
        # 정렬 여백으로 둔다. 컬러바(아래 by)는 원래대로 위젯 전체 폭을
        # 그대로 쓴다 - 격자 위치와 무관하게 항상 같은 자리에 있는 게
        # 범례처럼 더 읽기 쉽다.
        s = min(W / n, H / n)
        grid_px = s * n
        ox = (W - grid_px) / 2
        oy = (H - grid_px) / 2
        cw = ch = s
        # ★셀이 좁아지면(해상도를 올렸을 때, 또는 창이 좁아졌을 때) 고정
        # 폰트 크기로는 숫자가 옆 칸까지 번져 안 읽힌다(GRID_N=20으로
        # 올렸을 때 실측 확인됨). 셀 크기에 비례해 폰트를 줄이고, 그래도
        # 못 들어갈 만큼 좁으면(11px 미만) 숫자를 생략하고 색상만 보여준다.
        pt = max(4, min(7, int(s * 0.32)))
        f = QFont(); f.setPointSize(pt); p.setFont(f)
        show_text = s >= 8

        for r in range(n):
            for c in range(n):
                x, y = ox + c * cw, oy + r * ch
                blk = self.block[r, c] if self.block is not None else 0
                if blk == 1:
                    p.fillRect(int(x), int(y), int(cw) + 1, int(ch) + 1,
                              self.C_BLOCK_OBST)
                    continue
                if blk == 2:
                    p.fillRect(int(x), int(y), int(cw) + 1, int(ch) + 1,
                              self.C_BLOCK_PINK)
                    continue
                v = g[r, c]
                if np.isnan(v):
                    p.fillRect(int(x), int(y), int(cw) + 1, int(ch) + 1,
                               QColor(38, 44, 54))
                    continue
                t = (v - self.vmin) / (self.vmax - self.vmin)
                col = self._color(t)
                p.fillRect(int(x), int(y), int(cw) + 1, int(ch) + 1, col)
                if show_text:
                    lum = 0.299 * col.red() + 0.587 * col.green() + 0.114 * col.blue()
                    p.setPen(QColor("#080c12") if lum > 150 else QColor("#e6edf3"))
                    p.drawText(int(x), int(y), int(cw), int(ch),
                               Qt.AlignCenter, self.fmt.format(v))

        by = H + 3
        for i in range(W):
            p.fillRect(i, int(by), 1, 8, self._color(i / max(W - 1, 1)))
        p.setPen(QColor(TEXT_DIM))
        f.setPointSize(6); p.setFont(f)
        p.drawText(1, int(by) + 7, self.fmt.format(self.vmin))
        p.drawText(W - 26, int(by) + 7, self.fmt.format(self.vmax))
        p.end()

class CameraView(QLabel):
    def __init__(self):
        super().__init__()
        self.setMinimumHeight(220)
        self.setAlignment(Qt.AlignCenter)
        self.setStyleSheet(f"background:#05070a; border-radius:8px;")
        self._pm = None
        self._dets = []
        self._current_rgb = None 

    def update_frame(self, frame_bgr, detections):
        self._dets = detections or []
        if frame_bgr is None:
            self.setText("영상 없음")
            return
            
        self._current_rgb = frame_bgr[..., ::-1].copy()
        h, w, _ = self._current_rgb.shape
        img = QImage(self._current_rgb.data, w, h, 3 * w, QImage.Format_RGB888)
        pm = QPixmap.fromImage(img)

        p = QPainter(pm)
        for d in self._dets:
            x, y, bw, bh = d.get("box", (0, 0, 0, 0))
            p.setPen(QPen(QColor(DANGER), 2))
            p.drawRect(x, y, bw, bh)
            p.setFont(QFont("", 9, QFont.Bold))
            p.fillRect(x, y - 18, 8 * len(d["label"]) + 12, 18, QColor(DANGER))
            p.setPen(QColor("#ffffff"))
            p.drawText(x + 6, y - 5, d["label"])
        p.end()

        self._pm = pm
        self.setPixmap(pm.scaled(self.size(), Qt.KeepAspectRatio,
                                 Qt.SmoothTransformation))

    def resizeEvent(self, e):
        if self._pm:
            self.setPixmap(self._pm.scaled(self.size(), Qt.KeepAspectRatio,
                                           Qt.SmoothTransformation))
        super().resizeEvent(e)

class MapView(QLabel):
    def __init__(self):
        super().__init__()
        self.setAlignment(Qt.AlignCenter)
        self.setStyleSheet("background:#05070a; border-radius:8px;")
        self.setMinimumSize(320, 320)
        self._pm = None

    def update_map(self, arr):
        if arr is None:
            self.setText("지도 없음")
            return
        arr = np.ascontiguousarray(arr)
        h, w, _ = arr.shape
        img = QImage(arr.data, w, h, 3 * w, QImage.Format_RGB888)
        self._pm = QPixmap.fromImage(img)
        self.setPixmap(self._pm.scaled(self.size(), Qt.KeepAspectRatio,
                                       Qt.SmoothTransformation))

    def resizeEvent(self, e):
        if self._pm:
            self.setPixmap(self._pm.scaled(self.size(), Qt.KeepAspectRatio,
                                           Qt.SmoothTransformation))
        super().resizeEvent(e)

class StatusRow(QWidget):
    def __init__(self, name):
        super().__init__()
        lay = QHBoxLayout(self); lay.setContentsMargins(0, 2, 0, 2)
        self.dot = QLabel("\u25cf"); self.dot.setFixedWidth(14)
        n = QLabel(name)
        self.st = QLabel("--"); self.st.setAlignment(Qt.AlignRight)
        lay.addWidget(self.dot); lay.addWidget(n); lay.addStretch(); lay.addWidget(self.st)

    def set_ok(self, ok):
        c = OK if ok else DANGER
        self.dot.setStyleSheet(f"color:{c};")
        self.st.setStyleSheet(f"color:{c}; font-size:11px;")
        self.st.setText("OK" if ok else "FAIL")

class Dashboard(QWidget):

    def __init__(self, state: RobotState, source=None):
        super().__init__()
        self.state = state
        self.source = source
        self.setWindowTitle("Disaster Exploration Robot")
        self.resize(1440, 900)
        self.setStyleSheet(QSS)
        self._build()

        from PySide6.QtGui import QShortcut, QKeySequence
        QShortcut(QKeySequence("G"), self, activated=self._toggle_cells)
        QShortcut(QKeySequence("L"), self, activated=self._toggle_legend)

        self.timer = QTimer(self)
        self.timer.timeout.connect(self._refresh)
        self.timer.start(100)          

    def _build(self):
        root = QVBoxLayout(self)
        root.setContentsMargins(14, 12, 14, 14)
        root.setSpacing(10)
        root.addWidget(self._topbar())

        body = QHBoxLayout(); body.setSpacing(10)
        body.addLayout(self._left(), 55)
        body.addLayout(self._right(), 45)
        root.addLayout(body)

    def _topbar(self):
        f = QFrame(); f.setObjectName("card"); f.setFixedHeight(46)
        lay = QHBoxLayout(f); lay.setContentsMargins(16, 0, 16, 0)

        t = QLabel("DISASTER EXPLORATION ROBOT"); t.setObjectName("topTitle")
        self.lbl_online = QLabel("\u25cf SYSTEM OFFLINE")
        self.lbl_online.setStyleSheet(f"color:{DANGER};")
        self.lbl_clock = QLabel("--:--:--"); self.lbl_clock.setStyleSheet(f"color:{TEXT_DIM};")
        self.lbl_batt = QLabel("BATTERY --%")
        
        self.btn_quit = QPushButton("시스템 종료")
        self.btn_quit.setCursor(Qt.PointingHandCursor)
        self.btn_quit.setStyleSheet(f"""
            QPushButton {{
                background-color: {DANGER}; 
                color: white; 
                font-weight: bold; 
                border-radius: 4px; 
                padding: 4px 12px;
            }}
            QPushButton:hover {{ background-color: #ff6b6b; }}
        """)
        self.btn_quit.clicked.connect(self.close) 

        lay.addWidget(t); lay.addSpacing(24)
        lay.addWidget(self.lbl_online); lay.addStretch()
        lay.addWidget(self.lbl_clock); lay.addSpacing(20)
        lay.addWidget(self.lbl_batt); lay.addSpacing(20)
        lay.addWidget(self.btn_quit) 
        return f

    def closeEvent(self, event):
        print("시스템 종료 커맨드 수신. 앱을 종료합니다.")
        if self.source and hasattr(self.source, 'stop'):
            self.source.stop()
        event.accept()

    def _left(self):
        col = QVBoxLayout(); col.setSpacing(10)

        c, lay = card("2D EXPLORATION MAP")
        self.map_canvas = MapCanvas(cell_m=0.05)
        self.map_view = MapView()
        self.map_view.hide()
        lay.addWidget(self.map_canvas, 1)
        lay.addWidget(self.map_view, 1)
        col.addWidget(c, 3)

        c2, lay2 = card("센서 데이터")
        row = QHBoxLayout(); row.setSpacing(10)
        self.m_dist  = MetricCard("거리 (ToF)", "m",   ACCENT)
        self.m_temp  = MetricCard("온도",       "\u00b0C", DANGER)
        self.m_gas   = MetricCard("가스",       "ppm", WARN)
        self.m_humid = MetricCard("습도",       "%",   OK,
                                  none_text="센서 미설치")
        for m in (self.m_dist, self.m_temp, self.m_gas, self.m_humid):
            row.addWidget(m)
        lay2.addLayout(row)
        col.addWidget(c2, 1)
        return col

    def _right(self):
        col = QVBoxLayout(); col.setSpacing(10)

        c, lay = card("카메라 영상 (실시간)")
        self.cam = CameraView()
        lay.addWidget(self.cam, 1)
        self.det_line = QLabel("탐지 없음")
        self.det_line.setStyleSheet(f"color:{TEXT_DIM}; font-size:11px;")
        lay.addWidget(self.det_line)
        col.addWidget(c, 2)   # ★ 카메라 비중 축소 (3→2) - 실물 카메라 없이는
                              #   노이즈 텍스처만 채우는 자리라 우선순위 낮춤

        grids = QHBoxLayout(); grids.setSpacing(10)
        c1, l1 = card("온도 맵 (\u00b0C)")
        self.heat_t = HeatGrid(25, 60)
        l1.addWidget(self.heat_t); grids.addWidget(c1)
        c2, l2 = card("가스 농도 맵 (ppm)")
        self.heat_g = HeatGrid(0, 60)
        l2.addWidget(self.heat_g); grids.addWidget(c2)
        col.addLayout(grids, 5)   # ★ 히트맵 비중 확대 (3→5) - 세분화된
                                  #   해상도(GRID_N=20)를 살리려면 더 큰
                                  #   자리가 필요하다

        bottom = QHBoxLayout(); bottom.setSpacing(10)

        c3, l3 = card("시스템 상태")
        c3.setMinimumHeight(180)   # ★ 비중 축소돼도 항목 7개가 안 찌그러지게
        self.rows = {}
        for name in self.state.modules:
            r = StatusRow(name); self.rows[name] = r; l3.addWidget(r)
        l3.addStretch()
        bottom.addWidget(c3)

        c4, l4 = card("경로 계획 정보")
        c4.setMinimumHeight(180)   # ★ 시스템상태 카드와 동일한 최소 높이
        self.lbl_goal   = QLabel(); self.lbl_pos = QLabel()
        self.lbl_remain = QLabel(); self.lbl_eta = QLabel()
        self.lbl_pstat  = QLabel()
        self.lbl_phase  = QLabel()
        for w in (self.lbl_goal, self.lbl_pos, self.lbl_remain,
                  self.lbl_eta, self.lbl_pstat, self.lbl_phase):
            w.setStyleSheet("font-size:12px;")
            l4.addWidget(w)
        self.lbl_rescue = QLabel("요구조자 : 미발견")
        self.lbl_rescue.setWordWrap(True)
        self.lbl_rescue.setStyleSheet(f"color:{TEXT_DIM}; font-size:12px;")
        l4.addWidget(self.lbl_rescue)
        l4.addStretch()
        bottom.addWidget(c4)

        col.addLayout(bottom, 2)
        return col

    def _toggle_cells(self):
        self.map_canvas.show_cells = not self.map_canvas.show_cells
        self.map_canvas.update()

    def _toggle_legend(self):
        self.map_canvas.show_legend = not self.map_canvas.show_legend
        self.map_canvas.update()

    def _refresh(self):
        if self.source:
            self.source.tick()
        s = self.state

        self.lbl_clock.setText(time.strftime("%H:%M:%S"))
        self.lbl_online.setText("\u25cf SYSTEM ONLINE" if s.online
                                else "\u25cf SYSTEM OFFLINE")
        self.lbl_online.setStyleSheet(f"color:{OK if s.online else DANGER};")
        self.lbl_batt.setText(f"BATTERY {s.battery_pct}%")

        if s.map_grid is not None:
            self.map_canvas.show(); self.map_view.hide()
            self.map_canvas.set_data(s.map_grid, s.robot_cell, s.heading,
                                     s.trail, s.plan_path, s.hazards,
                                     rescue_target=s.rescue_target,
                                     rescue_path_safe=s.rescue_path_safe,
                                     rescue_path_risky=s.rescue_path_risky)
        else:
            self.map_canvas.hide(); self.map_view.show()
            self.map_view.update_map(s.map_image)
        self.cam.update_frame(s.frame, s.detections)

        if s.detections:
            d = s.detections[0]
            self.det_line.setText(
                f"{d['label']}  |  신뢰도 {d['conf']*100:.0f}%  |  "
                f"거리 {d['dist_m']:.2f} m  |  각도 {d['angle']:+.1f}\u00b0")
            self.det_line.setStyleSheet(f"color:{DANGER}; font-size:11px;")
        else:
            self.det_line.setText("탐지 없음")
            self.det_line.setStyleSheet(f"color:{TEXT_DIM}; font-size:11px;")

        self.m_dist.update_value(
            None if s.dist_mm is None else s.dist_mm / 1000, "{:.2f}",
            f"신뢰도 {s.dist_conf*100:.0f}%",
            [None if math.isnan(v) else v/1000 for v in s.hist_dist])
        t_sub, t_color = sensor_level(s.temp_c, TEMP_WARN_C, FIRE_TEMP_C)
        self.m_temp.update_value(s.temp_c, "{:.1f}", t_sub, s.hist_temp,
                                 level_color=t_color)
        g_sub, g_color = sensor_level(s.gas_ppm, GAS_WARN_PPM, GAS_PPM)
        self.m_gas.update_value(s.gas_ppm, "{:.0f}", g_sub, s.hist_gas,
                                level_color=g_color)
        self.m_humid.update_value(s.humid_pct, "{:.0f}", "")

        self.heat_t.update_grid(s.temp_grid, s.block_grid)
        self.heat_g.update_grid(s.gas_grid, s.block_grid)

        for name, ok in s.modules.items():
            self.rows[name].set_ok(ok)

        g = s.goal or (0, 0)
        self.lbl_goal.setText(f"목표 위치 : ({g[0]:.1f}, {g[1]:.1f})")
        self.lbl_pos.setText(f"현재 위치 : ({s.pos[0]:.1f}, {s.pos[1]:.1f})")
        self.lbl_remain.setText(f"남은 거리 : {s.remain_m:.2f} m")
        self.lbl_eta.setText(f"예상 시간 : {int(s.eta_s)//60}분 {int(s.eta_s)%60}초")
        self.lbl_pstat.setText("경로 상태 : " + ("안전" if s.path_ok else "재계획 필요"))
        self.lbl_pstat.setStyleSheet(
            f"color:{OK if s.path_ok else WARN}; font-size:12px;")
        self.lbl_phase.setText(f"단계 : {s.phase}  (스텝 {s.step})")
        self.lbl_phase.setStyleSheet(f"color:{TEXT_DIM}; font-size:11px;")

        if s.rescue_target is None:
            self.lbl_rescue.setText("요구조자 : 미발견")
            self.lbl_rescue.setStyleSheet(f"color:{TEXT_DIM}; font-size:12px;")
        else:
            tx, ty = s.rescue_target
            lines = [f"요구조자 발견 : ({tx}, {ty})"]
            lines.append(f"안전 경로 : {len(s.rescue_path_safe)}칸"
                        if s.rescue_path_safe else "안전 경로 : 없음")
            lines.append(f"위험감수 경로(가스 경유) : {len(s.rescue_path_risky)}칸"
                        if s.rescue_path_risky else "위험감수 경로 : 없음")
            self.lbl_rescue.setText("\n".join(lines))
            color = OK if s.rescue_path_safe else (
                WARN if s.rescue_path_risky else DANGER)
            self.lbl_rescue.setStyleSheet(f"color:{color}; font-size:12px;")


def main():
    app = QApplication(sys.argv)
    state = RobotState()
    
    if "--live" in sys.argv:
        print("시뮬레이션 맵 + 라이브 카메라 모드로 실행합니다.")
        src = SimSource(state, live_cam=True)
    else:
        print("순수 시뮬레이션 모드로 실행합니다.")
        src = SimSource(state, live_cam=False)
        
    win = Dashboard(state, src)
    win.show()
    sys.exit(app.exec())

if __name__ == "__main__":
    main()
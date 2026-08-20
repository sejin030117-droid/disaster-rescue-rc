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
                               QSizePolicy, QPushButton, QProgressBar)
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

# ★시뮬레이션 시드 - 여기가 유일한 출처다(dashboard_ex.py 도 이 값을 쓴다).
#   31 을 쓰는 이유: 요구조자로 가는 두 경로(안전/위험감수)가 실제로 서로
#   다른 길로 갈라지는 시드다. 대부분의 시드는 가스 위험구역이 화재 원과
#   겹쳐 있어서, 가스만 허용해주는 위험감수 경로가 결국 불에 막혀 안전
#   경로와 똑같은 길이 된다(두 색 점선이 완전히 포개져 보임 -
#   rescue_planner.plan_rescue_paths 참고: risky 는 가스만 뚫어주고 불은
#   safe 와 똑같이 막는다). 31 은 가스 얼룩이 불과 안 겹치는 자리에 있어
#   위험감수 경로가 실제로 지름길이 된다.
#   ★값을 바꾸기 전에 두 경로가 갈라지는지 먼저 확인할 것 - 안 갈라지는
#   시드로 바꾸면 "구조경로 2개" 기능이 화면상 사라진 것처럼 보인다.
SIM_SEED = 31


def sensor_level(v, warn, danger):
    if v is None:
        return "", TEXT_DIM
    if v >= danger:
        return "위험", DANGER
    if v >= warn:
        return "경계(주의)", WARN
    return "정상", OK


def dist_conf_color(conf):
    """ToF 신뢰도가 낮으면(값을 못 믿을 수준) 경고색을 준다. 온도/가스
    카드처럼 3단계로 색이 바뀌는 것과 달리, 거리는 값 자체가 아니라
    신뢰도가 낮을 때만 경고하면 되므로 정상 구간은 None(카드 기본색
    유지)을 반환한다."""
    if conf < 0.5:
        return DANGER
    if conf < 0.8:
        return WARN
    return None


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

    rescue_targets:      list = field(default_factory=list)   # [(x,y), ...] 발견된 요구조자들
    rescue_paths_safe:   list = field(default_factory=list)   # rescue_targets 와 같은 순서/길이
    rescue_paths_risky:  list = field(default_factory=list)

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
    collisions: int = 0

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
    def __init__(self, state: RobotState, seed=SIM_SEED, live_cam=False):
        import sim_bridge as B
        self.B = B
        self.s = state
        self.t = 0.0
        self.live_cam = live_cam
        self.cam_running = True
        self.paused = False
        self._viz_cls = None
        
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
        self._viz_cls = S2._Viz   # 일시정지 신호(pause_event)를 들고 있는 클래스

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

    def set_paused(self, paused):
        """일시정지/재개. sim2 모드는 시뮬 스레드를 progress() 안에서 실제로
        멈춰 세우고(건너뛰는 구간 없음), mini 모드는 tick 을 건너뛴다."""
        self.paused = paused
        if self._viz_cls is not None and self._viz_cls.pause_event is not None:
            if paused:
                self._viz_cls.pause_event.clear()
            else:
                self._viz_cls.pause_event.set()

    def tick(self, dt=0.1):
        if self.paused:
            return
        if self.mode == "sim2":
            self._tick_sim2(dt)
        else:
            self._tick_mini(dt)

    def _tick_sim2(self, dt):
        s = self.s
        self.t += dt
        if not self.live_cam:
            s.frame = None
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
            s.frame = None
            s.detections = self._detect(s)
        s.push_history()

    def _hazards(self):
        # ★[버그수정] 50/40 하드코딩 -> FIRE_TEMP_C/GAS_PPM. 이 파일 상단이
        # 이미 robot_config 에서 단일 출처로 끌어온 값인데 mini 모드(sim2
        # 로드 실패시 대체 경로)만 따로 놀고 있었다 - CLAUDE.md §2 인계
        # 이슈와 같은 패턴.
        ex = self.ex
        out = []
        for (x, y), v in ex.measured_t.items():
            if v > FIRE_TEMP_C:
                out.append({"kind": "fire", "x": x, "y": y,
                            "r": 4 + (v - FIRE_TEMP_C) / 6,
                            "level": (v - FIRE_TEMP_C) / 20})
        for (x, y), v in ex.measured_g.items():
            if v > GAS_PPM:
                out.append({"kind": "gas", "x": x, "y": y,
                            "r": 4 + (v - GAS_PPM) / 6,
                            "level": (v - GAS_PPM) / 25})
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
        # ★[버그수정] 45 하드코딩 -> FIRE_TEMP_C(50). 임계값이 카드/지도/
        # 경로회피와 따로 놀면 "카드는 정상인데 왜 이게 뜨지?" 류 혼란이
        # 생긴다(§8-3 단일 출처화 원칙, dashboard.py 상단 주석 참고).
        if s.temp_c is not None and s.temp_c > FIRE_TEMP_C:
            conf = min(0.99, 0.5 + (s.temp_c - FIRE_TEMP_C) / 40)
            heading = self.S2.robot_angle if self.mode == "sim2" else self.ex.heading
            return [{"label": "FIRE", "conf": conf,
                     "dist_m": (s.dist_mm or 0) / 1000,
                     "angle": heading % 360 - 180,
                     "box": (300, 60, 130, 150)}]
        return []

    def stop(self):
        self.done = True
        self.cam_running = False
        self.set_paused(False)   # 일시정지 중 종료하면 시뮬 스레드가 wait 에 갇힌다

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
        self.curve.setPen(pg.mkPen(level_color or self.color, width=2))
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
        # ★setPointSize는 Windows 고배율 DPI 환경에서 실제 렌더링 픽셀
        # 크기가 사실상 0에 가깝게 줄어들어 숫자가 안 보이는 문제가 있었다.
        # setPixelSize는 DPI와 무관하게 항상 지정한 픽셀 수로 그려진다.
        px = max(8, min(15, int(s * 0.9)))
        f = QFont(); f.setPixelSize(px); p.setFont(f)
        # ★px가 이미 최소 7로 바닥을 잡아주므로, 여기 컷오프는 정말
        # 못 그릴 정도로 좁을 때만 걸러내면 된다(예전 8은 옛 포인트
        # 크기 공식 기준이라 지금 공식에서는 너무 쉽게 걸렸다).
        show_text = s >= 5

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
        f.setPixelSize(11); p.setFont(f)
        p.drawText(1, int(by) + 7, self.fmt.format(self.vmin))
        p.drawText(W - 26, int(by) + 7, self.fmt.format(self.vmax))
        p.end()

class CameraView(QLabel):
    def __init__(self):
        super().__init__()
        self.setMinimumHeight(130)   # ★ 220 -> 130. 카메라 실물이 없어 노이즈
                                     #   텍스처만 채우는 자리라, 이 최소값이
                                     #   커서 그리드(히트맵) stretch 비율을
                                     #   먹어버리고 있었다 - 실측 확인됨.
        self.setAlignment(Qt.AlignCenter)
        self.setStyleSheet(f"background:#05070a; border-radius:8px; color:{TEXT_DIM};")
        self._pm = None
        self._dets = []
        self._current_rgb = None
        self.none_text = "영상 없음"   # 실물 카메라 연결 실패용 기본 문구.
                                       # 시뮬레이션 모드에서는 Dashboard 가
                                       # "시뮬레이션 모드 - 카메라 비활성"로 바꿔준다.

    def update_frame(self, frame_bgr, detections):
        self._dets = detections or []
        if frame_bgr is None:
            self.setText(self.none_text)
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
        self.none_text = "지도 없음"   # CameraView 와 같은 패턴 - 시작 전에는
                                       # Dashboard 가 "대기 중" 문구로 바꿔준다

    def update_map(self, arr):
        if arr is None:
            self.setText(self.none_text)
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

    def __init__(self, state: RobotState, source=None, start_callback=None):
        super().__init__()
        self.state = state
        self.source = source
        # ★start_callback 이 있으면 "시작" 버튼을 누르기 전까지 시뮬레이션을
        # 만들지도, 타이머를 돌리지도 않는다. source 를 직접 넘겨서 만들면
        # (start_callback 없이) 예전처럼 곧바로 동작한다 - 기존 호출부 호환.
        self.start_callback = start_callback
        self.started = source is not None
        self.setWindowTitle("Disaster Exploration Robot")
        self.resize(1440, 900)
        self.setStyleSheet(QSS)
        self._build()

        self._t0 = time.time()        # 미션 경과시간 기준점

        from PySide6.QtGui import QShortcut, QKeySequence
        QShortcut(QKeySequence("G"), self, activated=self._toggle_cells)
        QShortcut(QKeySequence("L"), self, activated=self._toggle_legend)
        QShortcut(QKeySequence("Space"), self, activated=self._toggle_pause)

        self.timer = QTimer(self)
        self.timer.timeout.connect(self._refresh)
        if self.started:
            self.timer.start(100)
        else:
            self._refresh()            # 시작 전에도 대기 화면은 한 번 그려둔다

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
        # ★요구조자 발견 배너 - 미션에서 가장 중요한 이벤트인데 예전엔
        #   우측 하단 텍스트만 조용히 바뀌어 놓치기 쉬웠다.
        self.lbl_alert = QLabel("")
        self.lbl_alert.setStyleSheet(
            f"background:{DANGER}; color:white; font-weight:bold;"
            f"border-radius:4px; padding:3px 10px;")
        self.lbl_alert.hide()

        self.lbl_elapsed = QLabel("경과 --:--")
        self.lbl_elapsed.setStyleSheet(f"color:{TEXT_DIM};")
        self.lbl_clock = QLabel("--:--:--"); self.lbl_clock.setStyleSheet(f"color:{TEXT_DIM};")
        self.lbl_batt = QLabel("BATTERY --%")

        self.btn_start = QPushButton("시작")
        self.btn_start.setCursor(Qt.PointingHandCursor)
        self.btn_start.setStyleSheet(f"""
            QPushButton {{
                background-color: {ACCENT};
                color: white;
                font-weight: bold;
                border-radius: 4px;
                padding: 4px 14px;
            }}
            QPushButton:hover {{ background-color: #4c93ff; }}
        """)
        self.btn_start.clicked.connect(self._on_start_clicked)
        self.btn_start.setVisible(not self.started)

        self.btn_pause = QPushButton("일시정지")
        self.btn_pause.setCursor(Qt.PointingHandCursor)
        self.btn_pause.setEnabled(self.started)
        self.btn_pause.setStyleSheet(f"""
            QPushButton {{
                background-color: {PANEL};
                color: {TEXT};
                border: 1px solid {BORDER};
                border-radius: 4px;
                padding: 4px 12px;
            }}
            QPushButton:hover {{ background-color: #21262d; }}
        """)
        self.btn_pause.clicked.connect(self._toggle_pause)

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
        lay.addWidget(self.lbl_online); lay.addSpacing(16)
        lay.addWidget(self.lbl_alert); lay.addStretch()
        lay.addWidget(self.lbl_elapsed); lay.addSpacing(20)
        lay.addWidget(self.lbl_clock); lay.addSpacing(20)
        lay.addWidget(self.lbl_batt); lay.addSpacing(20)
        lay.addWidget(self.btn_start); lay.addSpacing(8)
        lay.addWidget(self.btn_pause); lay.addSpacing(8)
        lay.addWidget(self.btn_quit)
        return f

    def closeEvent(self, event):
        print("시스템 종료 커맨드 수신. 앱을 종료합니다.")
        if self.source and hasattr(self.source, 'stop'):
            self.source.stop()
        event.accept()

    def _left(self):
        col = QVBoxLayout(); col.setSpacing(10)
        self._build_map_card(col)
        self._build_sensor_card(col)
        return col

    def _build_map_card(self, col):
        c, lay = card("2D EXPLORATION MAP")
        self.map_canvas = MapCanvas(cell_m=0.05)
        self.map_view = MapView()
        self.map_view.hide()
        if not self.started:
            self.map_view.none_text = "대기 중 - 시작 버튼을 눌러주세요"
        lay.addWidget(self.map_canvas, 1)
        lay.addWidget(self.map_view, 1)

        # ★탐사 진행률 - map_grid 의 미탐색(-1) 비율에서 바로 나오는 값인데
        #   예전엔 "스텝 N"만 있어서 얼마나 남았는지 알 방법이 없었다.
        prog_row = QHBoxLayout(); prog_row.setSpacing(8)
        self.prog_explore = QProgressBar()
        self.prog_explore.setRange(0, 100)
        self.prog_explore.setTextVisible(False)
        self.prog_explore.setFixedHeight(8)
        self.prog_explore.setStyleSheet(f"""
            QProgressBar {{ background:{BG}; border:1px solid {BORDER};
                            border-radius:4px; }}
            QProgressBar::chunk {{ background:{ACCENT}; border-radius:3px; }}
        """)
        self.lbl_explore = QLabel("탐사 --%")
        self.lbl_explore.setStyleSheet(f"color:{TEXT_DIM}; font-size:11px;")
        self.lbl_explore.setFixedWidth(96)
        prog_row.addWidget(self.lbl_explore)
        prog_row.addWidget(self.prog_explore, 1)
        lay.addLayout(prog_row)

        # ★단축키가 있는지조차 알 수 없었다 - 화면에 명시.
        hint = QLabel("G 격자  ·  L 범례  ·  Space 일시정지")
        hint.setStyleSheet(f"color:{TEXT_DIM}; font-size:10px;")
        hint.setAlignment(Qt.AlignRight)
        lay.addWidget(hint)

        col.addWidget(c, 3)

    def _build_sensor_card(self, col):
        c2, lay2 = card("센서 데이터")
        row = QHBoxLayout(); row.setSpacing(10)
        self.m_dist  = MetricCard("거리 (ToF)", "m",   ACCENT)
        self.m_temp  = MetricCard("온도",       "\u00b0C", DANGER)
        self.m_gas   = MetricCard("가스",       "ppm", WARN)
        # ★습도 카드 제거 - 센서가 아예 없어 항상 "센서 미설치"만 띄우고
        #   자리만 차지했다. 센서를 달면 MetricCard 한 줄과 _refresh 의
        #   update_value 한 줄만 되살리면 된다.
        for m in (self.m_dist, self.m_temp, self.m_gas):
            row.addWidget(m)
        lay2.addLayout(row)
        col.addWidget(c2, 1)

    def _right(self):
        col = QVBoxLayout(); col.setSpacing(10)

        c, lay = card("카메라 영상 (실시간)")
        self.cam = CameraView()
        if not self.started:
            # ★시작 버튼을 누르기 전에는 소스가 아직 없으므로 "대기 중"임을
            #   보여준다 - _on_start_clicked 에서 실제 모드에 맞게 갱신한다.
            self.cam.none_text = "대기 중 - 시작 버튼을 눌러주세요"
        elif not (self.source and getattr(self.source, "live_cam", False)):
            self.cam.none_text = "시뮬레이션 모드 - 카메라 비활성"
        lay.addWidget(self.cam, 1)
        self.det_line = QLabel("탐지 없음")
        self.det_line.setStyleSheet(f"color:{TEXT_DIM}; font-size:11px;")
        lay.addWidget(self.det_line)
        col.addWidget(c, 2)   # ★ 카메라 비중 축소 (3→2) - 실물 카메라 없이는
                              #   노이즈 텍스처만 채우는 자리라 우선순위 낮춤

        grids = QHBoxLayout(); grids.setSpacing(10)
        c1, l1 = card("온도 맵 (\u00b0C)")
        # ★컬러바 상한을 FIRE_TEMP_C/GAS_PPM 기준으로 계산한다(기존
        # 25~60/0~60 하드코딩과 지금 값은 같지만, 이제 임계값을 바꾸면
        # 같이 따라간다) - §8-3 단일 출처화와 같은 이유.
        self.heat_t = HeatGrid(25, FIRE_TEMP_C + 10)
        l1.addWidget(self.heat_t); grids.addWidget(c1)
        c2, l2 = card("가스 농도 맵 (ppm)")
        self.heat_g = HeatGrid(0, GAS_PPM + 20)
        l2.addWidget(self.heat_g); grids.addWidget(c2)
        col.addLayout(grids, 5)   # ★ 히트맵 비중 확대 (3→5) - 세분화된
                                  #   해상도(GRID_N=20)를 살리려면 더 큰
                                  #   자리가 필요하다

        bottom = QHBoxLayout(); bottom.setSpacing(10)

        c3, l3 = card("시스템 상태")
        c3.setMinimumHeight(140)   # ★ 180 -> 140. 항목 7개가 안 찌그러지는
                                   #   선에서 더 낮춰 그리드 쪽에 공간을 더
                                   #   준다 - 실측 확인됨.
        self.rows = {}
        for name in self.state.modules:
            r = StatusRow(name); self.rows[name] = r; l3.addWidget(r)
        l3.addStretch()
        bottom.addWidget(c3, 1)

        c4, l4 = card("경로 계획 정보")
        c4.setMinimumHeight(170)   # ★ 140 -> 170. 고정 6줄 + 요구조자 줄이
                                   #   들어갈 최소치(실측: 2명까지 안 잘림)
        l4.setSpacing(2)           # ★ 기본 8 은 줄이 8개라 간격만 56px 를
                                   #   먹어 정작 글자가 잘렸다(실측).
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
        # ★wordWrap 라벨은 height-for-width 라서, 레이아웃이 최소 높이를
        #   1줄로 잡고 나머지 줄을 잘라버린다(실측: 필요 32px 인데 16px 만
        #   받음). MinimumExpanding 이면 sizeHint 아래로 안 눌린다.
        self.lbl_rescue.setSizePolicy(QSizePolicy.Preferred,
                                      QSizePolicy.MinimumExpanding)
        self.lbl_rescue.setStyleSheet(f"color:{TEXT_DIM}; font-size:12px;")
        l4.addWidget(self.lbl_rescue)
        l4.addStretch()
        bottom.addWidget(c4, 1)   # ★ 요구조자 줄이 좁아서 감기던 문제 -
                                  #   시스템상태 카드와 폭을 균등하게 나눈다

        col.addLayout(bottom, 2)
        return col

    def _on_start_clicked(self):
        """대기 상태에서 '시작'을 누르면 그때 시뮬레이션(소스)을 만들고
        타이머를 돌리기 시작한다. 그 전까지는 화면이 '대기' 상태로만
        표시된다."""
        if self.started or not self.start_callback:
            return
        self.source = self.start_callback()
        self.started = True
        if not (self.source and getattr(self.source, "live_cam", False)):
            self.cam.none_text = "시뮬레이션 모드 - 카메라 비활성"
        self.map_view.none_text = "지도 없음"
        self._t0 = time.time()      # 경과시간은 창을 연 시점이 아니라
                                    # 임무를 시작한 시점부터 센다
        self.btn_start.hide()
        self.btn_pause.setEnabled(True)
        self.timer.start(100)

    def _toggle_pause(self):
        if not (self.source and hasattr(self.source, "set_paused")):
            return
        paused = not getattr(self.source, "paused", False)
        self.source.set_paused(paused)
        self.btn_pause.setText("재개" if paused else "일시정지")

    def _set_map_mode(self, canvas: bool):
        """지도 영역에 벡터 캔버스를 쓸지, 비트맵 폴백(map_view)을 쓸지 전환.

        ★따로 메서드로 뺀 이유: 서브클래스(dashboard_ex.ExDashboard)는
        map_canvas 를 래퍼 위젯 안에 넣어 두므로, map_canvas 를 직접
        숨기면 래퍼가 레이아웃 지분을 그대로 쥔 채 빈 칸으로 남는다
        (대기 화면에서 지도 카드가 반으로 쪼개져 보이던 원인). 무엇을
        보이고 숨길지는 레이아웃을 만든 쪽이 정하게 한다."""
        self.map_canvas.setVisible(canvas)
        self.map_view.setVisible(not canvas)

    def _toggle_cells(self):
        self.map_canvas.show_cells = not self.map_canvas.show_cells
        self.map_canvas.update()

    def _toggle_legend(self):
        self.map_canvas.show_legend = not self.map_canvas.show_legend
        self.map_canvas.update()

    def _update_metric_cards(self, s):
        self.m_dist.update_value(
            None if s.dist_mm is None else s.dist_mm / 1000, "{:.2f}",
            f"신뢰도 {s.dist_conf*100:.0f}%",
            [None if math.isnan(v) else v/1000 for v in s.hist_dist],
            level_color=dist_conf_color(s.dist_conf))
        t_sub, t_color = sensor_level(s.temp_c, TEMP_WARN_C, FIRE_TEMP_C)
        self.m_temp.update_value(s.temp_c, "{:.1f}", t_sub, s.hist_temp,
                                 level_color=t_color)
        g_sub, g_color = sensor_level(s.gas_ppm, GAS_WARN_PPM, GAS_PPM)
        self.m_gas.update_value(s.gas_ppm, "{:.0f}", g_sub, s.hist_gas,
                                level_color=g_color)

    def _refresh(self):
        if self.source:
            self.source.tick()
        s = self.state

        self.lbl_clock.setText(time.strftime("%H:%M:%S"))
        self.lbl_online.setText("\u25cf SYSTEM ONLINE" if s.online
                                else "\u25cf SYSTEM OFFLINE")
        self.lbl_online.setStyleSheet(f"color:{OK if s.online else DANGER};")
        # \u2605\ubc30\ud130\ub9ac\ub294 \uc2e4\uc81c \ud154\ub808\uba54\ud2b8\ub9ac\uac00 \uc544\ub2c8\ub77c \uacbd\uacfc\uc2dc\uac04\uc73c\ub85c \uae4e\uc774\ub294 \uc2dc\ubbac\uac12\uc774\ub2e4
        #   (SimSource._tick_*). \uc2e4\ubb3c \ubc30\ud3ec \uc2dc ESP32 \uc804\uc555\uc744 \ubc1b\uae30 \uc804\uae4c\uc9c0\ub294
        #   \uc9c4\uc9dc\ucc98\ub7fc \ubcf4\uc774\uba74 \uc704\ud5d8\ud558\ubbc0\ub85c (SIM) \uc744 \ubd99\uc5ec \uba85\uc2dc\ud55c\ub2e4.
        self.lbl_batt.setText(f"BATTERY {s.battery_pct}% (SIM)")

        el = int(time.time() - self._t0)
        self.lbl_elapsed.setText(f"\uacbd\uacfc {el // 60:02d}:{el % 60:02d}")

        # \ud0d0\uc0ac \uc9c4\ud589\ub960 - \uc804\uccb4 \uce78 \uc911 \ubbf8\ud0d0\uc0c9(-1) \uc774 \uc544\ub2cc \ube44\uc728
        if s.map_grid is not None:
            g = np.asarray(s.map_grid)
            pct = int(round(100.0 * (g != -1).sum() / g.size))
            self.prog_explore.setValue(pct)
            self.lbl_explore.setText(f"\ud0d0\uc0ac {pct}%")

        if s.map_grid is not None:
            self._set_map_mode(canvas=True)
            self.map_canvas.set_data(s.map_grid, s.robot_cell, s.heading,
                                     s.trail, s.plan_path, s.hazards,
                                     rescue_targets=s.rescue_targets,
                                     rescue_paths_safe=s.rescue_paths_safe,
                                     rescue_paths_risky=s.rescue_paths_risky)
        else:
            self._set_map_mode(canvas=False)
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

        self._update_metric_cards(s)
        self.heat_t.update_grid(s.temp_grid, s.block_grid)
        self.heat_g.update_grid(s.gas_grid, s.block_grid)

        for name, ok in s.modules.items():
            self.rows[name].set_ok(ok)

        g = s.goal or (0, 0)
        self.lbl_goal.setText(f"목표 위치 : ({g[0]:.1f}, {g[1]:.1f})")
        self.lbl_pos.setText(f"현재 위치 : ({s.pos[0]:.1f}, {s.pos[1]:.1f})")
        self.lbl_remain.setText(f"남은 거리 : {s.remain_m:.2f} m")
        self.lbl_eta.setText(f"예상 시간 : {int(s.eta_s)//60}분 {int(s.eta_s)%60}초")
        # ★충돌 횟수 - 예전엔 path_ok(bool) 로만 축약돼서 몇 번 부딪혔는지
        #   알 수 없었다. 시뮬은 이미 세고 있던 값이다.
        self.lbl_pstat.setText(
            "경로 상태 : " + ("안전" if s.path_ok else "재계획 필요")
            + f"   (충돌 {s.collisions}회)")
        self.lbl_pstat.setStyleSheet(
            f"color:{OK if s.path_ok else WARN}; font-size:12px;")
        self.lbl_phase.setText(f"단계 : {s.phase}  (스텝 {s.step})")
        self.lbl_phase.setStyleSheet(f"color:{TEXT_DIM}; font-size:11px;")

        if not s.rescue_targets:
            self.lbl_alert.hide()
            self.lbl_rescue.setText("요구조자 : 미발견")
            self.lbl_rescue.setStyleSheet(f"color:{TEXT_DIM}; font-size:12px;")
        else:
            self.lbl_alert.setText(f"⚠ 요구조자 {len(s.rescue_targets)}명 발견")
            self.lbl_alert.show()
            # ★1명당 3줄이면 2명만 발견돼도 카드 높이를 넘겨 잘렸다
            #   (실측 확인). 한 줄로 압축해 인원이 늘어도 버티게 한다.
            lines = []
            any_safe = False
            any_risky_only = False
            for i, (tx, ty) in enumerate(s.rescue_targets):
                safe = s.rescue_paths_safe[i] if i < len(s.rescue_paths_safe) else []
                risky = s.rescue_paths_risky[i] if i < len(s.rescue_paths_risky) else []
                s_txt = f"{len(safe)}칸" if safe else "없음"
                r_txt = f"{len(risky)}칸" if risky else "없음"
                lines.append(
                    f"구조자{i+1} ({tx},{ty})  안전 {s_txt} / 가스 {r_txt}")
                if safe:
                    any_safe = True
                elif risky:
                    any_risky_only = True
            self.lbl_rescue.setText("\n".join(lines))
            color = OK if any_safe else (WARN if any_risky_only else DANGER)
            self.lbl_rescue.setStyleSheet(f"color:{color}; font-size:12px;")


def main():
    app = QApplication(sys.argv)
    state = RobotState()
    live_cam = "--live" in sys.argv

    def start_callback():
        if live_cam:
            print("시뮬레이션 맵 + 라이브 카메라 모드로 실행합니다.")
            return SimSource(state, seed=SIM_SEED, live_cam=True)
        print("순수 시뮬레이션 모드로 실행합니다.")
        return SimSource(state, seed=SIM_SEED, live_cam=False)

    win = Dashboard(state, start_callback=start_callback)
    win.show()
    sys.exit(app.exec())

if __name__ == "__main__":
    main()
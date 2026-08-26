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
from PySide6.QtCore import Qt, QTimer, QSize, QRectF
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
CELL_SIZE_M  = getattr(C, "CELL_SIZE", 0.05)


def path_length_m(cells, start=None):
    """구조 경로의 실제 이동 거리(m).

    ★[버그수정] 예전엔 화면에 len(path) 를 그대로 "N칸"으로 찍었다. 그런데
    경로는 16방향(직진 1칸, 대각 1.41칸, 나이트 이동 (2,1)=2.24칸)이라
    len(path) 는 '칸 수'가 아니라 '이동 횟수'다 - 실측 3개 시드에서 실제
    거리보다 16~46% 짧게 표시되고 있었다(예: 52 를 2.60m 로 읽게 되는데
    실제로는 3.90m). 구조자가 이 숫자로 거리를 가늠하므로 그대로 두면
    안 된다. 선분 길이를 실제로 합산한다.

    start(출발 칸)는 경로에 안 들어있다 - path_planner._reconstruct 가
    출발 칸을 빼고 돌려주므로, 넘겨받으면 첫 구간까지 포함해서 센다."""
    if not cells:
        return 0.0
    pts = ([start] + list(cells)) if start is not None else list(cells)
    return sum(math.hypot(b[0] - a[0], b[1] - a[1])
               for a, b in zip(pts, pts[1:])) * CELL_SIZE_M

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
    home_cell:  tuple | None = None   # 구조 경로의 출발 칸(진입 지점)

    rescue_targets:      list = field(default_factory=list)   # [(x,y), ...] 발견된 요구조자들
    rescue_paths_safe:   list = field(default_factory=list)   # rescue_targets 와 같은 순서/길이
    rescue_paths_risky:  list = field(default_factory=list)

    temp_grid: np.ndarray = field(default_factory=lambda: np.full((GRID_N, GRID_N), np.nan))
    gas_grid: np.ndarray = field(default_factory=lambda: np.full((GRID_N, GRID_N), np.nan))
    block_grid: np.ndarray = field(default_factory=lambda: np.zeros((GRID_N, GRID_N), dtype=np.int8))
    # 히트맵 칸별 '측정 시각'(time.monotonic 기준, 미측정은 NaN). 값이
    # 얼마나 오래된 정보인지 화면에서 구분하기 위한 것 - sim_bridge 의
    # meas_time_to_display() 가 채운다. None 이면 흐리기 없이 그린다
    # (MiniExplorer 폴백 경로처럼 시각 정보가 없는 소스용).
    meas_time_grid: np.ndarray | None = None
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
        # ★예전엔 0.1초 자고 바로 release() 했다. 스트림이 끊겨 read() 가
        #   그보다 오래 붙들려 있으면, 읽는 도중에 VideoCapture 를 놓아
        #   OpenCV 가 죽는다(종료할 때만 나서 재현이 어려운 크래시). 실제로
        #   루프가 빠져나온 걸 확인하고 놓는다.
        self.running = False
        if self.t.is_alive():
            self.t.join(timeout=2.0)
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
        # ★탐사가 끝난 뒤에는 시간을 더 안 센다. 예전엔 완료 후에도 계속
        #   배터리가 깎여서, 화면을 켜 둔 채 두면 다 끝난 미션의 배터리가
        #   5%까지 내려가 "로봇에 문제가 생긴 것"처럼 보였다.
        if s.phase == "done":
            return
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
        self._colors = None           # _apply_colors 가 바뀔 때만 갱신하도록
        lay.addWidget(self.spark)

    def _apply_colors(self, val_c, sub_c, curve_c):
        """색이 실제로 바뀔 때만 스타일시트/펜을 갱신한다.

        ★값(숫자)은 매 프레임 바뀌지만 색은 위험단계가 넘어갈 때만 바뀐다.
          그런데 예전엔 프레임마다 setStyleSheet 2회 + mkPen 새 객체 생성이
          카드 3개분 돌았다 - 스타일시트는 걸 때마다 파싱+스타일 재계산이
          일어나고, 펜 교체는 스파크라인 곡선을 통째로 다시 그리게 한다."""
        if (val_c, sub_c, curve_c) == self._colors:
            return
        self._colors = (val_c, sub_c, curve_c)
        self.val.setStyleSheet(f"color:{val_c};")
        self.sub.setStyleSheet(f"color:{sub_c}; font-size:11px;")
        self.curve.setPen(pg.mkPen(curve_c, width=2))

    def update_value(self, v, fmt="{:.1f}", sub="", hist=None, level_color=None):
        if v is None:
            self.val.setText("--")
            self.sub.setText(sub or self.none_text)
            self._apply_colors(TEXT_DIM, TEXT_DIM, self.color)
            return
        self.val.setText(fmt.format(v))
        c = level_color or self.color
        self.sub.setText(sub)
        self._apply_colors(c, c if level_color else TEXT_DIM,
                           level_color or self.color)
        if hist:
            arr = np.asarray(hist, float)
            self.curve.setData(np.arange(len(arr)), arr)

class HeatGrid(QWidget):
    STOPS = [(0.00, (33, 82, 168)),
             (0.25, (38, 150, 170)),
             (0.50, (60, 170, 90)),
             (0.75, (215, 175, 55)),
             (1.00, (225, 70, 55))]
    C_BLOCK_OBST = (17, 21, 29)      # "#11151d" 장애물이라 못 잼
    C_BLOCK_PINK = (161, 84, 122)    # "#a1547a" 가려서 확인 포기
    C_NODATA     = (38, 44, 54)      # 아직 안 가봄

    # ── 측정 신선도 표시 (§②) ────────────────────────────────────
    # ★온도·가스는 로봇이 그 칸을 지날 때 한 번 재고 끝이다(접촉식). 실측
    #   결과 미션 종료 시점에 측정 경과가 중앙값 280~447스텝, 최대 656스텝
    #   이었는데, 화면은 4분 전 40°C 와 2초 전 40°C 를 완전히 똑같이
    #   보여주고 있었다. 불이 번지는 상황(§③)에서는 이 차이가 곧 안전
    #   여부라, 오래된 칸일수록 무채색 쪽으로 흐리게 만들어 "이 정보는
    #   낡았다"를 색으로 드러낸다. 값 자체는 그대로 두고 채도만 낮춘다 -
    #   숫자를 바꾸면 측정값을 왜곡하는 셈이라 안 된다.
    FRESH_S = 5.0        # 이 이내면 100% 선명
    STALE_S = 45.0       # 이 이상이면 DIM_FLOOR 까지 흐려짐
    DIM_FLOOR = 0.35     # 아무리 오래돼도 이만큼은 남긴다(안 보이면 무의미)
    DIM_LEVELS = 12      # 흐리기 단계 - 매 프레임 미세하게 바뀌어 화면을
                         # 계속 다시 그리는 걸 막으려고 양자화한다
    # ★단계만 양자화해서는 부족하다. 칸이 400개라 그중 하나쯤은 거의 매
    #   프레임 단계 경계를 넘어서, 시뮬이 멈춰 있어도(일시정지·탐사완료)
    #   히트맵이 계속 다시 그려졌다(실측: 값 고정 3초 동안 1회 -> 65회).
    #   시각 자체를 이 간격으로 끊어서 보면 흐리기 결과가 초당 한 번만
    #   바뀐다 - 40초에 걸친 페이드에 1초 granularity 는 눈에 안 보인다.
    DIM_TICK_S = 1.0

    # ★색상 룩업테이블(256단계). 예전엔 셀마다 _color() 를 파이썬으로 돌려
    #   STOPS 구간을 선형보간했다 - 격자 400칸 x 히트맵 2개 x 10Hz =
    #   초당 8000번. 값->색은 순수 함수라 한 번만 만들어두면 된다.
    _LUT = None

    @classmethod
    def _lut(cls):
        if cls._LUT is None:
            xs = np.linspace(0.0, 1.0, 256)
            ps = np.array([p for p, _ in cls.STOPS], float)
            cs = np.array([c for _, c in cls.STOPS], float)
            lut = np.empty((256, 3), float)
            for ch in range(3):
                lut[:, ch] = np.interp(xs, ps, cs[:, ch])
            cls._LUT = lut.astype(np.uint8)
        return cls._LUT

    def __init__(self, vmin, vmax, fmt="{:.0f}"):
        super().__init__()
        self.vmin, self.vmax, self.fmt = vmin, vmax, fmt
        self.grid = np.full((GRID_N, GRID_N), np.nan)
        self.block = np.zeros((GRID_N, GRID_N), dtype=np.int8)
        self.setMinimumHeight(150)
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self._rgb = None        # drawImage 용 버퍼(QImage 가 참조하므로 보관 필수)
        self._bar = None
        self.meas_time = None   # 칸별 측정 시각 (없으면 흐리기 안 함)
        self._dim_q = None      # 양자화된 흐리기 단계 (다시 그릴지 판단용)

    def _dim_quantized(self):
        """칸별 흐리기 계수를 DIM_LEVELS 단계로 양자화해 돌려준다.
        측정 시각이 없으면 None(= 흐리기 없음)."""
        if self.meas_time is None:
            return None
        now = math.floor(time.monotonic() / self.DIM_TICK_S) * self.DIM_TICK_S
        age = now - self.meas_time
        span = max(self.STALE_S - self.FRESH_S, 1e-6)
        t = np.clip((age - self.FRESH_S) / span, 0.0, 1.0)
        dim = 1.0 - (1.0 - self.DIM_FLOOR) * t
        dim[np.isnan(self.meas_time)] = 1.0      # 측정 없는 칸은 어차피 회색
        return np.round(dim * self.DIM_LEVELS).astype(np.int16)

    def update_grid(self, g, block=None, meas_time=None):
        # ★값이 그대로면 다시 그리지 않는다. 대시보드는 시뮬 진행 여부와
        #   무관하게 100ms 마다 이 함수를 부르는데(일시정지·탐사완료 후에도
        #   계속), 예전엔 그때마다 무조건 update() 를 걸어 똑같은 그림을
        #   다시 그렸다.
        same = (self.grid is g or np.array_equal(self.grid, g, equal_nan=True))
        if block is not None:
            if not np.array_equal(self.block, block):
                same = False
            self.block = block
        self.grid = g
        # ★흐리기(§②)는 시간이 흐르기만 해도 바뀐다 - 시뮬이 멈춰 있어도
        #   낡아가는 게 맞다. 다만 매 프레임 미세하게 달라지는 값으로
        #   비교하면 영영 다시 그리게 되므로 단계로 양자화해서 본다.
        self.meas_time = meas_time
        dim_q = self._dim_quantized()
        if not (dim_q is None and self._dim_q is None) and \
                not np.array_equal(dim_q, self._dim_q):
            same = False
        self._dim_q = dim_q
        if not same:
            self.update()

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
        if W <= 0 or H <= 0:
            p.end()
            return
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

        # ── 셀 색상: 격자 전체를 numpy 로 한 번에 만들어 이미지 한 장으로
        #    그린다. 예전엔 셀마다 fillRect + 파이썬 색보간이었다(400칸 x
        #    히트맵 2개 x 10Hz). 확대는 NEAREST 로 - 셀 경계가 뭉개지면
        #    격자를 읽는 의미가 없다.
        lut = self._lut()
        blk = self.block if self.block is not None else np.zeros_like(g, np.int8)
        with np.errstate(invalid="ignore"):
            t = (np.nan_to_num(g, nan=0.0) - self.vmin) / (self.vmax - self.vmin)
        idx = np.clip(t * 255.0, 0, 255).astype(np.uint8)
        rgb = lut[idx].astype(np.float32)
        nan_m = np.isnan(g)
        # ★오래된 측정일수록 무채색(C_NODATA) 쪽으로 섞어 흐리게 한다.
        #   어둡게 하는 대신 회색으로 빼는 이유: 어둡게 하면 컬러맵의
        #   낮은 값(파랑 계열)과 헷갈린다 - "낡음"과 "차가움"은 다른 정보다.
        if self._dim_q is not None:
            # ★변수명 주의: f 는 위에서 QFont 로 쓰고 있다(가리면 아래
            #   컬러바 라벨에서 폰트 대신 배열을 만지게 된다).
            dim = (self._dim_q.astype(np.float32) / self.DIM_LEVELS)[..., None]
            rgb = rgb * dim + np.float32(self.C_NODATA) * (1.0 - dim)
        rgb = rgb.astype(np.uint8)
        rgb[nan_m] = self.C_NODATA
        rgb[blk == 2] = self.C_BLOCK_PINK
        rgb[blk == 1] = self.C_BLOCK_OBST      # 장애물이 확인불가보다 우선
        self._rgb = np.ascontiguousarray(rgb, dtype=np.uint8)
        img = QImage(self._rgb.data, n, n, 3 * n, QImage.Format_RGB888)
        p.setRenderHint(QPainter.SmoothPixmapTransform, False)
        p.drawImage(QRectF(ox, oy, grid_px, grid_px), img)

        if show_text:
            # 숫자는 값이 있고 가려지지 않은 칸에만. 배경 밝기에 따라
            # 글자색을 뒤집는다(어두운 칸엔 밝은 글자).
            lum = (0.299 * rgb[..., 0] + 0.587 * rgb[..., 1]
                   + 0.114 * rgb[..., 2])
            dark_pen, light_pen = QColor("#080c12"), QColor("#e6edf3")
            cur = None
            for r, c in zip(*np.where(~nan_m & (blk == 0))):
                want = dark_pen if lum[r, c] > 150 else light_pen
                if want is not cur:
                    p.setPen(want); cur = want
                p.drawText(int(ox + c * cw), int(oy + r * ch),
                           int(cw), int(ch),
                           Qt.AlignCenter, self.fmt.format(g[r, c]))

        # ── 컬러바: 예전엔 픽셀 폭만큼 fillRect 를 돌렸다(가로 700px 이면
        #    프레임당 700회). LUT 자체가 이미 그라디언트라 이미지 한 장으로
        #    늘려 그리면 된다. 여기는 NEAREST 가 아니라 부드럽게.
        by = H + 3
        if self._bar is None:
            self._bar = np.ascontiguousarray(lut.reshape(1, 256, 3))
        bar_img = QImage(self._bar.data, 256, 1, 3 * 256, QImage.Format_RGB888)
        p.setRenderHint(QPainter.SmoothPixmapTransform, True)
        p.drawImage(QRectF(0, by, W, 8), bar_img)
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
        self._last_frame = None       # 같은 프레임 재렌더 방지 (update_frame)
        self.none_text = "영상 없음"   # 실물 카메라 연결 실패용 기본 문구.
                                       # 시뮬레이션 모드에서는 Dashboard 가
                                       # "시뮬레이션 모드 - 카메라 비활성"로 바꿔준다.

    def update_frame(self, frame_bgr, detections):
        detections = detections or []
        if frame_bgr is None:
            self._dets = detections
            self.setText(self.none_text)
            return
        # ★같은 프레임을 다시 그리지 않는다. 카메라 스레드는 대략 30fps 로
        #   프레임을 갈아끼우지만 대시보드는 10Hz 로 여기를 부르고, 연결이
        #   끊기거나 느려지면 같은 배열이 계속 들어온다 - 그때마다 BGR->RGB
        #   복사 + QImage + 박스 그리기 + 스케일링을 통째로 다시 했다.
        if frame_bgr is self._last_frame and detections == self._dets:
            return
        self._last_frame = frame_bgr
        self._dets = detections

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
        self.dot = QLabel("●"); self.dot.setFixedWidth(14)
        n = QLabel(name)
        self.st = QLabel("--"); self.st.setAlignment(Qt.AlignRight)
        self._ok = None               # set_ok 가 바뀔 때만 스타일을 갱신하도록
        lay.addWidget(self.dot); lay.addWidget(n); lay.addStretch(); lay.addWidget(self.st)

    def set_ok(self, ok):
        # ★상태는 거의 안 바뀌는데 예전엔 100ms 마다 무조건 스타일시트를
        #   다시 걸었다(항목 7개 x 2회 = 초당 140회의 스타일 재계산).
        if ok == self._ok:
            return
        self._ok = ok
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
        self._done_elapsed = None     # 탐사 완료 시점의 경과시간(그 뒤로 고정)
        self._css = {}                # 위젯별 마지막 스타일시트 (_set_text_color)

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
        self.lbl_online = QLabel("● SYSTEM OFFLINE")
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
        # ★타이머를 먼저 세운다 - 안 그러면 source.stop() 이후에도 _refresh
        #   가 한 번 더 돌면서 이미 정리된 소스를 건드릴 수 있다.
        self.timer.stop()
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
        self.m_temp  = MetricCard("온도",       "°C", DANGER)
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
        # ★"흐릴수록 오래된 측정"을 제목에 적어둔다 - 안 적으면 흐린 칸을
        #   낮은 값으로 오해한다(색이 옅어진 게 아니라 회색이 섞인 것).
        c1, l1 = card("온도 맵 (°C)  ·  흐릴수록 오래된 측정")
        # ★컬러바 상한을 FIRE_TEMP_C/GAS_PPM 기준으로 계산한다(기존
        # 25~60/0~60 하드코딩과 지금 값은 같지만, 이제 임계값을 바꾸면
        # 같이 따라간다) - §8-3 단일 출처화와 같은 이유.
        self.heat_t = HeatGrid(25, FIRE_TEMP_C + 10)
        l1.addWidget(self.heat_t); grids.addWidget(c1)
        c2, l2 = card("가스 농도 맵 (ppm)  ·  흐릴수록 오래된 측정")
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

    def _set_text_color(self, label, text, color, extra=""):
        """라벨 글자와 색을 함께 갱신하되, 스타일시트는 실제로 바뀔 때만 건다.

        ★setStyleSheet 는 문자열을 파싱해서 위젯 스타일을 다시 계산(polish)
          하게 만든다 - 값이 같아도 비용이 그대로 든다. 이 대시보드는
          100ms 마다 화면 전체를 갱신하는데 색이 바뀌는 일은 드물다."""
        label.setText(text)
        css = f"color:{color};{extra}"
        if self._css.get(label) != css:
            self._css[label] = css
            label.setStyleSheet(css)

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

        # ★탐사가 끝나면(phase=="done") 경과시간을 그 시점에 멈춘다 - 계속
        #   올라가면 아직 수색 중인 것처럼 읽힌다(구조자 뷰와 동일한 규칙).
        done = (s.phase == "done")
        if done and self._done_elapsed is None:
            self._done_elapsed = int(time.time() - self._t0)
            # 끝난 시뮬레이션을 "일시정지"할 수는 없다 - 눌러도 아무 일이
            # 안 일어나는 버튼은 고장난 것처럼 보이므로 비활성화한다.
            self.btn_pause.setEnabled(False)

        self.lbl_clock.setText(time.strftime("%H:%M:%S"))
        if done:
            self._set_text_color(self.lbl_online, "● 탐사 완료", ACCENT)
        elif s.online:
            self._set_text_color(self.lbl_online, "● SYSTEM ONLINE", OK)
        else:
            self._set_text_color(self.lbl_online, "● SYSTEM OFFLINE", DANGER)
        # ★배터리는 실제 텔레메트리가 아니라 경과시간으로 깎이는 시뮬값이다
        #   (SimSource._tick_*). 실물 배포 시 ESP32 전압을 받기 전까지는
        #   진짜처럼 보이면 위험하므로 (SIM) 을 붙여 명시한다.
        self.lbl_batt.setText(f"BATTERY {s.battery_pct}% (SIM)")

        el = self._done_elapsed if self._done_elapsed is not None \
            else int(time.time() - self._t0)
        self.lbl_elapsed.setText(f"경과 {el // 60:02d}:{el % 60:02d}")

        if s.map_grid is not None:
            # 탐사 진행률 - 전체 칸 중 미탐색(-1) 이 아닌 비율
            g = np.asarray(s.map_grid)
            pct = int(round(100.0 * (g != -1).sum() / g.size))
            self.prog_explore.setValue(pct)
            self.lbl_explore.setText(f"탐사 {pct}%")

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
            self._set_text_color(
                self.det_line,
                f"{d['label']}  |  신뢰도 {d['conf']*100:.0f}%  |  "
                f"거리 {d['dist_m']:.2f} m  |  각도 {d['angle']:+.1f}°",
                DANGER, " font-size:11px;")
        else:
            self._set_text_color(self.det_line, "탐지 없음",
                                 TEXT_DIM, " font-size:11px;")

        self._update_metric_cards(s)
        self.heat_t.update_grid(s.temp_grid, s.block_grid, s.meas_time_grid)
        self.heat_g.update_grid(s.gas_grid, s.block_grid, s.meas_time_grid)

        for name, ok in s.modules.items():
            self.rows[name].set_ok(ok)

        # ★시작 전에는 값이 없는데도 "목표 (0.0, 0.0) / 남은 거리 0.00 m /
        #   예상 0분 0초 / 경로 상태 안전"이 실제 측정값처럼 찍혀 있었다 -
        #   대기 화면이 정상 운행 중인 것처럼 읽힌다. 데이터가 오기 전에는
        #   "--" 로 비워 둔다.
        if s.map_grid is None:
            self.lbl_goal.setText("목표 위치 : --")
            self.lbl_pos.setText("현재 위치 : --")
            self.lbl_remain.setText("남은 거리 : --")
            self.lbl_eta.setText("예상 시간 : --")
            self._set_text_color(self.lbl_pstat, "경로 상태 : --",
                                 TEXT_DIM, " font-size:12px;")
            self._set_text_color(self.lbl_phase, "단계 : 대기",
                                 TEXT_DIM, " font-size:11px;")
            self.lbl_alert.hide()
            self._set_text_color(self.lbl_rescue, "요구조자 : --",
                                 TEXT_DIM, " font-size:12px;")
            return

        g = s.goal or (0, 0)
        self.lbl_goal.setText(f"목표 위치 : ({g[0]:.1f}, {g[1]:.1f})")
        self.lbl_pos.setText(f"현재 위치 : ({s.pos[0]:.1f}, {s.pos[1]:.1f})")
        self.lbl_remain.setText(f"남은 거리 : {s.remain_m:.2f} m")
        self.lbl_eta.setText(f"예상 시간 : {int(s.eta_s)//60}분 {int(s.eta_s)%60}초")
        # ★충돌 횟수 - 예전엔 path_ok(bool) 로만 축약돼서 몇 번 부딪혔는지
        #   알 수 없었다. 시뮬은 이미 세고 있던 값이다.
        self._set_text_color(
            self.lbl_pstat,
            "경로 상태 : " + ("안전" if s.path_ok else "재계획 필요")
            + f"   (충돌 {s.collisions}회)",
            OK if s.path_ok else WARN, " font-size:12px;")
        self._set_text_color(self.lbl_phase, f"단계 : {s.phase}  (스텝 {s.step})",
                             TEXT_DIM, " font-size:11px;")

        if not s.rescue_targets:
            self.lbl_alert.hide()
            self._set_text_color(self.lbl_rescue, "요구조자 : 미발견",
                                 TEXT_DIM, " font-size:12px;")
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
                s_txt = (f"{path_length_m(safe, s.home_cell):.2f}m"
                         if safe else "없음")
                r_txt = (f"{path_length_m(risky, s.home_cell):.2f}m"
                         if risky else "없음")
                lines.append(
                    f"구조자{i+1} ({tx},{ty})  안전 {s_txt} / 가스 {r_txt}")
                if safe:
                    any_safe = True
                elif risky:
                    any_risky_only = True
            color = OK if any_safe else (WARN if any_risky_only else DANGER)
            self._set_text_color(self.lbl_rescue, "\n".join(lines),
                                 color, " font-size:12px;")


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
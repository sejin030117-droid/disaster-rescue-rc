"""
dashboard_ex.py — 구조자용 창 + 지도 확대판 대시보드 (2창 구성)

dashboard.py 의 상태/시뮬레이션/기본 위젯을 그대로 재사용하고, 레이아웃만
다르게 짠 두 개의 창을 띄운다.

- RescuerWindow : 지도만 크게, 최소한의 정보만 표시한다. 나중에 앱과
  연동해서 이 창만 노출할 계획이다(구조자용 뷰) - 그래서 조작 버튼은
  두지 않는다.
- ExDashboard   : dashboard.Dashboard 를 상속한다. 정보량은 기존과
  같지만 "센서 데이터" 카드를 지도 위 우측 하단 구석으로 밀어 최소
  크기로 줄이고, 그만큼 지도가 커진다.

두 창 모두 ExDashboard 의 "시작" 버튼을 누르기 전까지는 아무 것도
동작하지 않는다(SimSource 생성도, 화면 갱신도 없음) - dashboard.py 에
추가된 시작 게이트(Dashboard.start_callback/started)를 그대로 물려받아
쓴다. 시작을 누르면 ExDashboard 가 시뮬레이션 tick 을 갖고, RescuerWindow
는 같은 state 를 읽어 그리기만 한다(tick 을 두 번 부르면 시뮬레이션
속도가 두 배가 되므로 tick 은 반드시 한쪽에서만 부른다).
"""
import sys
import time

from PySide6.QtCore import Qt, QTimer
from PySide6.QtWidgets import (QApplication, QWidget, QLabel, QFrame,
                               QVBoxLayout, QHBoxLayout, QProgressBar,
                               QSizePolicy)

import dashboard as D
from map_canvas import MapCanvas


class RescueMapCanvas(MapCanvas):
    """구조자 뷰 전용 지도 - 범례에서 '탐색 경로'/'계획 경로'를 뺀다.

    ★이 창은 로봇의 탐사 궤적과 탐사 계획 경로를 아예 안 그리는데
    (RescuerWindow._refresh 에서 trail/path 를 안 넘긴다), 범례는 원래
    고정 10항목이라 화면에 없는 두 색을 계속 안내하고 있었다 - 범례가
    거짓말을 하는 셈. 실제로 그리는 것만 남겨서, 구조자가 가장 봐야 할
    두 색(구조경로 안전/위험감수)이 덜 묻히게 한다."""

    def _legend_items(self):
        drop = {"탐색 경로", "계획 경로"}
        return [(c, name) for c, name in super()._legend_items()
                if name not in drop]


class CornerOverlay(QWidget):
    """main_widget 을 위젯 전체에 채우고, corner_widget 을 지정한 모서리에
    작게 겹쳐 띄운다. 센서 카드를 지도 구석으로 밀어붙이는 용도."""

    def __init__(self, main_widget, corner_widget, margin=12, corner="br"):
        super().__init__()
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self.setMinimumSize(320, 320)
        self.main_widget = main_widget
        self.corner_widget = corner_widget
        self.margin = margin
        self.corner = corner
        main_widget.setParent(self)
        corner_widget.setParent(self)
        corner_widget.raise_()

    def resizeEvent(self, ev):
        self.main_widget.setGeometry(0, 0, self.width(), self.height())
        cw = self.corner_widget.sizeHint().width()
        ch = self.corner_widget.sizeHint().height()
        x = self.margin if "l" in self.corner else self.width() - cw - self.margin
        y = self.margin if "t" in self.corner else self.height() - ch - self.margin
        self.corner_widget.setGeometry(int(x), int(y), cw, ch)
        super().resizeEvent(ev)


class MiniSensorPanel(QFrame):
    """센서 데이터를 지도 구석에 아주 작게 - 카드/스파크라인 없이 값만."""

    def __init__(self):
        super().__init__()
        self.setAttribute(Qt.WA_StyledBackground, True)
        self.setStyleSheet(f"""
            QFrame {{
                background: rgba(22, 27, 34, 225);
                border: 1px solid {D.BORDER};
                border-radius: 8px;
            }}
        """)
        lay = QVBoxLayout(self)
        lay.setContentsMargins(10, 8, 10, 8)
        lay.setSpacing(3)
        self.lbl_dist = QLabel("거리 --")
        self.lbl_temp = QLabel("온도 --")
        self.lbl_gas = QLabel("가스 --")
        for w in (self.lbl_dist, self.lbl_temp, self.lbl_gas):
            w.setStyleSheet(f"color:{D.TEXT}; font-size:11px; background:transparent;")
            lay.addWidget(w)

    def update_values(self, s: "D.RobotState"):
        dist_txt = "--" if s.dist_mm is None else f"{s.dist_mm / 1000:.2f} m"
        self.lbl_dist.setText(f"거리 {dist_txt}")
        dc = D.dist_conf_color(s.dist_conf)
        self.lbl_dist.setStyleSheet(
            f"color:{dc or D.TEXT}; font-size:11px; background:transparent;")

        t_sub, t_color = D.sensor_level(s.temp_c, D.TEMP_WARN_C, D.FIRE_TEMP_C)
        t_txt = "--" if s.temp_c is None else f"{s.temp_c:.1f}°C"
        self.lbl_temp.setText(f"온도 {t_txt} {t_sub}".rstrip())
        self.lbl_temp.setStyleSheet(
            f"color:{t_color}; font-size:11px; background:transparent;")

        g_sub, g_color = D.sensor_level(s.gas_ppm, D.GAS_WARN_PPM, D.GAS_PPM)
        g_txt = "--" if s.gas_ppm is None else f"{s.gas_ppm:.0f} ppm"
        self.lbl_gas.setText(f"가스 {g_txt} {g_sub}".rstrip())
        self.lbl_gas.setStyleSheet(
            f"color:{g_color}; font-size:11px; background:transparent;")


class ExDashboard(D.Dashboard):
    """dashboard.Dashboard 와 동일하지만, '센서 데이터' 카드를 지도 위
    우측 하단 구석으로 밀어붙이고 그 자리를 지도가 채운다."""

    def __init__(self, state, start_callback=None, on_started=None,
                 rescuer_window=None):
        # ★super().__init__() 안에서 _build() -> _build_map_card() 가
        #   바로 호출되므로, 거기서 참조하는 값들을 먼저 세팅해둔다.
        self._on_started_cb = on_started
        self._rescuer_win = rescuer_window
        super().__init__(state, source=None, start_callback=start_callback)
        self.setWindowTitle("Disaster Exploration Robot — 통합 관제")

    def _build_map_card(self, col):
        c, lay = D.card("2D EXPLORATION MAP")
        self.map_canvas = MapCanvas(cell_m=0.05)
        self.map_view = D.MapView()
        self.map_view.hide()

        # ★센서 데이터 카드를 없애고 지도 위 우측 하단 구석에 작게 겹쳐
        #   띄운다 - 그만큼 지도가 열 전체를 쓴다(사용자 요청).
        self.mini_sensor = MiniSensorPanel()
        self._map_overlay = CornerOverlay(self.map_canvas, self.mini_sensor,
                                          corner="br")
        lay.addWidget(self._map_overlay, 1)
        lay.addWidget(self.map_view, 1)   # 지도 격자가 없을 때의 폴백 경로

        prog_row = QHBoxLayout(); prog_row.setSpacing(8)
        self.prog_explore = QProgressBar()
        self.prog_explore.setRange(0, 100)
        self.prog_explore.setTextVisible(False)
        self.prog_explore.setFixedHeight(8)
        self.prog_explore.setStyleSheet(f"""
            QProgressBar {{ background:{D.BG}; border:1px solid {D.BORDER};
                            border-radius:4px; }}
            QProgressBar::chunk {{ background:{D.ACCENT}; border-radius:3px; }}
        """)
        self.lbl_explore = QLabel("탐사 --%")
        self.lbl_explore.setStyleSheet(f"color:{D.TEXT_DIM}; font-size:11px;")
        self.lbl_explore.setFixedWidth(96)
        prog_row.addWidget(self.lbl_explore)
        prog_row.addWidget(self.prog_explore, 1)
        lay.addLayout(prog_row)

        hint = QLabel("G 격자  ·  L 범례  ·  Space 일시정지")
        hint.setStyleSheet(f"color:{D.TEXT_DIM}; font-size:10px;")
        hint.setAlignment(Qt.AlignRight)
        lay.addWidget(hint)

        col.addWidget(c, 1)

    def _build_sensor_card(self, col):
        pass   # 센서 카드는 지도 구석의 mini_sensor 로 대체됐다 (없음)

    def _set_map_mode(self, canvas: bool):
        # map_canvas 가 아니라 그걸 감싼 오버레이를 숨겨야 레이아웃 지분까지
        # 같이 넘어간다 - 안 그러면 지도 카드가 반으로 쪼개진다(base 주석 참고).
        self._map_overlay.setVisible(canvas)
        self.map_view.setVisible(not canvas)

    def _update_metric_cards(self, s):
        self.mini_sensor.update_values(s)

    def _on_start_clicked(self):
        was_started = self.started
        super()._on_start_clicked()
        if not was_started and self.started and self._on_started_cb:
            self._on_started_cb(self.source)

    def closeEvent(self, event):
        super().closeEvent(event)
        if self._rescuer_win is not None:
            self._rescuer_win.close()


class RescuerWindow(QWidget):
    """구조자용 창 - 지도만 크게, 정보는 최소한으로.

    나중에 앱과 연동해 이 창만 노출할 계획이라 조작 버튼(시작/일시정지/
    종료)은 두지 않는다 - 그건 ExDashboard(통합 관제) 쪽 역할이다. 같은
    이유로 시뮬레이션 tick 도 여기선 부르지 않는다: ExDashboard 가 이미
    매 프레임 tick 을 부르고 있으므로, 여기서도 부르면 시뮬레이션 속도가
    두 배가 된다. 이 창은 공유된 RobotState 를 읽어 그리기만 한다."""

    def __init__(self, state: "D.RobotState"):
        super().__init__()
        self.state = state
        self.setWindowTitle("Disaster Exploration Robot — 구조자 뷰")
        self.resize(900, 720)
        self.setStyleSheet(D.QSS)
        self._t0 = time.time()
        self._done_elapsed = None      # 탐사 완료 시점의 경과시간(그 뒤로 고정)
        self._build()

        self.timer = QTimer(self)
        self.timer.timeout.connect(self._refresh)
        self._refresh()          # 시작 전에도 대기 화면은 한 번 그려둔다

    def start(self):
        """ExDashboard 에서 '시작'을 눌렀을 때만 호출된다."""
        if not self.timer.isActive():
            self._t0 = time.time()
            self._done_elapsed = None
            self.timer.start(150)   # 표시 전용이라 조작용 창보다 여유있게

    def _build(self):
        root = QVBoxLayout(self)
        root.setContentsMargins(14, 12, 14, 14)
        root.setSpacing(10)

        top = QFrame(); top.setObjectName("card"); top.setFixedHeight(46)
        tl = QHBoxLayout(top); tl.setContentsMargins(16, 0, 16, 0)
        t = QLabel("구조 지원 - 실시간 위치"); t.setObjectName("topTitle")
        self.lbl_online = QLabel("● 대기 중")
        self.lbl_online.setStyleSheet(f"color:{D.TEXT_DIM};")
        self.lbl_alert = QLabel("")
        self.lbl_alert.setStyleSheet(
            f"background:{D.DANGER}; color:white; font-weight:bold;"
            f"border-radius:4px; padding:3px 10px;")
        self.lbl_alert.hide()
        self.lbl_elapsed = QLabel("경과 --:--")
        self.lbl_elapsed.setStyleSheet(f"color:{D.TEXT_DIM};")
        self.lbl_batt = QLabel("BATTERY --%")
        tl.addWidget(t); tl.addSpacing(20)
        tl.addWidget(self.lbl_online); tl.addSpacing(16)
        tl.addWidget(self.lbl_alert); tl.addStretch()
        tl.addWidget(self.lbl_elapsed); tl.addSpacing(20)
        tl.addWidget(self.lbl_batt)
        root.addWidget(top)

        self.map_canvas = RescueMapCanvas(cell_m=0.05)
        # ★지도가 오기 전(시작 전)에는 MapCanvas 가 아무것도 안 그려서 창이
        #   통째로 검은 화면이 된다 - 고장난 것과 구분이 안 된다. 이 창은
        #   나중에 앱으로 구조자에게 그대로 노출할 화면이라 특히 문제라,
        #   대기 중임을 명시하는 자리표시자를 둔다.
        self.lbl_waiting = QLabel("대기 중\n\n관제 화면에서 임무를 시작하면\n"
                                  "구조 경로가 여기에 표시됩니다")
        self.lbl_waiting.setAlignment(Qt.AlignCenter)
        self.lbl_waiting.setStyleSheet(
            f"color:{D.TEXT_DIM}; font-size:13px; line-height:150%;"
            f"background:#0a0e14; border-radius:8px;")
        root.addWidget(self.map_canvas, 1)
        root.addWidget(self.lbl_waiting, 1)
        self.map_canvas.hide()

    def _refresh(self):
        s = self.state
        # ★탐사가 끝나면(phase=="done") 경과시간을 그 시점에 멈춘다 - 계속
        #   올라가면 아직 수색 중인 것처럼 읽힌다. 구조자에게는 "언제 끝난
        #   지도인가"가 곧 정보의 신선도라 중요하다.
        done = (s.phase == "done")
        if done and self._done_elapsed is None:
            self._done_elapsed = int(time.time() - self._t0)

        if done:
            self.lbl_online.setText("● 탐사 완료")
            self.lbl_online.setStyleSheet(f"color:{D.ACCENT};")
        elif s.online:
            self.lbl_online.setText("● 임무 진행 중")
            self.lbl_online.setStyleSheet(f"color:{D.OK};")
        else:
            self.lbl_online.setText("● 대기 중")
            self.lbl_online.setStyleSheet(f"color:{D.TEXT_DIM};")

        self.lbl_batt.setText(
            f"BATTERY {s.battery_pct}%" if s.online else "BATTERY --%")
        if self._done_elapsed is not None:
            el = self._done_elapsed
        else:
            el = int(time.time() - self._t0) if s.online else 0
        self.lbl_elapsed.setText(f"경과 {el // 60:02d}:{el % 60:02d}")

        if s.map_grid is not None:
            self.lbl_waiting.hide()
            self.map_canvas.show()
            # ★구조자 뷰 요청: 로봇의 탐사 궤적(trail)·탐사 계획 경로
            #   (plan_path)는 구조자에게 필요 없는 정보라 안 그린다 -
            #   요구조자로 가는 경로(rescue_paths_safe/risky)만 보여준다.
            #   메인 관제창(ExDashboard)은 그대로 다 그린다.
            self.map_canvas.set_data(s.map_grid, s.robot_cell, s.heading,
                                     trail=None, path=None, hazards=s.hazards,
                                     rescue_targets=s.rescue_targets,
                                     rescue_paths_safe=s.rescue_paths_safe,
                                     rescue_paths_risky=s.rescue_paths_risky)

        if not s.rescue_targets:
            self.lbl_alert.hide()
        else:
            self.lbl_alert.setText(f"⚠ 요구조자 {len(s.rescue_targets)}명 발견")
            self.lbl_alert.show()


def main():
    app = QApplication(sys.argv)
    state = D.RobotState()
    live_cam = "--live" in sys.argv

    def start_callback():
        # 시드는 dashboard.SIM_SEED 가 유일한 출처다 - 기본값에 기대지 않고
        # 명시적으로 넘겨서, 이 파일만 봐도 어느 시드로 도는지 보이게 한다.
        if live_cam:
            print("시뮬레이션 맵 + 라이브 카메라 모드로 실행합니다.")
            return D.SimSource(state, seed=D.SIM_SEED, live_cam=True)
        print("순수 시뮬레이션 모드로 실행합니다.")
        return D.SimSource(state, seed=D.SIM_SEED, live_cam=False)

    rescuer_win = RescuerWindow(state)

    def on_started(source):
        rescuer_win.start()

    ex_win = ExDashboard(state, start_callback=start_callback,
                         on_started=on_started, rescuer_window=rescuer_win)

    rescuer_win.show()
    ex_win.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()

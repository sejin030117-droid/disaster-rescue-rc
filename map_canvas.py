"""
map_canvas.py — 벡터 스타일 2D 탐사 지도 위젯

격자를 셀 단위로 찍지 않고, 윤곽을 뽑아 부드러운 곡선으로 그린다.

[파이프라인]
  격자(-1/0/1/2/3/4)
    → 이진 마스크 (자유공간 / 장애물)
    → 모폴로지로 계단 완화
    → cv2.findContours 로 폴리곤 추출
    → Chaikin 세분화로 모서리 둥글리기
    → QPainterPath 채우기

[왜 격자를 직접 안 그리나]
  탐사 지도는 5cm 격자라 확대하면 계단이 그대로 보인다.
  실제 물체는 매끈하므로, 관측 격자를 '그대로' 보여주는 것보다
  윤곽을 복원해 그리는 편이 사람이 읽기 쉽다.
  ★ 다만 이건 표시 전용이다. 경로 계획은 여전히 격자로 한다.
    부드럽게 그린 경계가 실제 통행 가능 경계와 다를 수 있으므로,
    화면만 보고 "여기 지나갈 수 있겠다"고 판단하면 안 된다.
"""
from __future__ import annotations

import math

import numpy as np
import cv2
from PySide6.QtCore import Qt, QPointF, QRectF
from PySide6.QtGui import (QPainter, QPainterPath, QColor, QPen, QBrush,
                           QFont, QRadialGradient, QPolygonF)
from PySide6.QtWidgets import QWidget, QSizePolicy


# ── 색 ──────────────────────────────────────────────────────────
C_BG        = QColor("#0a0e14")
C_UNKNOWN   = QColor("#1b2230")
C_FREE      = QColor("#dfe4ea")
C_FREE_EDGE = QColor("#ffffff")
C_OBST      = QColor("#11151d")
C_OBST_EDGE = QColor("#3a4354")
# ★미확정(값2) 상태는 시스템 자체에서 삭제됨 (path_planner_sim2.py
#   update_obstacle_shape 참고) - C_UNCONF 도 함께 제거. 이제 grid 에는
#   장애물이 "볼록껍질로 확정됨(1)" 아니면 "아직 안 그려짐(직접관측 점만
#   보임)" 둘 중 하나뿐이라, 중간 색이 필요 없다.
C_PINK      = QColor("#a1547a")     # 확인 불가(값 4)
C_TRAIL     = QColor("#4a90d9")
C_PATH      = QColor("#3fd17a")
C_RESCUE_MARK  = QColor("#ff3d9a")     # 요구조자 SOS 마커
C_RESCUE_SAFE  = QColor("#2fd8e8")     # 구조경로 - 안전 (고온+가스 모두 회피)
C_RESCUE_RISKY = QColor("#ffa53d")     # 구조경로 - 위험감수 (가스 경유, 고온은 회피)
C_ROBOT     = QColor("#ff4d4d")
C_AXIS      = QColor("#3a4250")
C_TEXT      = QColor("#8b949e")


# ── 폴리곤 매끈하게 ──────────────────────────────────────────────
def chaikin(pts, iters=2, closed=True):
    """Chaikin 코너 컷팅. 반복마다 점이 2배가 되고 모서리가 둥글어진다.
    2회면 격자 계단이 거의 사라지고 형태는 유지된다."""
    p = np.asarray(pts, float)
    for _ in range(iters):
        if len(p) < 3:
            return p
        a = p
        b = np.roll(p, -1, axis=0) if closed else np.vstack([p[1:], p[-1]])
        q = 0.75 * a + 0.25 * b
        r = 0.25 * a + 0.75 * b
        p = np.empty((len(a) * 2, 2))
        p[0::2], p[1::2] = q, r
    return p


def _desaturate(c: QColor, factor=0.6):
    """채도를 factor 배로 낮춘다 (1.0=원본, 0.0=무채색). 명도/알파는 유지."""
    h, s, v, a = c.getHsv()
    return QColor.fromHsv(h, int(s * factor), v, a)


def mask_to_polys(mask, min_area=6, smooth_px=1.2, chaikin_iters=2):
    """이진 마스크 → 매끈한 폴리곤 목록(외곽, 구멍 구분).

    반환: [(outer_pts, [hole_pts, ...]), ...]  좌표 단위는 '셀'
    """
    if not mask.any():
        return []

    # 업샘플 후 블러 → 계단 완화. 격자 해상도가 낮아서 이 단계가 필요하다.
    # ★ UP=6(원래값)은 60x60 격자를 360x360 으로 키워 매 프레임 처리한다.
    #   격자가 자주 바뀌는 실시간 탐사 중엔 이게 그대로 프레임당 비용이라,
    #   4로 낮춰 처리 픽셀을 절반 이하(240x240)로 줄였다. 매끈함 차이는
    #   거의 안 보이는데 findContours/GaussianBlur 비용은 크게 준다.
    UP = 4
    m = (mask.astype(np.uint8) * 255)
    m = cv2.resize(m, None, fx=UP, fy=UP, interpolation=cv2.INTER_NEAREST)
    k = int(smooth_px * UP) | 1
    m = cv2.GaussianBlur(m, (k, k), 0)
    _, m = cv2.threshold(m, 127, 255, cv2.THRESH_BINARY)

    # ★ CHAIN_APPROX_SIMPLE 을 쓰면 안 된다.
    #   직선 구간을 양 끝점 2개로 압축하므로, 큰 사각형이 4점 폴리곤이 되고
    #   거기에 Chaikin 을 걸면 변 길이의 25% 씩 잘려나간다.
    #   (60칸 벽 → 모서리가 15칸씩 대각선으로 깎임)
    #   NONE 으로 모든 점을 남기면 Chaikin 이 픽셀 규모에서만 작용한다.
    cnts, hier = cv2.findContours(m, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_NONE)
    if hier is None:
        return []
    hier = hier[0]

    out = []
    for i, c in enumerate(cnts):
        if hier[i][3] != -1:            # 구멍은 부모에서 처리
            continue
        if cv2.contourArea(c) < min_area * UP * UP:
            continue
        # 점이 UP 배 촘촘하므로 솎아낸다(약 0.5칸 간격). 안 솎으면
        # 점이 수천 개가 되어 매 프레임 QPainterPath 구성이 무거워진다.
        stride = max(1, UP // 2)
        outer = chaikin(c[::stride, 0, :] / UP, chaikin_iters)
        holes = []
        j = hier[i][2]                  # 첫 자식
        while j != -1:
            if cv2.contourArea(cnts[j]) >= min_area * UP * UP:
                holes.append(chaikin(cnts[j][::stride, 0, :] / UP, chaikin_iters))
            j = hier[j][0]
        out.append((outer, holes))
    return out


# ── 위젯 ────────────────────────────────────────────────────────
class MapCanvas(QWidget):
    """set_data() 로 격자와 상태를 넣으면 벡터로 그린다."""

    def __init__(self, cell_m=0.05, show_axes=True, show_legend=True,
                 show_cells=False):
        super().__init__()
        self.cell_m = cell_m
        self.show_axes = show_axes
        self.show_legend = show_legend
        self.show_cells = show_cells      # 디버그: 원시 격자 오버레이
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self.setMinimumSize(320, 320)

        self.grid = None
        self.robot = None            # (x, y) 셀 좌표
        self.heading = 0.0
        self.trail = []
        self.path = []
        self.hazards = []            # [{"kind","x","y","r","level"}]
        self.rescue_target = None    # (x, y) 셀 좌표
        self.rescue_path_safe = []   # [(x,y), ...] 안전 경로 (고온+가스 회피)
        self.rescue_path_risky = []  # [(x,y), ...] 위험감수 경로 (가스 경유)
        self._cache_key = None
        self._polys = {}
        self._hazard_cache_key = None
        self._hazard_cell_polys = []  # [(hz, smoothed_cell_pts), ...] 캐시
        self._legend_rows = []        # 줄바꿈된 범례 항목 (paintEvent 에서 채움)
        self.LEGEND_H = self.LEGEND_PAD_TOP + self.LEGEND_ROW_H   # 첫 프레임 전 기본값

    def set_data(self, grid, robot=None, heading=0.0, trail=None,
                 path=None, hazards=None, rescue_target=None,
                 rescue_path_safe=None, rescue_path_risky=None):
        self.grid = np.asarray(grid)
        self.robot = robot
        self.heading = heading
        self.trail = trail or []
        self.path = path or []
        self.hazards = hazards or []
        self.rescue_target = rescue_target
        self.rescue_path_safe = rescue_path_safe or []
        self.rescue_path_risky = rescue_path_risky or []
        self.update()

    # ---- 좌표 변환 ------------------------------------------------
    LEGEND_ROW_H = 20      # 범례 한 줄 높이
    LEGEND_PAD_TOP = 10    # 범례 블록과 지도 사이 여백

    def _setup_transform(self):
        n = self.grid.shape[0]
        pad_l = 46 if self.show_axes else 8
        pad_b = 30 if self.show_axes else 8
        pad_r = 10
        # ★ 범례를 지도 안에 겹쳐 그리면 탐사 영역을 가린다.
        #   위쪽에 자리를 따로 내주고 지도를 그만큼 내린다.
        pad_t = (10 + self.LEGEND_H) if self.show_legend else 10
        w = self.width() - pad_l - pad_r
        h = self.height() - pad_t - pad_b
        s = min(w, h) / n
        ox = pad_l + (w - s * n) / 2
        oy = pad_t + (h - s * n) / 2
        return s, ox, oy, n

    def _pt(self, cx, cy):
        """셀 좌표 → 화면 좌표. y 는 위로 증가하므로 여기서 뒤집는다.
        ★ 축 뒤집기는 이 함수 한 곳에서만 한다 — 여러 곳에 흩어지면
          히트맵과 지도가 어긋나는 종류의 버그가 난다."""
        s, ox, oy, n = self._tf
        return QPointF(ox + cx * s, oy + (n - cy) * s)

    # ---- 그리기 ---------------------------------------------------
    def paintEvent(self, ev):
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing, True)
        p.fillRect(self.rect(), C_BG)
        if self.grid is None:
            return

        if self.show_legend:
            self._compute_legend_layout(p)     # LEGEND_H 를 줄 수에 맞춰 갱신
        self._tf = self._setup_transform()
        s, ox, oy, n = self._tf

        # 미탐색 배경
        p.fillRect(QRectF(ox, oy, s * n, s * n), C_UNKNOWN)

        self._update_polys()

        # 1) 자유공간
        self._fill(p, self._polys.get("free", []), C_FREE,
                   QPen(C_FREE_EDGE, 1.2))
        # 2) 가스 위험구역 — ★배경 레이어. 자유공간 바로 위, 장애물/경로보다
        #    먼저 그려서 항상 그 밑에 깔리게 한다(가시성 확보 - 사용자 요청).
        #    윤곽선 없이 빗금 채움만.
        self._draw_hazards_bg(p)
        # 3) 확인 불가
        self._fill(p, self._polys.get("pink", []), C_PINK, Qt.NoPen)
        # 4) 장애물 (볼록껍질로 확정된 것 + 직접관측 점 모두 포함)
        self._fill(p, self._polys.get("obst", []), C_OBST,
                   QPen(C_OBST_EDGE, 1.4))

        self._draw_cells(p)          # 디버그 오버레이 (기본 꺼짐)
        self._draw_hazards_fg(p)     # 불(원 하나) — 장애물 위, 앞쪽 강조
        self._draw_axes(p)
        self._draw_trail(p)
        self._draw_path(p)
        self._draw_rescue(p)
        self._draw_robot(p)
        if self.show_legend:
            self._draw_legend(p)
        p.end()

    def _update_polys(self):
        g = self.grid
        key = hash(g.tobytes())
        if key == self._cache_key:
            return                        # 격자가 안 변했으면 재사용
        self._cache_key = key
        self._polys = {
            "free":   mask_to_polys(g == 0, min_area=3),
            "obst":   mask_to_polys(np.isin(g, [1, 3]), min_area=2),
            "pink":   mask_to_polys(g == 4, min_area=2),
        }

    def _fill(self, p, polys, color, pen):
        if not polys:
            return
        p.setPen(pen)
        p.setBrush(QBrush(color))
        for outer, holes in polys:
            path = QPainterPath()
            path.addPolygon(QPolygonF([self._pt(x, y) for x, y in outer]))
            path.closeSubpath()
            for h in holes:
                sub = QPainterPath()
                sub.addPolygon(QPolygonF([self._pt(x, y) for x, y in h]))
                sub.closeSubpath()
                path = path.subtracted(sub)
            p.drawPath(path)

    def _rebuild_hazard_polys(self):
        """위험구역 얼룩 형태(chaikin+jitter)를 셀 좌표로 미리 계산해 캐싱한다.

        ★ 장애물 윤곽(_update_polys)은 원래 캐싱돼 있었는데 위험구역은
          빠져 있어서, 매 paintEvent 마다 rng+chaikin 을 새로 돌리고 있었다.
          값(x,y,r,level)이 실제로 안 바뀌면 재계산할 이유가 없다."""
        key = tuple((hz["kind"], round(hz["x"], 2), round(hz["y"], 2),
                    round(hz.get("r", 5), 2), round(hz.get("level", 1.0), 2))
                   for hz in self.hazards)
        if key == self._hazard_cache_key:
            return
        self._hazard_cache_key = key

        out = []
        for hz in self.hazards:
            cx, cy = hz["x"], hz["y"]
            r = max(hz.get("r", 5), 2.0)
            seed = int(abs(cx * 97 + cy * 31) * 1000) & 0xFFFF
            rng = np.random.default_rng(seed)
            n_pts = 10
            angs = np.linspace(0, 2 * math.pi, n_pts, endpoint=False)
            jitter = rng.uniform(0.82, 1.15, n_pts)
            raw = [(cx + r * jitter[i] * math.cos(a),
                   cy + r * jitter[i] * math.sin(a))
                  for i, a in enumerate(angs)]
            smooth = chaikin(raw, iters=2, closed=True)
            out.append((hz, smooth))
        self._hazard_cell_polys = out

    def _draw_hazards_bg(self, p):
        """가스 위험구역 - 배경 레이어. 채움+빗금만, 윤곽선 없음.

        ★사용자 요청: 윤곽선을 없애고 배경으로 깔아서, 장애물/경로 같은
        전경 요소가 항상 또렷하게 보이게 한다. paintEvent 에서 자유공간
        바로 다음, 장애물보다 먼저 그린다."""
        self._rebuild_hazard_polys()
        for hz, smooth in self._hazard_cell_polys:
            if hz["kind"] == "gas":
                self._paint_hazard_blob(p, hz, smooth, outline=False)

    def _draw_hazards_fg(self, p):
        """불(화재) 위험구역 - 앞쪽 레이어, 원 하나. sim_bridge._hazards() 가
        이미 '원 하나'로 병합해서 넘긴다(온도 80도 이상 지점을 전부 모은
        하나의 원) - 여기서는 그걸 그대로 그리기만 한다."""
        self._rebuild_hazard_polys()
        for hz, smooth in self._hazard_cell_polys:
            if hz["kind"] == "fire":
                self._paint_hazard_blob(p, hz, smooth, outline=True)

    def _paint_hazard_blob(self, p, hz, smooth, outline=True):
        """위험구역 하나를 그린다: 옅은 채움 -> 대각선 해칭 -> (선택) 윤곽선.

        그리는 순서가 중요하다: 해칭을 마지막에 그리면 테두리 위까지
        삐져나가므로, 윤곽선을 그릴 때는 반드시 해칭 다음(채움 위)에 그린다.
        outline=False(가스, 배경용)면 3단계를 건너뛴다 - 라벨도 안 그린다
        (배경이라 시선을 끌 필요가 없다, 사용자 요청)."""
        r = max(hz.get("r", 5), 2.0)
        level = min(hz.get("level", 1.0), 1.0)
        is_fire = hz["kind"] == "fire"
        base = QColor(224, 90, 60) if is_fire else QColor(214, 176, 60)
        pts = [self._pt(x, y) for x, y in smooth]

        path = QPainterPath()
        path.moveTo(pts[0])
        for q in pts[1:]:
            path.lineTo(q)
        path.closeSubpath()

        # 1) 옅은 채움
        fill_a = int(70 * (0.4 + 0.6 * level))
        p.setPen(Qt.NoPen)
        p.setBrush(QBrush(QColor(base.red(), base.green(), base.blue(), fill_a)))
        p.drawPath(path)

        # 2) 대각선 해칭 — 윤곽 안쪽으로 클리핑한 뒤 45도 평행선을 긋는다
        p.save()
        p.setClipPath(path)
        hatch_pen = QPen(QColor(base.red(), base.green(), base.blue(), 100), 1.0)
        p.setPen(hatch_pen)
        bbox = path.boundingRect()
        step = 6.0
        diag = bbox.height()
        x = bbox.left() - diag
        x_end = bbox.right() + diag
        while x < x_end:
            p.drawLine(QPointF(x, bbox.top()), QPointF(x + diag, bbox.bottom()))
            x += step
        p.restore()

        if not outline:
            return

        # 3) 윤곽선 (불만 - 가스는 배경이라 생략)
        p.setPen(QPen(base, 1.6))
        p.setBrush(Qt.NoBrush)
        p.drawPath(path)

        f = QFont(); f.setPointSize(7); f.setBold(True); p.setFont(f)
        p.setPen(base)
        label = "FIRE" if is_fire else "GAS"
        anchor = self._pt(hz["x"], hz["y"] + r * 1.15)
        p.drawText(anchor, label)

    def _draw_axes(self, p):
        if not self.show_axes:
            return
        s, ox, oy, n = self._tf
        half_m = n * self.cell_m / 2
        p.setPen(QPen(C_AXIS, 1))
        f = QFont(); f.setPointSize(7); p.setFont(f)

        # 눈금 간격을 1m 또는 0.5m 로
        stepm = 1.0 if half_m > 1.6 else 0.5
        v = -math.floor(half_m / stepm) * stepm
        while v <= half_m + 1e-6:
            cx = n / 2 + v / self.cell_m
            pt = self._pt(cx, 0)
            p.setPen(QPen(C_AXIS, 1, Qt.DotLine))
            p.drawLine(QPointF(pt.x(), oy), QPointF(pt.x(), oy + s * n))
            p.setPen(C_TEXT)
            p.drawText(QRectF(pt.x() - 18, oy + s * n + 3, 36, 14),
                       Qt.AlignCenter, f"{v:g}")

            pt2 = self._pt(0, cx)
            p.setPen(QPen(C_AXIS, 1, Qt.DotLine))
            p.drawLine(QPointF(ox, pt2.y()), QPointF(ox + s * n, pt2.y()))
            p.setPen(C_TEXT)
            p.drawText(QRectF(ox - 40, pt2.y() - 7, 34, 14),
                       Qt.AlignRight | Qt.AlignVCenter, f"{v:g}")
            v += stepm

        p.setPen(C_TEXT)
        p.drawText(QRectF(ox, oy + s * n + 15, s * n, 14),
                   Qt.AlignCenter, "X (m)")
        p.save()
        p.translate(ox - 34, oy + s * n / 2)
        p.rotate(-90)
        p.drawText(QRectF(-40, -8, 80, 14), Qt.AlignCenter, "Y (m)")
        p.restore()

    def _draw_trail(self, p):
        if len(self.trail) < 2:
            return
        p.setPen(QPen(C_TRAIL, 1.6, Qt.DotLine))
        p.setBrush(Qt.NoBrush)
        path = QPainterPath()
        pts = [self._pt(x, y) for x, y in self.trail[-800:]]
        path.moveTo(pts[0])
        for q in pts[1:]:
            path.lineTo(q)
        p.drawPath(path)

    def _draw_path(self, p):
        if len(self.path) < 2:
            return
        pts = [self._pt(x, y) for x, y in self.path]
        p.setPen(QPen(C_PATH, 2.2))
        p.setBrush(Qt.NoBrush)
        path = QPainterPath()
        path.moveTo(pts[0])
        for q in pts[1:]:
            path.lineTo(q)
        p.drawPath(path)
        p.setPen(Qt.NoPen)
        p.setBrush(QBrush(C_PATH))
        p.drawEllipse(pts[-1], 4, 4)

    def _draw_rescue(self, p):
        """요구조자 위치(SOS 마커) + 두 후보 경로(안전/위험감수).

        계획 경로(_draw_path, 초록 실선 - 탐사 중 다음 프런티어로 가는 경로)와
        용도가 다르므로 색/선 스타일을 확실히 구분한다(점선) - 둘 다
        초록 계열이면 화면에서 헷갈린다. 안전경로를 위험감수경로보다
        나중에(위에) 그려서 겹칠 때 안전경로가 우선적으로 보이게 한다."""
        def draw_line(cells, color):
            if len(cells) < 2:
                return
            pts = [self._pt(x, y) for x, y in cells]
            p.setPen(QPen(color, 2.4, Qt.DashLine))
            p.setBrush(Qt.NoBrush)
            path = QPainterPath()
            path.moveTo(pts[0])
            for q in pts[1:]:
                path.lineTo(q)
            p.drawPath(path)

        draw_line(self.rescue_path_risky, C_RESCUE_RISKY)
        draw_line(self.rescue_path_safe, C_RESCUE_SAFE)

        if self.rescue_target is not None:
            c = self._pt(*self.rescue_target)
            p.setPen(QPen(QColor("#ffffff"), 1.4))
            p.setBrush(QBrush(C_RESCUE_MARK))
            p.drawEllipse(c, 7, 7)
            f = QFont(); f.setPointSize(7); f.setBold(True); p.setFont(f)
            p.setPen(C_RESCUE_MARK)
            p.drawText(QRectF(c.x() - 22, c.y() - 22, 44, 14),
                      Qt.AlignCenter, "SOS")

    def _draw_robot(self, p):
        if not self.robot:
            return
        c = self._pt(*self.robot)
        a = math.radians(-self.heading)      # 화면 y 가 아래로 증가
        r = 7
        tri = QPolygonF([
            QPointF(c.x() + r * math.cos(a), c.y() + r * math.sin(a)),
            QPointF(c.x() + r * math.cos(a + 2.5), c.y() + r * math.sin(a + 2.5)),
            QPointF(c.x() + r * math.cos(a - 2.5), c.y() + r * math.sin(a - 2.5)),
        ])
        p.setPen(QPen(QColor("#ffffff"), 1.2))
        p.setBrush(QBrush(C_ROBOT))
        p.drawPolygon(tri)

    def _legend_items(self):
        """범례 항목 목록. 요구조자/구조경로는 계획 경로 바로 옆에 둔다 -
        '경로' 계열 항목끼리 묶여 보이도록."""
        return [(C_FREE, "자유 공간"), (C_OBST, "장애물"),
                (C_UNKNOWN, "미탐색"), (C_PINK, "확인 불가"),
                (C_TRAIL, "탐색 경로"), (C_PATH, "계획 경로"),
                (C_RESCUE_MARK, "요구조자"),
                (C_RESCUE_SAFE, "구조경로-안전"),
                (C_RESCUE_RISKY, "구조경로-위험감수"),
                (C_ROBOT, "로봇")]

    def _compute_legend_layout(self, p):
        """범례 항목을 폭에 맞춰 여러 줄로 나눈다.

        ★ 예전엔 한 줄에 다 안 들어가면 뒤쪽 항목이 아무 표시 없이 잘렸다
        (스크린샷에서 '로봇' 항목이 안 보였던 원인 - 요구조자를 추가하기
        전부터 있던 문제). 항목 수가 늘어도 전부 보이도록 줄바꿈한다.
        LEGEND_H 는 줄 수에 맞춰 매 프레임 다시 계산되고, _setup_transform
        이 그 값만큼 지도 위 여백을 확보한다."""
        items = self._legend_items()
        pad_l = 46 if self.show_axes else 8
        pad_r = 10
        avail = max(50, self.width() - pad_l - pad_r)

        f = QFont(); f.setPointSize(7)
        p.setFont(f)
        fm = p.fontMetrics()

        rows, cur, x = [], [], 0
        for c, name in items:
            wtxt = fm.horizontalAdvance(name)
            need = 12 + 4 + wtxt + 12
            if cur and x + need > avail:
                rows.append(cur)
                cur, x = [], 0
            cur.append((c, name, wtxt))
            x += need
        if cur:
            rows.append(cur)

        self._legend_rows = rows
        self.LEGEND_H = self.LEGEND_PAD_TOP + max(1, len(rows)) * self.LEGEND_ROW_H

    def _draw_legend(self, p):
        """지도 위쪽 바깥에 여러 줄로. 지도를 가리지 않는다."""
        s_, ox, oy, n = self._tf
        f = QFont(); f.setPointSize(7); p.setFont(f)
        top = oy - self.LEGEND_H + self.LEGEND_PAD_TOP
        for row_i, row in enumerate(self._legend_rows):
            y = top + row_i * self.LEGEND_ROW_H
            x = ox
            for c, name, wtxt in row:
                p.setPen(QPen(QColor("#4a5262"), 0.8))
                p.setBrush(QBrush(c))
                p.drawRoundedRect(QRectF(x, y + 3, 9, 9), 2, 2)
                p.setPen(C_TEXT)
                p.drawText(QRectF(x + 13, y - 2, wtxt + 6, self.LEGEND_ROW_H),
                          Qt.AlignLeft | Qt.AlignVCenter, name)
                x += 12 + 4 + wtxt + 12

    def _draw_cells(self, p):
        """디버그용 원시 격자 오버레이.

        ★ 왜 필요한가: 부드러운 벡터 렌더링은 오도메트리 오차를 감춘다.
          벽이 두 겹으로 잡히거나 통로가 어긋나도 매끈하게 그려지면
          '잘 되고 있는 것처럼' 보인다. 실물 검증 때는 켜서 볼 것.
        """
        if not self.show_cells:
            return
        s_, ox, oy, n = self._tf
        g = self.grid
        p.setPen(Qt.NoPen)
        for v, col in ((3, QColor(232, 106, 52, 90)),
                       (1, QColor(120, 140, 180, 60))):
            ys, xs = np.where(g == v)
            for x, y in zip(xs.tolist(), ys.tolist()):
                q = self._pt(x, y + 1)
                p.fillRect(QRectF(q.x(), q.y(), s_, s_), col)
        # 격자선
        p.setPen(QPen(QColor(255, 255, 255, 18), 0.5))
        for i in range(0, n + 1, 5):
            p.drawLine(self._pt(i, 0), self._pt(i, n))
            p.drawLine(self._pt(0, i), self._pt(n, i))
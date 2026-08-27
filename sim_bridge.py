"""
sim_bridge.py — 탐사 시뮬레이터를 대시보드 2D 맵에 연결

────────────────────────────────────────────────────────────────
[연결 방식]
  path_planner_sim2.py 는 시각화를 _Viz 클래스에 위임하고,
  sim_pygame.py 가 `S._Viz = PygameViz` 로 교체하는 구조다.
  여기서도 같은 자리를 쓴다:

      import path_planner_sim2 as S
      S._Viz = DashViz(state)      # RobotState 에 그려 넣는다
      S.run(seed, visualize=True)

  시뮬 코드는 한 줄도 안 고친다.

[path_planner_sim2 가 없을 때]
  MiniExplorer 로 자동 대체된다. 프런티어 탐사 + 광선 스캔만 있는
  축소판이라 화면 확인 용도로는 충분하다.

[가스 / 온도]
  ScalarField 가 지도 위에 가우시안 발생원을 깔아둔다.
  ★ 로봇이 그 칸을 지날 때만 값이 기록된다 — 실제 접촉식 센서와 같다.
    ToF 처럼 멀리서 읽히지 않으므로 격자가 궤적을 따라서만 찬다.
────────────────────────────────────────────────────────────────
"""
from __future__ import annotations
import math
import random
import time

import numpy as np

try:
    import robot_config as _RC
except ImportError:
    _RC = None
FIRE_TEMP_C = getattr(_RC, "FIRE_TEMP_C", 50.0)
GAS_PPM = getattr(_RC, "GAS_PPM", 40.0)

def _cluster_points(points, gap):
    """points: [(x, y, v), ...]. gap 칸 이내로 연결된(전이적으로도) 점들을
    한 그룹으로 묶는다(union-find)."""
    n = len(points)
    parent = list(range(n))

    def find(i):
        while parent[i] != i:
            parent[i] = parent[parent[i]]
            i = parent[i]
        return i

    for i in range(n):
        for j in range(i + 1, n):
            if math.hypot(points[i][0] - points[j][0],
                         points[i][1] - points[j][1]) < gap:
                ri, rj = find(i), find(j)
                if ri != rj:
                    parent[ri] = rj

    groups = {}
    for i in range(n):
        groups.setdefault(find(i), []).append(points[i])
    return list(groups.values())

# ═══════════════════════════════════════════════════════════════
#  1. 격자 → RGB 렌더링
# ═══════════════════════════════════════════════════════════════
#
# path_planner_sim2 의 grid 값 체계:
#   -1 미탐색 / 0 빈공간 / 1 확정채움 / 2 미확정채움 /
#    3 직접관측 / 4 확인불가
#
# 대시보드가 어두운 테마라 원본 색(회색/흰/검정/노랑/주황/핑크)을
# 그대로 쓰면 눈이 아프다. 명도만 뒤집어 어두운 배경에 맞춘다.

PALETTE = {
    -1: (26, 32, 44),      # 미탐색 — 배경보다 살짝 밝은 남색
     0: (206, 214, 224),   # 빈공간 — 밝은 회백
     1: (18, 22, 30),      # 확정 채움 — 거의 검정
     2: (198, 158, 46),    # ★미확정 상태 삭제됨(path_planner_sim2.py) - grid 에
                           #   이제 이 값이 안 나오지만, PALETTE 딕셔너리 자체는
                           #   render_grid()가 임의의 값 체계를 받을 수 있게
                           #   value->color 매핑을 완전하게 유지하는 게 안전하다.
     3: (232, 106, 52),    # 직접 관측 — 주황
     4: (150, 70, 110),    # 확인 불가 — 자주
}


def render_grid(grid, robot=None, path=None, frontiers=None,
                scale=8, trail=None, goal=None):
    """격자를 RGB 배열로. dashboard.MapView 가 그대로 받는다.

    scale: 셀 하나를 몇 픽셀로 그릴지. 60x60 격자에 8이면 480px.
    goal: 지금 노리는 목표 지점 (target). 작은 초록 고리로 표시.
    """
    g = np.asarray(grid)
    h, w = g.shape
    img = np.zeros((h, w, 3), np.uint8)
    for v, c in PALETTE.items():
        img[g == v] = c

    # 셀 확대 (보간 없이 — 격자가 뭉개지면 안 된다)
    img = np.repeat(np.repeat(img, scale, axis=0), scale, axis=1)

    def dot(cx, cy, r, color):
        y0, y1 = max(0, cy - r), min(img.shape[0], cy + r + 1)
        x0, x1 = max(0, cx - r), min(img.shape[1], cx + r + 1)
        yy, xx = np.mgrid[y0:y1, x0:x1]
        m = (xx - cx) ** 2 + (yy - cy) ** 2 <= r * r
        img[y0:y1, x0:x1][m] = color

    if trail:
        for (x, y) in trail[-400:]:
            dot(int(x * scale + scale / 2), int(y * scale + scale / 2),
                max(1, scale // 4), (70, 130, 200))

    if frontiers:
        for (x, y) in frontiers[:600]:
            dot(int(x * scale + scale / 2), int(y * scale + scale / 2),
                max(1, scale // 5), (60, 150, 220))

    if path:
        for (x, y) in path:
            dot(int(x * scale + scale / 2), int(y * scale + scale / 2),
                max(1, scale // 4), (80, 200, 120))

    if goal:
        gx, gy = goal
        dot(int(gx * scale + scale / 2), int(gy * scale + scale / 2),
            max(2, scale // 3), (60, 220, 140))

    if robot:
        rx, ry = robot
        dot(int(rx * scale + scale / 2), int(ry * scale + scale / 2),
            max(2, scale // 2), (240, 80, 70))

    # 위아래 뒤집기: 시뮬은 origin='lower', 이미지는 위가 0행
    return np.ascontiguousarray(img[::-1])


# ═══════════════════════════════════════════════════════════════
#  2. 가상 가스 / 온도 필드
# ═══════════════════════════════════════════════════════════════
class ScalarField:
    """지도 위에 발생원 몇 개를 깔고, 그 지점에서 퍼져나가는 값을 만든다.

    ★ 핵심: sample() 은 '로봇이 그 칸에 있을 때만' 호출된다.
      온도·가스는 ToF 와 달리 접촉식이라 원격 측정이 안 된다.
      그래서 격자가 목업처럼 꽉 차지 않고 궤적을 따라서만 찬다.
      이게 버그가 아니라 실제 하드웨어의 성질이다.
    """

    def __init__(self, size, base, peak, n_src=2, sigma=9.0, seed=0):
        rng = random.Random(seed)
        self.size = size
        self.base = base
        self.peak = peak
        self.sigma = sigma
        self.sources = [(rng.uniform(size * .2, size * .8),
                         rng.uniform(size * .2, size * .8),
                         rng.uniform(.6, 1.0)) for _ in range(n_src)]

    def value_at(self, x, y):
        v = self.base
        for sx, sy, w in self.sources:
            d2 = (x - sx) ** 2 + (y - sy) ** 2
            v += (self.peak - self.base) * w * math.exp(-d2 / (2 * self.sigma ** 2))
        return v

    def sample(self, x, y, noise=0.6, rng=None):
        v = self.value_at(x, y)
        if rng is not None:
            v += rng.gauss(0, noise)
        return v

    def to_display_grid(self, measured, n_out):
        """측정된 셀들을 대시보드 격자(n_out x n_out)로 축약.
        measured: {(gx, gy): value}. 없는 칸은 NaN 으로 남는다.

        ★ 행 순서를 뒤집는다.
          시뮬 좌표계는 y 가 위로 증가(origin='lower')하는데,
          이미지/위젯은 0행이 맨 위다. render_grid 도 마지막에
          img[::-1] 로 뒤집으므로, 히트맵만 안 뒤집으면
          지도와 위아래가 반대로 나타난다."""
        out = np.full((n_out, n_out), np.nan)
        acc = {}
        k = self.size / n_out
        for (gx, gy), v in measured.items():
            cx, cy = int(gx / k), int(gy / k)
            if 0 <= cx < n_out and 0 <= cy < n_out:
                acc.setdefault((n_out - 1 - cy, cx), []).append(v)   # ★ 뒤집기
        for (r, c), vs in acc.items():
            out[r][c] = sum(vs) / len(vs)
        return out


def grid_status_to_display(grid, n_out):
    """path_planner_sim2.grid(60x60, 값 -1/0/1/3/4)를 히트맵 해상도
    (n_out x n_out)로 축약한 '상태' 격자. HeatGrid 가 온도/가스 값이 없는
    칸(NaN)을 전부 똑같은 회색으로 칠하던 걸, 이유별로 구분하기 위함이다
    - "아직 안 가봤다"(계속 회색)와 "장애물이라 절대 못 잰다"/"가려서
    확인을 포기했다"(새 색)는 다른 정보다.

    ScalarField.to_display_grid() 와 동일한 좌표 규칙(행 뒤집기)을 쓴다 -
    두 함수가 만드는 격자가 같은 위젯에 겹쳐 쓰이므로 규칙이 어긋나면
    지도와 히트맵이 서로 다른 방향으로 뒤집힌다.

    반환값: n_out x n_out int8 배열. 0=해당 없음(free/미탐색) 1=장애물
    2=확인 불가(가려서 못 봄). 한 히트맵 칸 안에 원본 칸이 여러 개
    뭉치므로, 그중 하나라도 장애물/확인불가면 그 히트맵 칸 전체를 그
    상태로 표시한다(과반수 대신 '하나라도' - 위험을 놓치지 않는 쪽으로
    보수적으로 잡음). 장애물이 확인불가보다 우선한다(더 확실한 정보)."""
    n = grid.shape[0]
    k = n / n_out
    out = np.zeros((n_out, n_out), dtype=np.int8)
    ys, xs = np.where(np.isin(grid, [1, 3, 4]))
    for gy, gx in zip(ys.tolist(), xs.tolist()):
        cx, cy = int(gx / k), int(gy / k)
        if not (0 <= cx < n_out and 0 <= cy < n_out):
            continue
        r = n_out - 1 - cy   # ★ to_display_grid 와 동일한 뒤집기
        cat = 1 if grid[gy][gx] in (1, 3) else 2
        if out[r][cx] < cat:
            out[r][cx] = cat
    return out


# ═══════════════════════════════════════════════════════════════
#  3. path_planner_sim2 용 뷰 어댑터
# ═══════════════════════════════════════════════════════════════
class DashViz:
    """S._Viz 자리에 끼우는 클래스.

    사용:
        import path_planner_sim2 as S
        S._Viz = make_dash_viz(state)      # 팩토리로 state 를 묶는다
        S.run(seed, visualize=True)

    _Viz 는 S.run 안에서 `_Viz(seed)` 로 생성되므로 생성자 인자가
    seed 하나여야 한다. 그래서 state 는 클래스 속성으로 미리 넣는다.
    """
    state = None          # make_dash_viz 가 채운다
    scale = 8
    grid_n = 12           # 대시보드 히트맵 격자 크기
    min_interval_s = 0.08  # ★ 이 이상 빠르게는 갱신 안 함 (대략 12Hz 상한)
    RESCUE_REPLAN_EVERY = 5   # 요구조자 발견 후 이 스텝마다 경로 재계산
    FIRE_CIRCLE_TEMP_C = FIRE_TEMP_C  # ★불(원) 표시 임계값.
                               #   [근거] 맨몸 인간 기준 - 습도 낮은 공기는
                               #   짧은 시간이면 고온도 버티지만(사우나
                               #   80~100도/10~20분), 그건 "탈출 가능+휴식"
                               #   전제다. 지속 노출·부상·탈수가 겹치면
                               #   훨씬 낮은 온도에서도 위험해진다. 산업
                               #   열스트레스 기준(WBGT)은 습도 반영 시
                               #   32~34도부터 위험 취급 - 요구조자는
                               #   스스로 못 움직일 수 있으니 그보다 보수적
                               #   으로, "건강한 사람도 짧게만 버티고 부상자
                               #   면 명백히 위험한" 50도로 잡았다.
                               #   [설계] rescue_planner.FIRE_TEMP_C(경로
                               #   회피 임계값)와 값을 맞췄다 - 카드 "위험"
                               #   표시 / 지도 화재원 / 실제 경로 회피가
                               #   전부 50도에서 함께 발동한다. 예전엔 이
                               #   값(당시 40도, 그 전엔 80도)이 회피
                               #   임계값과 따로 놀았는데, 그럴 이유가
                               #   딱히 없어서 통일했다 - 바꾸려면 여기와
                               #   rescue_planner.py, 그리고 아래 dashboard.py
                               #   의 온도 카드 danger 임계값을 같이 바꿀 것.

    def __init__(self, seed):
        import path_planner_sim2 as S
        self.S = S
        self.trail = []
        self.measured_t = {}
        self.measured_g = {}
        self.rng = random.Random(seed + 991)
        self._last_update = 0.0
        # 요구조자는 이제 path_planner_sim2.rescuee_truth 가 정한다(정답).
        # 여기서는 마지막으로 경로를 다시 계산한 스텝만 추적한다.
        self._rescue_last_plan_step = -10 ** 9
        self._rescue_error_shown = False   # 계산 오류 경고를 한 번만 찍기 위한 플래그
        n = S.GRID_SIZE
        self.f_temp = ScalarField(n, base=24.0, peak=68.0, n_src=2,
                                  sigma=8.0, seed=seed)
        self.f_gas = ScalarField(n, base=4.0, peak=72.0, n_src=2,
                                 sigma=7.0, seed=seed + 7)

    # --- S.run 이 호출하는 두 메서드 ---
    def progress(self, target, frontiers, cleanup_phase):
        # ★ 렉의 진짜 원인: 시뮬 스레드가 쉬지 않고 무거운 계산(다익스트라,
        #   클러스터링, 형태추정)을 돌리면서 파이썬 GIL 을 계속 붙잡고 있으면,
        #   화면을 그리는 Qt 메인 스레드가 순서를 못 받아 멈춘 것처럼 보인다.
        #   대시보드는 100ms(10Hz)마다 한 번만 state 를 읽는데, 시뮬은 그보다
        #   훨씬 빠르게 여러 번 progress() 를 부를 수 있다 — 그 초과분은
        #   화면에 반영도 안 되고 GIL 만 뺏는 순수 낭비다.
        #   여기서 최소 간격만큼 쉬어주면 GIL 이 자연히 Qt 스레드로 넘어간다.
        now = time.perf_counter()
        if now - self._last_update < self.min_interval_s:
            time.sleep(max(0.0, self.min_interval_s - (now - self._last_update)))
            return
        self._last_update = time.perf_counter()

        # [방어] progress() 는 S.run() 메인 루프 안에서 매 스텝 호출된다.
        # 여기(대시보드 어댑터 계층)에서 예외가 새면 S.run() 전체가 죽고
        # (_start_sim2._run() 이 잡아서 프린트만 하고 조용히 끝남), 그 뒤로
        # 어떤 갱신도 안 와서 화면이 멈춘 것처럼 보인다 - 실제로 이런
        # 사고가 났었다(rescue_planner.py 구버전 파일 문제로 progress()
        # 안에서 AttributeError 발생 -> 요구조자 발견 직후 멈춤). 원인이
        # 뭐든 이 어댑터 계층의 버그 하나가 핵심 시뮬레이션(탐사/지도작성)
        # 을 멈추게 해선 안 되므로, 실제 갱신 로직은 _progress_impl 로
        # 빼고 여기서 예외를 잡아 로그만 남긴다.
        try:
            self._progress_impl(target, frontiers, cleanup_phase)
        except Exception as e:                        # noqa: BLE001
            print(f"[DashViz] progress 갱신 중 오류 - 이번 프레임은 "
                  f"건너뜀(시뮬레이션은 계속됨): {type(e).__name__}: {e}")

    def _progress_impl(self, target, frontiers, cleanup_phase):
        S, st = self.S, self.state
        rx, ry = S.robot_x, S.robot_y
        self.trail.append((rx, ry))

        # 지나간 칸에서만 측정
        gx, gy = int(rx), int(ry)
        self.measured_t[(gx, gy)] = self.f_temp.sample(rx, ry, 0.7, self.rng)
        self.measured_g[(gx, gy)] = self.f_gas.sample(rx, ry, 1.5, self.rng)

        # ★ MapCanvas(벡터 렌더러)에 원본 격자를 직접 넘긴다.
        #   path_planner_sim2 의 grid 값 체계(-1/0/1/2/3/4)가 MapCanvas 가
        #   기대하는 것과 동일하므로 변환이 필요 없다.
        #   Dashboard._refresh 는 state.map_grid 가 있으면 map_canvas(벡터)를
        #   우선 쓰고 map_image(래스터)는 무시하므로, render_grid 호출 자체를
        #   생략해 매 스텝 불필요한 래스터 렌더링 비용도 없앤다.
        st.map_grid = S.grid.copy()
        st.robot_cell = (rx, ry)
        st.heading = S.robot_angle
        st.trail = self.trail
        # 실제 경로 전체는 S.run() 내부 지역변수라 밖에서 못 읽는다.
        # 로봇->목표 직선으로 방향만 근사해서 그린다 (정확한 경로가 아님).
        st.plan_path = [(rx, ry), target] if target else []

        st.temp_c = self.measured_t[(gx, gy)]
        st.gas_ppm = self.measured_g[(gx, gy)]
        st.temp_grid = self.f_temp.to_display_grid(self.measured_t, self.grid_n)
        st.gas_grid = self.f_gas.to_display_grid(self.measured_g, self.grid_n)
        # 히트맵에 '장애물이라 못 잰다'/'가려서 확인 포기'를 구분 표시하기
        # 위한 상태 격자. 온도/가스 둘 다 같은 grid 에서 나오므로 공용.
        st.block_grid = grid_status_to_display(S.grid, self.grid_n)

        # ToF: 헤딩 방향으로 관측 격자(값 3)까지의 거리. truth 가 아니라
        # 로봇이 실제로 관측한 grid 만 본다 — 이게 실물 센서가 읽는 값이다.
        st.dist_mm, st.dist_conf = self._front_dist_mm(S, rx, ry)

        # 위험 구역: 임계값 이상 측정된 지점만. 안 가본 곳의 발생원은
        # 표시되지 않는다 — 접촉식 센서라 실제로 그렇다.
        st.hazards = self._hazards()

        # ── 요구조자: 발견되는 순간 바로 지도에 표시하고, 이후로는
        #    RESCUE_REPLAN_EVERY 스텝마다 경로를 다시 계산한다. 탐사가
        #    진행될수록 grid/measured_t/g 커버리지가 넓어지므로, 경로도
        #    같이 갱신돼야 처음엔 없던 우회로가 나중에 반영된다. ──
        if S.rescuee_discovered:
            if st.rescue_target is None:
                st.rescue_target = S.rescuee_truth
                print(f"[구조] 요구조자 발견! 위치 {S.rescuee_truth} "
                      f"(스텝 {S.rescuee_discover_step})")
                self._update_rescue_paths()
                self._rescue_last_plan_step = S.step
            elif S.step - self._rescue_last_plan_step >= self.RESCUE_REPLAN_EVERY:
                self._update_rescue_paths()
                self._rescue_last_plan_step = S.step

        st.pos = ((rx - S.GRID_SIZE / 2) * S.CELL_SIZE_M,
                  (ry - S.GRID_SIZE / 2) * S.CELL_SIZE_M)
        if target:
            st.goal = ((target[0] - S.GRID_SIZE / 2) * S.CELL_SIZE_M,
                       (target[1] - S.GRID_SIZE / 2) * S.CELL_SIZE_M)
            st.remain_m = math.hypot(target[0] - rx, target[1] - ry) * S.CELL_SIZE_M
            st.eta_s = st.remain_m / max(S.ROBOT_SPEED_MPS, 1e-6)
        st.path_ok = (S.collisions == 0)
        st.phase = S.phase
        st.step = S.step
        st.push_history()

    def _front_dist_mm(self, S, rx, ry):
        """헤딩 방향 첫 관측 장애물(값 3)까지 거리(mm)와 대략적 신뢰도.
        사거리 끝까지 막힌 게 없으면 '아직 못 본 방향'이라 신뢰도를 낮춘다."""
        ar = math.radians(S.robot_angle)
        ca, sa = math.cos(ar), math.sin(ar)
        g, n = S.grid, S.GRID_SIZE
        for i in range(1, S.SCAN_RANGE + 1):
            x, y = int(rx + i * ca), int(ry + i * sa)
            if not (0 <= x < n and 0 <= y < n):
                break
            if g[y][x] == 3:
                return i * S.CELL_SIZE_M * 1000.0, 0.95
        return S.SCAN_RANGE * S.CELL_SIZE_M * 1000.0, 0.4

    def _hazards(self):
        """측정값이 임계 이상인 지점을 위험 구역으로.

        ★가스: 예전과 동일 - 임계 이상 측정 지점마다 얼룩을 만들고 가까운
          것끼리(거리<9) 병합한다. 로봇 궤적이 흩어져 있으면 여러 개가
          남을 수 있다 - map_canvas.py 가 배경 레이어(윤곽선 없이)로
          그린다.

        ★불: 사용자 요청으로 여러 개 대신 '원 하나'만 그린다. 임계값은
          FIRE_CIRCLE_TEMP_C(50도, 클래스 상단 정의 참고) - 온도가
          FIRE_CIRCLE_TEMP_C 이상인 실측 지점을 전부 모아 그 중심 +
          최대거리로 하나의 원을 만든다. 핫스팟이 지도 위 여러 군데
          떨어져 있어도 원은 하나뿐이라, 그 사이 안 뜨거운 영역까지 원
          안에 포함될 수 있다(경고: 시각적으로 과장될 수 있음 -
          ScalarField 발생원이 2개라 실제로 그런 상황이 생길 수 있다)."""
        out = []
        for (x, y), v in self.measured_g.items():
            if v > GAS_PPM:
                out.append({"kind": "gas", "x": x, "y": y,
                            "r": 4 + (v - GAS_PPM) / 6, "level": (v - GAS_PPM) / 25})
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

        fire_cells = [(x, y, v) for (x, y), v in self.measured_t.items()
                    if v >= self.FIRE_CIRCLE_TEMP_C]
        if fire_cells:
            dominant = max(_cluster_points(fire_cells, gap=9), key=len)
            cx = sum(x for x, y, v in dominant) / len(dominant)
            cy = sum(y for x, y, v in dominant) / len(dominant)
            r = max(math.hypot(x - cx, y - cy) for x, y, v in dominant) + 1.5
            level = min(max((v - self.FIRE_CIRCLE_TEMP_C) / 20
                            for x, y, v in dominant), 1.0)
            merged.append({"kind": "fire", "x": cx, "y": cy,
                        "r": max(r, 3.0), "level": level})

        return merged

    def final(self):
        S, st = self.S, self.state
        try:
            st.map_image = render_grid(S.grid, robot=(S.robot_x, S.robot_y),
                                       scale=self.scale, trail=self.trail)
        except Exception as e:                        # noqa: BLE001
            print(f"[DashViz] final 렌더 중 오류(무시하고 계속): "
                  f"{type(e).__name__}: {e}")
        st.phase = "done"
        # 미션 종료 시점의 완전한 grid/측정 데이터로 마지막 한 번 더
        # 갱신한다 - 중간 갱신 주기(RESCUE_REPLAN_EVERY)에 걸려 최신이
        # 아닐 수 있는 경로를 최종 상태로 맞춘다. (_update_rescue_paths
        # 자체도 내부에서 예외를 잡으므로 여기선 따로 안 감싼다.)
        if S.rescuee_discovered:
            self._update_rescue_paths()

    def _update_rescue_paths(self):
        """발견된 요구조자에 대해 두 경로(안전/위험감수)를 다시 계산해
        state 에 반영한다. ★ S.grid(실제로 그린 지도)와 self.measured_t/g
        (실제로 밟은 칸의 실측값)만 쓴다 - f_temp/f_gas.value_at() 은
        로봇이 모르는 '정답'이라 여기서 참조하면 안 된다
        (rescue_planner.py 상단 docstring, path_planner_sim2.py 의
        정답/로봇로직 분리 원칙과 동일한 이유).

        [방어] 이 함수는 S.run() 메인 루프와 같은 스레드에서 progress()를
        통해 호출된다. 여기서 예외가 새면 S.run() 전체가 죽고(_start_sim2
        의 _run() 이 잡아서 프린트만 하고 조용히 끝남), 그 뒤로 그 어떤
        갱신도 안 와서 화면이 그대로 멈춘 것처럼 보인다 - 실제로
        rescue_planner.py 구버전(plan_rescue_paths 이전 이름)이 로컬에
        남아있어서 이 증상이 났었다. 원인이 뭐든(구버전 파일, 새 버그 등)
        구조경로 계산 하나가 미션 전체를 멈추게 하면 안 되므로, 여기서
        예외를 잡아 로그만 남기고 다음 스텝을 계속 진행한다. 경고는
        RESCUE_REPLAN_EVERY 마다 계속 재시도되므로, 매번 다 찍으면 로그가
        같은 줄로 도배된다 - 처음 한 번만 출력한다."""
        import rescue_planner as RP
        S, st = self.S, self.state
        if S.rescuee_truth is None:
            return
        try:
            path_safe, path_risky = RP.plan_rescue_paths(
                S.grid, self.measured_t, self.measured_g, S.home, S.rescuee_truth)
        except Exception as e:                      # noqa: BLE001
            if not self._rescue_error_shown:
                self._rescue_error_shown = True
                print(f"[구조경로] 계산 중 오류 - 이후 계속 건너뜀(재계산은 "
                      f"계속 시도됨, 로그는 이번만 출력): "
                      f"{type(e).__name__}: {e}")
            return
        st.rescue_path_safe = path_safe or []
        st.rescue_path_risky = path_risky or []


def make_dash_viz(state, scale=8, grid_n=12):
    """state 를 묶은 _Viz 서브클래스를 만들어 반환."""
    return type("BoundDashViz", (DashViz,),
                {"state": state, "scale": scale, "grid_n": grid_n})


# ═══════════════════════════════════════════════════════════════
#  4. 축소판 탐사기 — path_planner_sim2 가 없을 때
# ═══════════════════════════════════════════════════════════════
class MiniExplorer:
    """프런티어 탐사 + 광선 스캔만 있는 최소 구현.

    path_planner_sim2 의 grid 값 체계(-1/0/3)를 그대로 쓰므로
    render_grid 를 공유한다. 형태 추정·클러스터링은 없다.
    """

    def __init__(self, n=60, n_obs=10, seed=7, scan_range=20, fov=130.0,
                 n_rays=45):
        self.n = n
        self.rng = random.Random(seed)
        self.scan_range = scan_range
        self.fov = fov
        self.n_rays = n_rays

        self.truth = np.zeros((n, n), bool)
        self._make_world(n_obs)

        self.grid = np.full((n, n), -1, np.int64)
        self.x, self.y = n // 6, n // 6
        self.heading = 0.0
        self.trail = []
        self.step_i = 0
        # 목표 고정. 매 스텝 최근접 프런티어를 새로 고르면
        # FOV 130도라 뒤쪽이 안 지워져 목표가 앞뒤로 튕긴다(라이브락).
        self.target = None
        self.hold = 0
        self.dead = set()

        self.f_temp = ScalarField(n, 24.0, 68.0, 2, 8.0, seed)
        self.f_gas = ScalarField(n, 4.0, 72.0, 2, 7.0, seed + 7)
        self.measured_t, self.measured_g = {}, {}

    def _make_world(self, k):
        n, r = self.n, self.rng
        for _ in range(k * 40):
            if k <= 0:
                break
            half = r.choice([2, 3, 4])
            ox = r.randint(half + 2, n - half - 3)
            oy = r.randint(half + 2, n - half - 3)
            if abs(ox - n // 6) < 8 and abs(oy - n // 6) < 8:
                continue
            shape = r.choice(["rect", "circle", "tri"])
            for yy in range(oy - half, oy + half + 1):
                for xx in range(ox - half, ox + half + 1):
                    if shape == "circle":
                        ok = (xx - ox) ** 2 + (yy - oy) ** 2 <= half * half
                    elif shape == "tri":
                        ok = abs(xx - ox) <= (oy + half - yy) / 2
                    else:
                        ok = True
                    if ok:
                        self.truth[yy][xx] = True
            k -= 1
        # 바깥 벽
        self.truth[0, :] = self.truth[-1, :] = True
        self.truth[:, 0] = self.truth[:, -1] = True

    def _ray(self, ang):
        ar = math.radians(ang)
        ca, sa = math.cos(ar), math.sin(ar)
        for i in range(self.scan_range + 1):
            xx = int(self.x + i * ca)
            yy = int(self.y + i * sa)
            if not (0 <= xx < self.n and 0 <= yy < self.n):
                return
            if self.truth[yy][xx]:
                self.grid[yy][xx] = 3
                return
            if self.grid[yy][xx] != 3:
                self.grid[yy][xx] = 0

    def scan(self):
        for i in range(self.n_rays):
            self._ray(self.heading - self.fov / 2 + self.fov * i / (self.n_rays - 1))

    def frontiers(self):
        g = self.grid
        f = []
        ys, xs = np.where(g == 0)
        for x, y in zip(xs.tolist(), ys.tolist()):
            for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1)):
                nx, ny = x + dx, y + dy
                if 0 <= nx < self.n and 0 <= ny < self.n and g[ny][nx] == -1:
                    f.append((x, y))
                    break
        return f

    def _bfs(self, goals):
        from collections import deque
        if not goals:
            return None
        gs = set(goals)
        prev = {(self.x, self.y): None}
        dq = deque([(self.x, self.y)])
        while dq:
            cx, cy = dq.popleft()
            if (cx, cy) in gs and (cx, cy) != (self.x, self.y):
                path = []
                cur = (cx, cy)
                while prev[cur] is not None:
                    path.append(cur)
                    cur = prev[cur]
                return path[::-1]
            for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1),
                           (1, 1), (1, -1), (-1, 1), (-1, -1)):
                nx, ny = cx + dx, cy + dy
                if not (1 <= nx < self.n - 1 and 1 <= ny < self.n - 1):
                    continue
                if (nx, ny) in prev or self.grid[ny][nx] == 3:
                    continue
                # 몸통 여유 3x3
                if self.truth[ny - 1:ny + 2, nx - 1:nx + 2].any():
                    continue
                prev[(nx, ny)] = (cx, cy)
                dq.append((nx, ny))
        return None

    def tick(self):
        """한 스텝. 반환: (frontiers, path, done)"""
        self.step_i += 1
        self.scan()
        self.trail.append((self.x, self.y))

        self.measured_t[(self.x, self.y)] = self.f_temp.sample(
            self.x, self.y, 0.7, self.rng)
        self.measured_g[(self.x, self.y)] = self.f_gas.sample(
            self.x, self.y, 1.5, self.rng)

        fr = [f for f in self.frontiers() if f not in self.dead]
        if not fr:
            return [], None, True

        # ---- 목표 유지 판정 ----
        self.hold += 1
        if (self.target is None or self.target not in fr
                or self.target == (self.x, self.y) or self.hold > 120):
            if self.target is not None and self.hold > 120:
                self.dead.add(self.target)       # 너무 오래 붙잡으면 포기
            self.target = self._pick(fr)
            self.hold = 0
            if self.target is None:
                return fr, None, True

        path = self._bfs([self.target])
        if not path:
            self.dead.add(self.target)
            self.target = None
            return fr, None, False

        nx, ny = path[0]
        self.heading = math.degrees(math.atan2(ny - self.y, nx - self.x))
        self.x, self.y = nx, ny
        return fr, path[:60], False

    def _pick(self, fr):
        """가장 가까운 프런티어. 단, 지금 위치 바로 옆(2칸 이내)은
        스캔으로 곧 지워지므로 건너뛴다 — 이게 왕복의 주원인이다."""
        cand = [f for f in fr
                if math.hypot(f[0] - self.x, f[1] - self.y) > 2.5]
        if not cand:
            cand = fr
        cand.sort(key=lambda f: math.hypot(f[0] - self.x, f[1] - self.y))
        for f in cand[:40]:
            if self._bfs([f]):
                return f
        return None
#	격자 A* 경로계획 핵심 — 16방향 이동, 안전맵/여유공간맵, 프런티어 탐색
"""
path_planner_sim2.py -- 프런티어 기반 탐사 + 장애물 형태 추정 시뮬레이터

[공부용 재배치판]
  같은 역할을 하는 함수끼리 묶고 구분선을 넣었다.

[이 파일을 읽는 가장 중요한 축: '정답(truth) 참조 여부']
  이 시뮬의 핵심 설계는 "로봇이 알 수 있는 것"과 "시뮬만 아는 것"을 엄격히
  분리하는 것이다. 실기에는 정답 지도가 없으므로, 정답을 참조하는 코드가
  로봇 로직에 섞여 있으면 시뮬 성적이 전부 거짓이 된다.
  그래서 각 섹션 머리에 다음 표시를 달았다.

    [정답 O / 시뮬 전용]  truth_id, hidden_obstacles 를 본다.
                          실기로 옮길 때 존재 자체가 사라지거나 실물로 대체된다.
    [정답 X / 로봇 로직]  관측 데이터만 본다. 실기에 그대로 올라간다.
    [경계]                정답을 물리 판정에만 쓰고 로봇에게는 넘기지 않는다.
                          여기가 실기 전환 시 갈아끼울 접합부다.

[★볼록껍질 전환 - 형태 분류(v1~v3) 완전 삭제]
  지금까지는 관측점을 원형도(circ)/면적비(extent)로 재서 원/삼각형/사각형
  세 틀 중 하나로 분류했다. 전시회에 놓일 장애물이 이 세 틀에 안 맞는
  부정형일 가능성이 제기되어, 분류 자체를 없애고 관측점을 감싸는
  볼록다각형(convex hull)을 분류 없이 그대로 지도에 채우는 방식으로
  바꿨다.

  장점: 관측이 늘수록 정확해진다. 어떤 형태든 분류를 시도하지 않으므로
  오분류 자체가 없다. 관측된 표면 바깥으로 그리지 않으므로 안전 측면에서
  보수적이다.

  근본 한계 (반드시 인지할 것): 오목한 형태는 볼록껍질 성질상 표현이
  원리적으로 불가능하다. 관측을 아무리 늘려도 이 한계는 없어지지 않는다 -
  오목한 부분은 항상 볼록하게 "메워져" 그려진다. 장애물이 뚜렷하게 오목한
  형태(ㄱ자, U자 등)일 가능성이 높다면 이 방식만으로는 부족하고, 오목
  다각형 분해(concave hull / alpha-shape)가 추가로 필요하다 - 지금은
  구현하지 않았다.

  이전 방식(v1~v3 분류)이 의존하던 원뿔 절단(trim_cone)은 이 방식에도
  똑같이 필요하다. 원뿔 부풀림은 점 복원(관측점 좌표 계산) 단계에서
  생기는 문제라, 그 뒤에 뭘 하든(분류하든 안 하든) 사라지지 않는다.

  제거된 것: 형태 분류 규칙(v1/v2/v3), circ/extent 계산, 회전각 추정
  (min_area_rect_angle/triangle_rotation/dp_simplify), thickness 게이팅
  (앞면만 봄 -> unknown 보류), 작은 원/사각 구분 보류. 전부 "이게 무슨
  모양인가"를 풀기 위한 코드였는데, 이제 그 질문 자체를 안 묻는다.
  유지된 것: 확정(값1)/미확정(값2) 판정 - 이건 형태가 아니라 '몇 방향에서
  봤는가'(viewpoint_spread)의 문제라 그대로 남는다.

[목차]
   1. 설정값 (파라미터)
   2. 격자(grid) 값 체계          -- 문서
   3. 전역 상태
   4. 순수 기하 원시연산           [정답 X]
   5. 각도 통계 (원형 통계)        [정답 X]
   6. (비움 - 회전각 적합 로직은 볼록껍질 전환으로 제거됨)
   7. 정답 월드 생성               [정답 O / 시뮬 전용]
   8. 센서 시뮬 -> 관측 기록       [경계]
   9. 관측 조회                    [정답 X]
  10. 클러스터링 (관측 묶기)       [정답 X]
  11. 볼록껍질 추정 -> 지도 반영   [정답 X]
  12. 경로 계획 (도달성)           [정답 X]
  13. 가시선 / 탐색 목표 선정      [정답 X]
  14. 충돌 범퍼                    [정답 O / 시뮬 전용]
  15. 모터 명령 (실기 접합부)
  16. 채점                         [정답 O / 시뮬 전용]
  17. 메인 루프
  18. 시각화
  19. 엔트리포인트
"""

import argparse
import heapq
import math
import random

import numpy as np

from path_planner import (GRID_SIZE, ROBOT_WIDTH_CELLS, find_frontiers,
                          get_neighbors_cost, segment_safe,
                          compute_clearance_map, compute_safe_map,
                          CLEARANCE_WANT, CLEARANCE_COST, find_path)


# =============================================================================
# =============================================================================
#  1. 설정값 (파라미터)
# =============================================================================
# =============================================================================

# -----------------------------------------------------------------------------
# 1-1. 센서 / 스캔 - 실물 하드웨어와 반드시 맞춰야 하는 값들
# -----------------------------------------------------------------------------

SCAN_RANGE = 36          # robot_config.DIST_MAX_MM(1800) / CELL_SIZE(50mm)

SERVO_MAX_OFF = 65.0     # 중심(헤딩) 기준 좌우 최대 오프셋 (robot_config 서보범위)
CAMERA_FOV = 2 * SERVO_MAX_OFF   # 130. 1회 스윕 커버리지
SCAN_COUNT = 30          # 한 번의 FOV 스캔에서 쏘는 광선 개수

LIMIT_SCAN_TO_SERVO = True
TURN_TO_AIM = True

CONTINUOUS_SCAN = True   # 실물(연속 스윕) 모델. False = 정지 일괄(레거시 A/B용)

CELL_SIZE_M = 0.05
ROBOT_SPEED_MPS = 0.15
ROBOT_TURN_DPS = 75.0
SERVO_SWEEP_DPS = 100.0
TOF_PERIOD_S = 0.020

FULL_SWEEP_S = None
IDLE_SWEEP_S = 0.30

VP_QUANT = 0.5
STATIONARY_RAY_S = 0.35

ROBOT_MARGIN = ROBOT_WIDTH_CELLS // 2      # 2 (몸통 5x5 의 반폭)

# ---- ToF 원뿔 각도 절단 (볼록껍질 방식에서도 여전히 필요 - 상단 docstring 참고) ----
TOF_FOV_DEG = 25.0
CONE_TRIM_DEG = TOF_FOV_DEG / 2.0     # 12.5도
USE_CONE_TRIM = True

USE_CONE_MODEL = True
CONE_SUBRAYS = 9

# -----------------------------------------------------------------------------
# 1-2. 탐사 종료 조건 - 무한루프 / 라이브락 방지용 상한들
# -----------------------------------------------------------------------------

MAX_STEPS = 1500
STALL_LIMIT = 90
COMMIT_LIMIT = 80
ABANDON_EVERY = 8
ABANDON_BUDGET = 150

# -----------------------------------------------------------------------------
# 1-3. 볼록껍질 추정 임계값
# -----------------------------------------------------------------------------
#
# ★볼록껍질 전환: 예전 여기 있던 THICKNESS_MIN/CIRC_*/EXTENT_*/SHAPE_RULE/
# ENABLE_ROTATION 등 형태 분류용 임계값은 전부 삭제했다. 남은 건 두 가지뿐 -
# "추정을 시도할 최소 조건"과 "확정 판정 기준" - 둘 다 형태가 아니라
# 관측량/관측방향의 문제라 볼록껍질 방식에서도 그대로 유효하다.

SHAPE_MIN_OBS = 5        # 볼록껍질 추정을 시도할 최소 관측 횟수
SHAPE_MIN_SPREAD = 63    # 시도할 최소 각도 커버리지(도) - 광선각 기준 사전필터
CONFIRM_SPREAD = 180     # 이 이상이면 확정(검정), 아니면 미확정(노랑)
                         # (관측지점 방위각 기준. §5 참고)

# -----------------------------------------------------------------------------
# 1-4. 점유격자 확률 모델 (log-odds)
# -----------------------------------------------------------------------------

USE_LOGODDS = True
L_OCC = 0.85
L_FREE = -0.40
L_MIN, L_MAX = -4.0, 4.0
L_THRESH = 1.5

# -----------------------------------------------------------------------------
# 1-5. 센서 오차 모델
# -----------------------------------------------------------------------------

SENSOR_FALSE_HIT = 0.0
SENSOR_MISS = 0.0

# -----------------------------------------------------------------------------
# 1-6. 경로 안전 정책
# -----------------------------------------------------------------------------

SAFE_BLOCK_ESTIMATED = True
# ★미확정 상태 삭제로 SAFE_BLOCK_UNCONFIRMED(값2를 막을지 여부)는 무의미해져
# 제거했다 - 값 2 가 이제 grid 에 아예 안 나온다 (§11 update_obstacle_shape).

# -----------------------------------------------------------------------------
# 1-7. 회전 비용
# -----------------------------------------------------------------------------

TURN_COST = 1.2

# -----------------------------------------------------------------------------
# 1-8. 가시선(LoS) 정책
# -----------------------------------------------------------------------------

LOS_BLOCK_CONFIRMED = True

# -----------------------------------------------------------------------------
# 1-9. 클러스터링 (관측 -> 장애물 연결) 및 월드 생성 난이도
# -----------------------------------------------------------------------------

CLUSTER_GAP = 2
CLUSTER_EVERY = 4
MAX_VIEWPOINTS_PER_CELL = 60

OBSTACLE_MIN_SEP = 4
OBSTACLE_COUNT = 12

# -----------------------------------------------------------------------------
# 1-10. 시각화 (표시 전용 - 시뮬 로직과 무관)
# -----------------------------------------------------------------------------

TRUTH_SMOOTH = True
TRUTH_SHOW_RASTER = True

# -----------------------------------------------------------------------------
# 1-11. 확정 순회 / 복귀 단계
# -----------------------------------------------------------------------------

CONFIRM_PHASE = True
CONFIRM_MAX_VISITS = 2
CONFIRM_STEP_BUDGET = 300
CONFIRM_STANDOFF = 8
CONFIRM_BEARING_PENALTY = 0.05

RETURN_HOME = True

# -----------------------------------------------------------------------------
# 1-12. 요구조자 배치 (정답 - 시뮬 전용)
# -----------------------------------------------------------------------------
#
# hidden_obstacles 와 같은 위상이다: 시뮬만 아는 '정답' 위치이고, 로봇은
# 실제로 발견하기(§17 discover 체크) 전까지 이 좌표를 로봇 로직에서
# 절대 참조하지 않는다.
RESCUEE_EDGE_MARGIN = ROBOT_MARGIN + 1     # 지도 가장자리 제외 - 로봇 몸통이
                                            # 원리적으로 못 들어가는 구간
                                            # (rescue_planner.place_rescuee 에서
                                            #  실측으로 확인된 문제, §11 참고)
RESCUEE_MIN_DIST_FROM_START = 15           # 시작점에서 이 정도는 떨어진 곳
RESCUEE_PLACEMENT_TRIES = 500              # 도달 가능한 위치 찾을 때까지 재시도


# =============================================================================
# =============================================================================
#  2. 격자(grid) 값 체계  -- 문서 (코드 없음)
# =============================================================================
# =============================================================================
#
#   -1 미탐색(회색)          : 아직 아무 정보 없음
#    0 빈공간(흰)            : 광선이 통과했다 = 직접 관측한 빈 곳
#    1 볼록껍질 채움(검정)    : 여러 방향(180도 이상)에서 확인되어 확정된 장애물
#    2 (사용 안 함)           : ★미확정 상태 삭제됨 - 아래 §11 docstring 참고.
#                              값 자체는 다른 코드와의 호환을 위해 번호만 비워둠.
#    3 직접관측(주황)         : 광선이 실제로 맞은 표면 칸. 경로를 막는다.
#    4 확인불가(핑크)         : 관측 시도했으나 실패해 포기
#
# 우선순위 규칙: 관측이 추정을 이긴다.


# =============================================================================
# =============================================================================
#  3. 전역 상태 (reset_world 로 초기화)
# =============================================================================
# =============================================================================

truth_id = None
obstacle_ids = []
hidden_obstacles = {}

# ── 요구조자 (§1-12) ── hidden_obstacles 와 같은 위상의 '정답' 값
rescuee_truth = None          # (x, y) 또는 None(배치 실패)
rescuee_discovered = False
rescuee_discover_step = None

grid = None
logodds = None
sensor_rng = None

turns = 0
turn_deg = 0.0
aim_turns = 0
aim_turn_deg = 0.0
aim_blocked = 0

servo_off = 0.0
servo_dir = 1
sim_time = 0.0
rays_fired = 0
moves = 0

_sweep_buffer = []
_leg_start_is_limit = False

phase = "explore"
home = (0, 0)
return_path = []
confirm_cid = None
confirm_done = 0
confirm_gained = 0
returned = False

robot_x = robot_y = 0
robot_angle = 0.0
step = 0
collisions = 0

hit_obs = {}
cluster_cells = {}
_next_cid = 0
low_confidence_obstacles = set()
unreachable_unknowns = set()
obstacle_hull = {}          # ★볼록껍질: cid -> [(x,y), ...] (CCW, 실수좌표)
obstacle_render_info = {}   # cid -> ("hull", cx, cy, half, rot_deg=0.0, value, half_w, half_h)
last_filled = {}
last_obs_count = {}
cell_owner = {}


# =============================================================================
# =============================================================================
#  4. 순수 기하 원시연산                                            [정답 X]
# =============================================================================
# =============================================================================

def point_in_triangle(px, py, x1, y1, x2, y2, x3, y3):
    """세 변에 대한 외적 부호가 모두 같으면 내부. (정답 월드 생성에서만 씀 - §7)"""
    def sign(ax, ay, bx, by, cx, cy):
        return (ax - cx) * (by - cy) - (bx - cx) * (ay - cy)
    d1 = sign(px, py, x1, y1, x2, y2)
    d2 = sign(px, py, x2, y2, x3, y3)
    d3 = sign(px, py, x3, y3, x1, y1)
    has_neg = (d1 < 0) or (d2 < 0) or (d3 < 0)
    has_pos = (d1 > 0) or (d2 > 0) or (d3 > 0)
    return not (has_neg and has_pos)


def convex_hull(points):
    """모노톤 체인. 관측점들을 감싸는 최소 볼록다각형. 반시계(CCW) 순서로 반환한다
    - point_in_convex_polygon() 이 이 순서에 의존한다."""
    pts = sorted(set(points))
    if len(pts) <= 2:
        return pts

    def cross(o, a, b):
        return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])
    lower = []
    for p in pts:
        while len(lower) >= 2 and cross(lower[-2], lower[-1], p) <= 0:
            lower.pop()
        lower.append(p)
    upper = []
    for p in reversed(pts):
        while len(upper) >= 2 and cross(upper[-2], upper[-1], p) <= 0:
            upper.pop()
        upper.append(p)
    return lower[:-1] + upper[:-1]


def point_in_convex_polygon(px, py, hull):
    """볼록다각형 내부 판정 (경계 포함). ★볼록껍질 방식의 핵심 함수.

    convex_hull() 이 반시계(CCW) 순서로 반환하므로, 모든 변에 대해 점이
    '왼쪽'(외적 >= 0)에 있으면 내부다. 순서가 CCW 라는 가정이 깨지면 이
    함수가 뒤집혀 전부 바깥/전부 안으로 오판하므로, hull 은 반드시
    convex_hull() 이 반환한 것만 넣을 것."""
    n = len(hull)
    for i in range(n):
        x1, y1 = hull[i]
        x2, y2 = hull[(i + 1) % n]
        cross = (x2 - x1) * (py - y1) - (y2 - y1) * (px - x1)
        if cross < -1e-9:
            return False
    return True


# =============================================================================
# =============================================================================
#  5. 각도 통계 (원형 통계)                                         [정답 X]
# =============================================================================
# =============================================================================
#
# '얼마나 여러 방향에서 봤는가'를 재는 두 함수. 둘 다 circular_spread 를 쓰지만
# **무엇의 각도를 넣는지**가 다르다.
#
#   circular_spread([광선각])        -> 값싼 사전 필터 (SHAPE_MIN_SPREAD, 63도)
#   viewpoint_spread(obs, cx, cy)   -> 확정 판정      (CONFIRM_SPREAD, 180도)

def circular_spread(angles):
    """각도 집합이 실제로 덮는 호의 크기(도). ±180 경계 wrap 을 올바르게 처리."""
    if len(angles) < 2:
        return 0.0
    a = sorted(x % 360.0 for x in angles)
    max_gap = 360.0 - (a[-1] - a[0])
    for i in range(len(a) - 1):
        max_gap = max(max_gap, a[i + 1] - a[i])
    return 360.0 - max_gap


def viewpoint_gap_bearing(obs, cx, cy):
    """관측지점 방위각에서 '가장 크게 비어 있는 구간'의 한가운데 방위(도).
    확정 순회에서 어느 방향으로 찾아가야 새 정보를 얻는지 정하는 데 쓴다."""
    vps = {(rx, ry) for _, _, rx, ry in obs}
    if not vps:
        return 0.0
    angs = sorted(math.degrees(math.atan2(ry - cy, rx - cx)) % 360.0
                  for rx, ry in vps)
    if len(angs) == 1:
        return (angs[0] + 180.0) % 360.0
    best_gap = 360.0 - (angs[-1] - angs[0])
    best_mid = (angs[-1] + best_gap / 2.0) % 360.0
    for i in range(len(angs) - 1):
        g = angs[i + 1] - angs[i]
        if g > best_gap:
            best_gap = g
            best_mid = (angs[i] + g / 2.0) % 360.0
    return best_mid


def cluster_center_estimate(cid):
    """클러스터의 대략적 중심 (히트 셀 좌표 평균).

    ★미확정 상태 삭제로 확정 전 클러스터는 obstacle_render_info 에 없다
    (볼록껍질 자체가 아직 없다). 그런데 확인 순회(§17 confirm phase)는
    "아직 확정 안 된 클러스터"를 찾아가야 하므로 중심 좌표가 필요하다 -
    정밀한 볼록껍질 중심일 필요는 없고, '대충 이 근처'로 접근점을 잡을
    정도면 충분해서 히트 셀 좌표를 그냥 평균낸다."""
    cells = cluster_cells.get(cid)
    if not cells:
        return None
    xs = [c[0] for c in cells]
    ys = [c[1] for c in cells]
    return sum(xs) / len(xs), sum(ys) / len(ys)


def needs_confirm(cid):
    """이 클러스터가 아직 확정(=화면/지도에 채워짐) 상태가 아닌가.

    ★미확정 상태 삭제: 이제 "그려져 있다"와 "확정됐다"가 완전히 같은 뜻이다
    (§11 update_obstacle_shape 참고 - 180도 미만인 동안은 아예 안 그린다).
    그래서 판정이 훨씬 단순해졌다: obstacle_render_info 에 없으면 아직
    확정 못 한 것이다."""
    return cid not in obstacle_render_info


def viewpoint_spread(obs, cx, cy):
    """추정 중심에서 본 '관측 지점들'의 방위각 커버리지(도).

    [수정2, 유지] 확정(검정) 판정에는 광선각이 아니라 관측지점 방위각을
    써야 한다 - 큰 장애물을 가까이서 보면 관측 지점 하나에서도 좌우 끝
    광선각이 크게 벌어져, 같은 쪽에서만 본 걸 여러 방향에서 본 걸로
    오판하기 때문이다."""
    vps = {(rx, ry) for _, _, rx, ry in obs}
    if len(vps) < 2:
        return 0.0
    return circular_spread([math.degrees(math.atan2(ry - cy, rx - cx))
                            for rx, ry in vps])


# =============================================================================
# =============================================================================
#  6. (비움)
# =============================================================================
# =============================================================================
#
# ★볼록껍질 전환으로 제거됨: _bbox_area_at / min_area_rect_angle /
# triangle_rotation / dp_simplify. 전부 "이 볼록껍질이 삼각형/사각형에
# 얼마나 가까운가, 몇 도 돌아갔는가"를 묻던 코드였다. 이제 볼록껍질을
# 그 자체로 그리므로 회전각을 별도로 추정할 필요가 없다(껍질 좌표 자체가
# 이미 회전을 담고 있다).


# =============================================================================
# =============================================================================
#  7. 정답 월드 생성                              [정답 O / 시뮬 전용]
# =============================================================================
# =============================================================================

def obstacle_half(size):
    return max(int(size), 2)


def is_point_in_obstacle(x, y, ox, oy, size, shape):
    """정답 도형의 '정의'. 시각화의 매끈한 윤곽도 이 정의를 그대로 쓴다."""
    half = obstacle_half(size)
    if shape == "circle":
        return (x - ox) ** 2 + (y - oy) ** 2 <= half ** 2
    if shape == "triangle":
        return point_in_triangle(x, y, ox, oy + half,
                                 ox - half, oy - half, ox + half, oy - half)
    return abs(x - ox) <= half and abs(y - oy) <= half


def generate_random_obstacles(count, rx, ry, rng, min_sep=None):
    """장애물 배치. 정답 도형은 여전히 원/삼각/사각으로 생성한다 - 채점
    기준(진짜로 뭐가 있었는지)이 있어야 IoU/center_err 를 잴 수 있다.
    볼록껍질 방식이 대응하려는 건 '로봇이 도형을 분류하지 않는다'는 것이지,
    '정답 월드가 부정형이어야 한다'는 게 아니다 - 부정형 정답 월드가
    필요하면 별도로 추가해야 한다(아래 §19 참고)."""
    if min_sep is None:
        min_sep = OBSTACLE_MIN_SEP
    obstacles = {}
    attempts = 0
    while len(obstacles) < count and attempts < count * 200:
        attempts += 1
        size = rng.choice([2, 2, 3, 3, 4])
        half = obstacle_half(size)
        ox = rng.randint(half + 1, GRID_SIZE - half - 2)
        oy = rng.randint(half + 1, GRID_SIZE - half - 2)
        clear = half + ROBOT_MARGIN + 3
        if abs(ox - rx) < clear and abs(oy - ry) < clear:
            continue
        if any(max(abs(ox - px), abs(oy - py))
               < half + obstacle_half(o["size"]) + min_sep
               for (px, py), o in obstacles.items()):
            continue
        shape = rng.choice(["rect", "rect", "circle", "triangle"])
        obstacles[(ox, oy)] = {"size": size, "shape": shape}
    return obstacles


def build_truth(obstacles):
    tid = np.full((GRID_SIZE, GRID_SIZE), -1, dtype=np.int32)
    ids = list(obstacles.keys())
    for idx, (ox, oy) in enumerate(ids):
        info = obstacles[(ox, oy)]
        half = obstacle_half(info["size"])
        for y in range(max(0, oy - half), min(GRID_SIZE, oy + half + 1)):
            for x in range(max(0, ox - half), min(GRID_SIZE, ox + half + 1)):
                if is_point_in_obstacle(x, y, ox, oy, info["size"], info["shape"]):
                    tid[y][x] = idx
    return tid, ids


def place_rescuee_truth(rng, robot_start):
    """요구조자의 '정답' 위치를 정한다. hidden_obstacles 와 같은 위상 -
    로봇은 실제로 발견하기 전까지 이 좌표를 모른다(§17 run() 의 discover
    체크가 유일하게 이 값을 로봇 쪽으로 넘기는 지점).

    조건 (요청 그대로):
      - 지도 가장자리(RESCUEE_EDGE_MARGIN 칸 이내) 제외 - 로봇 몸통(5x5)이
        원리적으로 못 들어가는 구간이다.
      - 장애물이 있는 구역 제외 - 로봇 몸통 반경(ROBOT_MARGIN)까지 포함해서
        장애물과 안 겹치는 자리만 후보로 삼는다.
      - 시작점에서 RESCUEE_MIN_DIST_FROM_START 칸 이상.
      - (추가 검증) 장애물 배치만으로 실제 도달 가능한 자리인지 A* 로
        확인한다 - 위 조건을 다 만족해도 장애물이 우연히 완전히 둘러싸면
        갈 수 없는 자리가 나올 수 있어서다. 해저드(온도/가스)는 여기서
        고려하지 않는다 - 그건 순전히 로봇이 탐사하며 알아가는 정보라
        배치 시점(정답 생성 단계)에서 참조하면 안 된다."""
    test_grid = np.where(truth_id >= 0, 3, 0).astype(np.int64)
    for _ in range(RESCUEE_PLACEMENT_TRIES):
        x = rng.randint(RESCUEE_EDGE_MARGIN, GRID_SIZE - RESCUEE_EDGE_MARGIN - 1)
        y = rng.randint(RESCUEE_EDGE_MARGIN, GRID_SIZE - RESCUEE_EDGE_MARGIN - 1)
        if math.hypot(x - robot_start[0], y - robot_start[1]) < RESCUEE_MIN_DIST_FROM_START:
            continue
        clear = True
        for dy in range(-ROBOT_MARGIN, ROBOT_MARGIN + 1):
            for dx in range(-ROBOT_MARGIN, ROBOT_MARGIN + 1):
                if truth_id[y + dy][x + dx] >= 0:
                    clear = False
                    break
            if not clear:
                break
        if not clear:
            continue
        if find_path(test_grid, robot_start, (x, y)) is not None:
            return (x, y)
    return None   # 극단적으로 좁은 월드 - 이번 시드는 요구조자 없이 진행


def reset_world(seed, robot_start=(10, 10)):
    """월드 + 로봇 + 모든 추정 상태를 초기화. 시드가 같으면 항상 같은 월드."""
    global grid, truth_id, obstacle_ids, hidden_obstacles
    global rescuee_truth, rescuee_discovered, rescuee_discover_step
    global robot_x, robot_y, robot_angle, step, collisions
    global hit_obs, cluster_cells, _next_cid, low_confidence_obstacles
    global unreachable_unknowns, obstacle_hull, obstacle_render_info
    global last_filled, last_obs_count, cell_owner
    global logodds, sensor_rng, turns, turn_deg
    global aim_turns, aim_turn_deg, aim_blocked
    global servo_off, servo_dir, sim_time, rays_fired, moves, FULL_SWEEP_S
    global _sweep_buffer, _leg_start_is_limit
    global phase, home, return_path, confirm_cid, confirm_done
    global confirm_gained, returned

    rng = random.Random(seed)
    robot_x, robot_y = robot_start
    home = tuple(robot_start)
    phase = "explore"
    return_path = []
    confirm_cid = None
    confirm_done = 0
    confirm_gained = 0
    returned = False
    robot_angle = 0.0
    step = 0
    collisions = 0
    turns = 0
    turn_deg = 0.0
    aim_turns = 0
    aim_turn_deg = 0.0
    aim_blocked = 0
    servo_off = 0.0
    servo_dir = 1
    sim_time = 0.0
    rays_fired = 0
    moves = 0
    _sweep_buffer = []
    _leg_start_is_limit = False
    FULL_SWEEP_S = (2 * SERVO_MAX_OFF) / SERVO_SWEEP_DPS

    grid = np.full((GRID_SIZE, GRID_SIZE), -1, dtype=np.int64)
    logodds = np.zeros((GRID_SIZE, GRID_SIZE), dtype=np.float64)
    sensor_rng = random.Random(seed * 7919 + 13)
    hidden_obstacles = generate_random_obstacles(OBSTACLE_COUNT, robot_x, robot_y, rng)
    truth_id, obstacle_ids = build_truth(hidden_obstacles)
    rescuee_truth = place_rescuee_truth(rng, robot_start)
    rescuee_discovered = False
    rescuee_discover_step = None

    hit_obs = {}
    cluster_cells = {}
    _next_cid = 0
    low_confidence_obstacles = set()
    unreachable_unknowns = set()
    obstacle_hull = {}
    obstacle_render_info = {}
    last_filled = {}
    last_obs_count = {}
    cell_owner = {}

    for dy in range(-ROBOT_MARGIN, ROBOT_MARGIN + 1):
        for dx in range(-ROBOT_MARGIN, ROBOT_MARGIN + 1):
            assert truth_id[robot_y + dy][robot_x + dx] < 0, "시작 지점이 장애물과 겹침"


# =============================================================================
# =============================================================================
#  8. 센서 시뮬 -> 관측 기록                                        [경계]
# =============================================================================
# =============================================================================
#
# 이 섹션은 볼록껍질 전환과 무관하다 (원뿔 절단은 여전히 필요 - 상단
# docstring 참고). 로직은 이전과 동일.

def get_scan_angles(base_angle):
    half_fov = CAMERA_FOV / 2
    return [base_angle - half_fov + (CAMERA_FOV / (SCAN_COUNT - 1)) * i
            for i in range(SCAN_COUNT)]


def servo_reachable(angle_deg):
    if not LIMIT_SCAN_TO_SERVO:
        return True
    d = (angle_deg - robot_angle + 180) % 360 - 180
    return abs(d) <= SERVO_MAX_OFF + 1e-6


def turn_body_to_reach(aim_deg):
    global robot_angle, turns, turn_deg, aim_turns, aim_turn_deg, aim_blocked

    if not LIMIT_SCAN_TO_SERVO:
        return True
    off = (aim_deg - robot_angle + 180) % 360 - 180
    if abs(off) <= SERVO_MAX_OFF:
        return True
    if not TURN_TO_AIM:
        aim_blocked += 1
        return False

    need = off - math.copysign(SERVO_MAX_OFF, off)
    turns += 1
    turn_deg += abs(need)
    aim_turns += 1
    aim_turn_deg += abs(need)
    if CONTINUOUS_SCAN:
        sweep_while_turning(robot_x, robot_y, robot_angle, need)
    robot_angle = (robot_angle + need + 180) % 360 - 180
    send_command({"cmd": "turn_left" if need > 0 else "turn_right",
                  "speed": 100, "duration_ms": int(abs(need) * 10)})
    return True


def trim_cone(rays, leg_is_full_sweep=True):
    """원뿔 부풀림 보정. ★볼록껍질 방식에서도 그대로 필요 (상단 docstring)."""
    idx_runs, cur = [], []
    for idx, r in enumerate(rays):
        if r[2] is not None:
            cur.append((idx, r))
        elif cur:
            idx_runs.append(cur)
            cur = []
    if cur:
        idx_runs.append(cur)

    n = len(rays)
    out = []
    for run in idx_runs:
        lo_idx, lo_r = run[0]
        hi_idx, hi_r = run[-1]
        lo_off, hi_off = lo_r[0], hi_r[0]
        at_lo_edge = leg_is_full_sweep and lo_idx == 0
        at_hi_edge = leg_is_full_sweep and hi_idx == n - 1
        lo = lo_off if at_lo_edge else lo_off + CONE_TRIM_DEG
        hi = hi_off if at_hi_edge else hi_off - CONE_TRIM_DEG
        if hi < lo:
            continue
        out += [r for _, r in run if lo <= r[0] <= hi]
    return out


def _flush_sweep_buffer(end_is_limit):
    global _sweep_buffer, _leg_start_is_limit
    if _sweep_buffer:
        full = _leg_start_is_limit and end_is_limit
        for r in trim_cone(_sweep_buffer, leg_is_full_sweep=full):
            _, angle, dist, hit_cell, x, y = r
            if hit_cell is not None:
                record_hit(hit_cell, angle, dist, x, y)
    _sweep_buffer = []
    _leg_start_is_limit = end_is_limit


def _advance_servo(dt):
    global servo_off, servo_dir
    servo_off += SERVO_SWEEP_DPS * dt * servo_dir
    if servo_off > SERVO_MAX_OFF:
        servo_off = 2 * SERVO_MAX_OFF - servo_off
        servo_dir = -1
    elif servo_off < -SERVO_MAX_OFF:
        servo_off = -2 * SERVO_MAX_OFF - servo_off
        servo_dir = 1
    return servo_off


def sweep_during(duration_s, pose_at):
    global sim_time
    n = int(duration_s / TOF_PERIOD_S)
    for i in range(n):
        t = (i + 1) * TOF_PERIOD_S
        prev_dir = servo_dir
        off = _advance_servo(TOF_PERIOD_S)
        x, y, h = pose_at(t)
        angle = h + off
        hit_cell, dist = real_scan(grid, x, y, angle)

        if USE_CONE_TRIM:
            _sweep_buffer.append((off, angle, dist, hit_cell, x, y))
            if servo_dir != prev_dir:
                _flush_sweep_buffer(end_is_limit=True)
        elif hit_cell is not None:
            record_hit(hit_cell, angle, dist, x, y)
    sim_time += duration_s


def sweep_while_turning(x, y, a0, delta_deg):
    dur = abs(delta_deg) / ROBOT_TURN_DPS
    if dur <= 0:
        return
    sweep_during(dur, lambda t: (x, y, a0 + delta_deg * (t / dur)))


def sweep_while_moving(x0, y0, x1, y1, heading):
    dist_cells = math.hypot(x1 - x0, y1 - y0)
    dur = dist_cells * CELL_SIZE_M / ROBOT_SPEED_MPS
    if dur <= 0:
        return
    sweep_during(dur, lambda t: (x0 + (x1 - x0) * (t / dur),
                                 y0 + (y1 - y0) * (t / dur),
                                 heading))
    return dur


def sweep_in_place(duration_s):
    sweep_during(duration_s, lambda t: (robot_x, robot_y, robot_angle))


def aim_sweep(g, aim_deg):
    if not turn_body_to_reach(aim_deg):
        return False
    sweep_in_place(FULL_SWEEP_S)
    return True


def scan_toward(g, rx, ry, aim_deg, extra=(-4, -2, 0, 2, 4)):
    """CONTINUOUS_SCAN=False(레거시) 경로 전용."""
    if not turn_body_to_reach(aim_deg):
        return False
    for a in get_scan_angles(aim_deg):
        if servo_reachable(a):
            hit_cell, dist = real_scan(g, rx, ry, a)
            if hit_cell is not None:
                record_hit(hit_cell, a, dist, rx, ry)
    for da in extra:
        aa = aim_deg + da
        if servo_reachable(aa):
            hit_cell, dist = real_scan(g, rx, ry, aa)
            if hit_cell is not None:
                record_hit(hit_cell, aa, dist, rx, ry)
    return True


def observe_cell(g, x, y, hit):
    if not USE_LOGODDS:
        if hit:
            g[y][x] = 3
        elif g[y][x] in (-1, 1, 2):
            g[y][x] = 0
        return
    lo = logodds[y][x] + (L_OCC if hit else L_FREE)
    logodds[y][x] = max(L_MIN, min(L_MAX, lo))
    g[y][x] = 3 if logodds[y][x] >= L_THRESH else 0


def _cast_ray_truth(rx, ry, angle_deg, max_range):
    ar = math.radians(angle_deg)
    ca, sa = math.cos(ar), math.sin(ar)
    for i in range(max_range + 1):
        x = math.floor(rx + i * ca)
        y = math.floor(ry + i * sa)
        if not (0 <= x < GRID_SIZE and 0 <= y < GRID_SIZE):
            return None, None
        if truth_id[y][x] >= 0:
            return (x, y), i
    return None, None


def _cone_min_distance(rx, ry, angle_deg, max_range):
    if CONE_SUBRAYS <= 1:
        return _cast_ray_truth(rx, ry, angle_deg, max_range)
    best_cell, best_dist = None, None
    half = TOF_FOV_DEG / 2.0
    step = (2 * half) / (CONE_SUBRAYS - 1)
    for k in range(CONE_SUBRAYS):
        off = -half + step * k
        cell, d = _cast_ray_truth(rx, ry, angle_deg + off, max_range)
        if d is not None and (best_dist is None or d < best_dist):
            best_cell, best_dist = cell, d
    return best_cell, best_dist


def real_scan(g, rx, ry, angle_deg, max_range=SCAN_RANGE):
    global rays_fired
    rays_fired += 1

    if USE_CONE_MODEL:
        hit_cell, dist = _cone_min_distance(rx, ry, angle_deg, max_range)
    else:
        hit_cell, dist = _cast_ray_truth(rx, ry, angle_deg, max_range)

    occupied = dist is not None
    if occupied and dist >= 1 and SENSOR_MISS > 0 and sensor_rng.random() < SENSOR_MISS:
        occupied = False
        dist = None
        hit_cell = None
    elif (not occupied) and SENSOR_FALSE_HIT > 0 \
            and sensor_rng.random() < SENSOR_FALSE_HIT:
        occupied = True
        dist = sensor_rng.randint(1, max_range)
        ar = math.radians(angle_deg)
        hit_cell = (math.floor(rx + dist * math.cos(ar)),
                    math.floor(ry + dist * math.sin(ar)))

    ar = math.radians(angle_deg)
    ca, sa = math.cos(ar), math.sin(ar)
    walk_to = dist if occupied else max_range + 1
    for i in range(walk_to):
        x = math.floor(rx + i * ca)
        y = math.floor(ry + i * sa)
        if not (0 <= x < GRID_SIZE and 0 <= y < GRID_SIZE):
            return None, None
        observe_cell(g, x, y, False)

    if not occupied:
        return None, None

    x, y = hit_cell
    if not (0 <= x < GRID_SIZE and 0 <= y < GRID_SIZE):
        return None, None
    observe_cell(g, x, y, True)
    return hit_cell, dist


def record_hit(hit_cell, angle_deg, distance, rx, ry):
    per_cell = hit_obs.setdefault(hit_cell, {})
    vp = (round(rx / VP_QUANT) * VP_QUANT, round(ry / VP_QUANT) * VP_QUANT)
    if vp in per_cell or len(per_cell) >= MAX_VIEWPOINTS_PER_CELL:
        return
    per_cell[vp] = (angle_deg, distance)


# =============================================================================
# =============================================================================
#  9. 관측 조회 (읽기 전용 헬퍼)                                    [정답 X]
# =============================================================================
# =============================================================================

def cluster_obs(cid):
    out = []
    for cell in cluster_cells.get(cid, ()):
        for (rx, ry), (a, d) in hit_obs[cell].items():
            out.append((a, d, rx, ry))
    return out


def cluster_obs_count(cid):
    return sum(len(hit_obs[c]) for c in cluster_cells.get(cid, ()))


def evaluate_confidence(cid):
    obs = cluster_obs(cid)
    if len(obs) < 2:
        return 0.0
    return 0.2 if circular_spread([o[0] for o in obs]) < SHAPE_MIN_SPREAD else 0.7


# =============================================================================
# =============================================================================
# 10. 클러스터링 (히트셀 -> 장애물 단위로 묶기)                     [정답 X]
# =============================================================================
# =============================================================================

def _drop_cluster(cid):
    """사라진 클러스터(주로 이웃과 병합됨)가 칠해둔 칸을 되돌린다."""
    for cell in last_filled.pop(cid, ()):
        if cell_owner.get(cell) != cid:
            continue
        x, y = cell
        if grid[y][x] == 1:
            grid[y][x] = -1
        cell_owner.pop(cell, None)
    obstacle_render_info.pop(cid, None)
    obstacle_hull.pop(cid, None)
    last_obs_count.pop(cid, None)
    low_confidence_obstacles.discard(cid)


def rebuild_clusters():
    global cluster_cells, _next_cid

    cells = list(hit_obs.keys())
    parent = {c: c for c in cells}

    def find(c):
        while parent[c] != c:
            parent[c] = parent[parent[c]]
            c = parent[c]
        return c

    cellset = set(cells)
    for (x, y) in cells:
        for dy in range(0, CLUSTER_GAP + 1):
            for dx in range(-CLUSTER_GAP, CLUSTER_GAP + 1):
                if dy == 0 and dx <= 0:
                    continue
                n = (x + dx, y + dy)
                if n in cellset:
                    ra, rb = find((x, y)), find(n)
                    if ra != rb:
                        parent[ra] = rb

    groups = {}
    for c in cells:
        groups.setdefault(find(c), set()).add(c)

    new_map = {}
    used = set()
    for members in sorted(groups.values(), key=len, reverse=True):
        best_id, best_ov = None, 0
        for cid, prev in cluster_cells.items():
            if cid in used:
                continue
            ov = len(members & prev)
            if ov > best_ov:
                best_id, best_ov = cid, ov
        if best_id is None:
            best_id = _next_cid
            _next_cid += 1
        used.add(best_id)
        new_map[best_id] = members

    for cid in set(cluster_cells) - set(new_map):
        _drop_cluster(cid)
    cluster_cells = new_map


# =============================================================================
# =============================================================================
# 11. 볼록껍질 추정 -> 지도 반영                                    [정답 X]
# =============================================================================
# =============================================================================
#
# 파이프라인: 관측점 복원 -> 볼록껍질(convex_hull) -> 그대로 셀 채우기
#
#   estimate_obstacle   : 관측점만으로 볼록껍질 + 중심을 뽑는다   [순수 계산]
#   cells_of_hull        : 그 볼록껍질이 덮는 셀 목록을 만든다     [순수 계산]
#   update_obstacle_shape: grid 에 실제로 칠하고 소유권을 관리한다  [상태 변경]
#
# ★볼록껍질 전환으로 원형도(circ)/면적비(extent)/두께(thickness) 계산과
# 회전각 추정이 전부 없어졌다. 판단할 게 "이게 무슨 모양이냐"가 아니라
# "관측점을 감싸는 볼록다각형이 뭐냐"뿐이라 파이프라인이 훨씬 짧아졌다.

def estimate_obstacle(observations):
    """관측점만으로 볼록껍질을 만든다. 정답 미참조.

    장점: 관측이 늘수록 정확해진다. 부정형 물체에도 강하다(분류를 안
    하므로 오분류 자체가 없다). 관측된 표면 바깥으로 그리지 않으므로
    안전 측면에서 보수적이다.

    근본 한계: 오목한 형태는 볼록껍질 성질상 표현이 원리적으로 불가능
    하다. 관측을 아무리 늘려도 이 한계는 없어지지 않는다."""
    if len(observations) < SHAPE_MIN_OBS:
        return None
    pts = [(rx + d * math.cos(math.radians(a)), ry + d * math.sin(math.radians(a)))
           for a, d, rx, ry in observations]
    h = convex_hull(pts)
    if len(h) < 3:
        return None
    xs = [p[0] for p in h]
    ys = [p[1] for p in h]
    cx = sum(xs) / len(xs)
    cy = sum(ys) / len(ys)
    half_w = (max(xs) - min(xs)) / 2.0
    half_h = (max(ys) - min(ys)) / 2.0
    return h, cx, cy, half_w, half_h


def cells_of_hull(hull):
    """볼록껍질이 덮는 셀 목록. 정답 미참조.

    [중요] 판정 기준점은 셀 '중심'(x+0.5, y+0.5) 이다. 정수 좌표(칸의
    좌상단)를 그대로 쓰면 지도 표시와 반 칸씩 어긋난다."""
    xs = [p[0] for p in hull]
    ys = [p[1] for p in hull]
    x0 = max(0, int(math.floor(min(xs))))
    x1 = min(GRID_SIZE - 1, int(math.ceil(max(xs))))
    y0 = max(0, int(math.floor(min(ys))))
    y1 = min(GRID_SIZE - 1, int(math.ceil(max(ys))))
    out = []
    for y in range(y0, y1 + 1):
        for x in range(x0, x1 + 1):
            if point_in_convex_polygon(x + 0.5, y + 0.5, hull):
                out.append((x, y))
    return out


def update_obstacle_shape(g, cid):
    """클러스터의 관측점으로 볼록껍질을 추정해 그린다(속까지 채움).

    ★미확정 상태 삭제: 예전엔 관측지점 커버리지가 180도 미만이면 노란
    '미확정' 색으로 일단 그려두고, 나중에 180도를 넘기면 검정 '확정'으로
    바꿔 그렸다. 그런데 그 미확정 상태에서 관측이 늘 때마다 볼록껍질
    경계가 계속 움직여서(가끔은 줄어들기도 해서) 화면이 계속 모양을
    바꾸는 얼룩처럼 보였고, grid 가 채워졌다 비워졌다 하는 처치도
    잦아져(§17 시뮬 A/B에서 스텝 수 증가로 확인됨) 탐사 효율에도 안
    좋았다.

    이제 180도를 넘기 전까지는 지도에 아무것도 안 그린다 - 그 동안은
    직접관측(값3, 주황) 히트 점만 화면에 보인다. 180도를 넘기는 순간
    '한 번에' 확정된 검정 볼록껍질로 나타난다. "보이는 것만 장애물로
    표시한다"는 원칙: 로봇이 실제로 충분히 확인한 것만 지도에 반영하고,
    확신이 부족한 중간 상태는 아예 보여주지 않는다.

    [버그9, 유지] 클러스터 ID 하나만 받는다 - 정답 좌표는 안 받는다."""
    obs = cluster_obs(cid)
    if len(obs) < SHAPE_MIN_OBS:
        return
    # 광선각 스프레드는 값싼 사전 필터로만 쓴다(추정 자체를 시도할지 여부).
    if circular_spread([o[0] for o in obs]) < SHAPE_MIN_SPREAD:
        return
    est = estimate_obstacle(obs)
    if est is None:
        return
    hull, cx, cy, half_w, half_h = est

    # [수정2, 유지] 확정 판정은 추정 중심 기준 '관측지점 방위각' 커버리지로.
    vp_spread = viewpoint_spread(obs, cx, cy)
    if vp_spread < CONFIRM_SPREAD:
        return   # ★아직 확정 전 - 지도에 아무것도 안 그리고 그냥 기다린다.

    obstacle_hull[cid] = hull
    # rot_deg 자리는 항상 0.0(볼록껍질 좌표 자체가 회전을 담고 있어 불필요).
    # shape 자리는 항상 "hull". value 자리는 이제 항상 1 - 여기 도달했다는
    # 것 자체가 이미 확정됐다는 뜻이라 더 이상 값을 고를 필요가 없다.
    obstacle_render_info[cid] = ("hull", cx, cy, max(half_w, half_h), 0.0,
                                 1, half_w, half_h)

    new_cells = cells_of_hull(hull)
    new_set = set(new_cells)

    # [버그3, 유지] 되돌림은 '내가 소유한 칸'에 대해서만.
    for cell in last_filled.get(cid, ()):
        if cell in new_set or cell_owner.get(cell) != cid:
            continue
        x, y = cell
        if g[y][x] == 1:
            g[y][x] = -1
        cell_owner.pop(cell, None)

    for cell in new_cells:
        owner = cell_owner.get(cell)
        if owner is not None and owner != cid:
            continue
        x, y = cell
        if g[y][x] == -1:
            g[y][x] = 1
        cell_owner[cell] = cid

    last_filled[cid] = new_cells


# =============================================================================
# =============================================================================
# 12. 경로 계획 (도달성)                                            [정답 X]
# =============================================================================
# =============================================================================

def _build_move_table():
    table = []
    for (dx, dy), cost in get_neighbors_cost(0, 0):
        seg_len2 = dx * dx + dy * dy
        mids = []
        for a in range(min(0, dx), max(0, dx) + 1):
            for b in range(min(0, dy), max(0, dy) + 1):
                if (a, b) in ((0, 0), (dx, dy)):
                    continue
                if abs(dx * b - dy * a) / math.sqrt(seg_len2) <= 0.72:
                    mids.append((a, b))
        table.append(((dx, dy), cost, tuple(mids)))
    return tuple(table)


MOVE_TABLE = _build_move_table()


def _build_turn_diff_table():
    """MOVE_TABLE 의 이동방향 i -> j 로 바뀔 때의 '90도 단위 회전량'을
    미리 계산해둔다.  ★성능 최적화

    [왜 필요한가 - 실측 프로파일]
      turn_penalty() 가 전체 실행시간의 31%(60초 중 18.7초)를 먹고 있었다.
      bfs_from 의 간선 완화마다 호출되어 1,190만 번 돌았고, 매번
      atan2 2회 + degrees 2회를 계산했다(atan2 만 2,380만 콜/4.5초).

      그런데 이동 방향은 16개짜리 고정 테이블이라 조합이 16x16=256 가지뿐이다.
      런타임에 매번 삼각함수를 돌 이유가 전혀 없어서 전부 미리 계산한다.

    ★ TURN_COST 를 곱하지 않은 '순수 기하량'만 저장한다 - CLI(--turn-cost)로
      런타임에 값을 바꿀 수 있으므로, 곱셈은 쓰는 시점에 한다. 여기에
      TURN_COST 를 미리 곱해 넣으면 플래그가 조용히 안 먹는 버그가 된다."""
    n = len(MOVE_TABLE)
    angs = [math.degrees(math.atan2(dy, dx)) for (dx, dy), _, _ in MOVE_TABLE]
    table = []
    for a0 in angs:
        row = []
        for a1 in angs:
            row.append(abs((a1 - a0 + 180) % 360 - 180) / 90.0)
        table.append(tuple(row))
    return tuple(table)


TURN_DIFF_TABLE = _build_turn_diff_table()


def planning_grid(g, strict=True):
    """경로계획용으로 값을 치환한 지도 사본. 확정 채움(1)을 막고 싶으면
    3으로 바꿔서 넘긴다. ★미확정(2)은 더 이상 grid 에 나오지 않으므로
    따로 처리할 게 없다."""
    if not (strict and SAFE_BLOCK_ESTIMATED):
        return g
    gg = g.copy()
    gg[gg == 1] = 3
    return gg


def turn_penalty(prev_dir, dx, dy):
    """진입 방향이 바뀔 때 무는 비용.

    ★ bfs_from 은 더 이상 이 함수를 쓰지 않는다(TURN_DIFF_TABLE 조회로
      대체 - 위 _build_turn_diff_table 주석 참고). 외부/테스트에서
      임의 방향으로 회전 비용을 물어볼 때를 위해 남겨둔 참조 구현이다."""
    if prev_dir is None or TURN_COST <= 0:
        return 0.0
    a0 = math.degrees(math.atan2(prev_dir[1], prev_dir[0]))
    a1 = math.degrees(math.atan2(dy, dx))
    diff = abs((a1 - a0 + 180) % 360 - 180)
    return TURN_COST * (diff / 90.0)


def bfs_from(g, sx, sy, heading_deg=None):
    """로봇에서 도달 가능한 칸 전체 + 부모포인터/누적비용/진입방향.

    reach[(x,y)] = (부모좌표, 누적비용, 진입방향_인덱스)
      ★ 인덱스 2 의 의미가 바뀌었다: 예전엔 (dx,dy) 튜플이었는데 이제
        MOVE_TABLE 의 인덱스(int) 다. TURN_DIFF_TABLE 을 O(1) 로 조회하기
        위해서다. 외부 코드는 인덱스 0(부모)과 1(비용)만 쓰므로 영향 없다
        (path_from/목표선정/find_confirm_point 전부 확인함).

    [근사 주의] 회전 비용을 정확히 다루려면 상태가 (x, y, 방향)이어야 한다.
    여기서는 부모에 기록된 진입 방향만 보므로, 한번 확정된 칸에 나중에 더
    좋은 방향으로 도달해도 갱신되지 않는다. 최적은 아니지만 상태 수를
    16배로 늘리지 않고 회전을 크게 줄인다."""
    use_turn = TURN_COST > 0

    # 시작 헤딩은 16방향 중 하나가 아니라 임의 실수각이라 테이블에 없다.
    # bfs_from 한 번당 16개만 계산하면 되므로 여기서 미리 뽑아둔다.
    start_diffs = None
    if heading_deg is not None and use_turn:
        start_diffs = tuple(
            abs((math.degrees(math.atan2(dy, dx)) - heading_deg + 180) % 360 - 180) / 90.0
            for (dx, dy), _, _ in MOVE_TABLE)

    for strict in (True, False):
        pg = planning_grid(g, strict)
        cmap = compute_clearance_map(pg)
        smap = compute_safe_map(pg)
        # 진입방향 인덱스: -1 = 시작 칸(아직 이동 안 함, start_diffs 사용)
        reach = {(sx, sy): (None, 0.0, -1)}
        pq = [(0.0, sx, sy)]
        done = set()
        while pq:
            d, cx, cy = heapq.heappop(pq)
            if (cx, cy) in done:
                continue
            done.add((cx, cy))
            prev_i = reach[(cx, cy)][2]
            # 이 칸에서 나가는 16방향 각각의 회전 벌점을 한 번에 고른다
            if not use_turn:
                diffs = None
            elif prev_i >= 0:
                diffs = TURN_DIFF_TABLE[prev_i]
            else:
                diffs = start_diffs          # None 이면 시작 헤딩 미지정 = 벌점 없음
            for i, ((dx, dy), stepc, mids) in enumerate(MOVE_TABLE):
                nx, ny = cx + dx, cy + dy
                if not (0 <= nx < GRID_SIZE and 0 <= ny < GRID_SIZE):
                    continue
                if not smap[ny][nx]:
                    continue
                blocked = False
                for mdx, mdy in mids:
                    if not smap[cy + mdy][cx + mdx]:
                        blocked = True
                        break
                if blocked:
                    continue
                lack = CLEARANCE_WANT - cmap[ny][nx]
                nd = d + stepc
                if diffs is not None:
                    nd += TURN_COST * diffs[i]
                if lack > 0:
                    nd += CLEARANCE_COST * lack
                if (nx, ny) not in reach or nd < reach[(nx, ny)][1]:
                    reach[(nx, ny)] = ((cx, cy), nd, i)
                    heapq.heappush(pq, (nd, nx, ny))
        if len(reach) > 1 or not (strict and SAFE_BLOCK_ESTIMATED):
            return reach
    return reach


def path_from(reach, target):
    if target not in reach:
        return None
    path, cur = [], target
    while reach[cur][0] is not None:
        path.append(cur)
        cur = reach[cur][0]
    path.reverse()
    return path


# =============================================================================
# =============================================================================
# 13. 가시선 / 탐색 목표 선정                                       [정답 X]
# =============================================================================
# =============================================================================

def has_line_of_sight(g, x0, y0, x1, y1, max_range=SCAN_RANGE):
    """(x0,y0) 에서 (x1,y1) 이 보이는지.

    [수정4] 무엇을 시야 차단으로 볼지가 LOS_BLOCK_CONFIRMED 로 갈린다.
    값 3(직접관측)만 막으면 광선이 드문드문한 관측점 사이를 빠져나가
    장애물 뒤 미탐색 칸까지 '보인다'고 오판한다. 확정(1)까지 막으면
    헛걸음은 줄지만 추정 도형이 과대할 때 빈 칸을 조기에 포기해버린다.

    ★성능: 실측 프로파일에서 이 함수가 48만 콜/10.4초로 최대 병목이었다
      (can_observe/find_observation_point 가 사거리 박스를 전수 훑으며
      칸마다 부른다). 아래 미세최적화는 전부 '결과가 완전히 동일한' 것만
      골랐다 - 판정 로직을 바꾸면 탐사 경로 자체가 달라져 그동안 쌓은
      실측 비교가 전부 무의미해지기 때문이다:
        - g[y][x] -> g[y, x] : 전자는 행 뷰(ndarray) 를 만든 뒤 다시
          인덱싱해서 임시 객체가 생긴다. 후자는 스칼라 직접 조회.
        - `in blockers` 튜플 멤버십 -> 정수 비교 2회
        - 루프 불변식(dx, dy, blk_confirmed) 을 루프 밖으로 호이스팅
      ※ int(round(...)) 는 그대로 뒀다. round 는 은행가 반올림이라
        int(v+0.5) 로 바꾸면 정확히 .5 인 지점에서 결과가 갈린다."""
    dist = math.hypot(x1 - x0, y1 - y0)
    if dist > max_range:
        return False
    steps = int(dist) + 1
    dx, dy = x1 - x0, y1 - y0
    blk_confirmed = LOS_BLOCK_CONFIRMED
    for i in range(1, steps + 1):
        t = i / steps
        x = int(round(x0 + dx * t))
        y = int(round(y0 + dy * t))
        if x == x1 and y == y1:
            return True
        if not (0 <= x < GRID_SIZE and 0 <= y < GRID_SIZE):
            return False
        v = g[y, x]
        if v == 3 or (blk_confirmed and v == 1):
            return False
    return True


def find_nearest_unknown(g, rx, ry):
    ys, xs = np.where(g == -1)
    if xs.size == 0:
        return None
    d = np.abs(xs - rx) + np.abs(ys - ry)
    i = int(np.argmin(d))
    return int(xs[i]), int(ys[i])


def find_confirm_point(g, reach, cx, cy, bearing_deg, max_range=None):
    if max_range is None:
        max_range = SCAN_RANGE
    icx, icy = int(round(cx)), int(round(cy))
    best, best_cost = None, float('inf')
    d_lo = max(3, CONFIRM_STANDOFF - 4)
    d_hi = min(max_range, CONFIRM_STANDOFF + 6)

    for dth in range(-90, 91, 10):
        th = math.radians(bearing_deg + dth)
        ct, st = math.cos(th), math.sin(th)
        for d in range(d_lo, d_hi + 1):
            px = int(round(cx + d * ct))
            py = int(round(cy + d * st))
            if not (0 <= px < GRID_SIZE and 0 <= py < GRID_SIZE):
                continue
            if g[py][px] != 0 or (px, py) not in reach:
                continue
            if not has_line_of_sight(g, px, py, icx, icy, max_range):
                continue
            cost = (reach[(px, py)][1]
                    + abs(dth) * CONFIRM_BEARING_PENALTY
                    + abs(d - CONFIRM_STANDOFF) * 0.3)
            if cost < best_cost:
                best_cost, best = cost, (px, py)
    return best


def can_observe(g, reach, tx, ty, max_range=SCAN_RANGE):
    for y in range(max(0, ty - max_range), min(GRID_SIZE, ty + max_range + 1)):
        for x in range(max(0, tx - max_range), min(GRID_SIZE, tx + max_range + 1)):
            if g[y][x] != 0 or (x, y) not in reach:
                continue
            if math.hypot(x - tx, y - ty) > max_range:
                continue
            if has_line_of_sight(g, x, y, tx, ty, max_range):
                return True
    return False


def find_observation_point(g, reach, tx, ty, max_range=SCAN_RANGE):
    best, best_d = None, float('inf')
    for y in range(max(0, ty - max_range), min(GRID_SIZE, ty + max_range + 1)):
        for x in range(max(0, tx - max_range), min(GRID_SIZE, tx + max_range + 1)):
            if g[y][x] != 0 or (x, y) not in reach:
                continue
            if math.hypot(x - tx, y - ty) > max_range:
                continue
            if not has_line_of_sight(g, x, y, tx, ty, max_range):
                continue
            d = reach[(x, y)][1]
            if d < best_d:
                best_d, best = d, (x, y)
    return best


# =============================================================================
# =============================================================================
# 14. 충돌 범퍼                                    [정답 O / 시뮬 전용]
# =============================================================================
# =============================================================================

def bump_cells(px, py, nx, ny):
    hits = set()
    dist = math.hypot(nx - px, ny - py)
    n = max(2, int(dist / 0.3))
    for i in range(n + 1):
        t = i / n
        cx = int(round(px + (nx - px) * t))
        cy = int(round(py + (ny - py) * t))
        for by in range(cy - ROBOT_MARGIN, cy + ROBOT_MARGIN + 1):
            for bx in range(cx - ROBOT_MARGIN, cx + ROBOT_MARGIN + 1):
                if 0 <= bx < GRID_SIZE and 0 <= by < GRID_SIZE and truth_id[by][bx] >= 0:
                    hits.add((bx, by))
    return hits


# =============================================================================
# =============================================================================
# 15. 모터 명령 (실기 접합부)
# =============================================================================
# =============================================================================

def send_command(cmd):
    """실기에서는 motor_commander.send_command 로 교체."""
    pass


# =============================================================================
# =============================================================================
# 16. 채점                                         [정답 O / 시뮬 전용]
# =============================================================================
# =============================================================================
#
# ★볼록껍질 전환: shape_acc(형태 분류 정확도) 지표를 제거했다 - 더 이상
# 형태를 분류하지 않으므로 "맞혔는가"를 잴 대상이 없다. 대신 center_err
# (추정 중심이 정답 중심에서 얼마나 벗어났는지)와 IoU(지도 전체 겹침도)로
# 정확도를 판단한다. IoU 가 이 방식의 핵심 지표다 - 볼록껍질이 정답 영역을
# 얼마나 정확히 덮는지를 그대로 보여준다.

def evaluate():
    """정답과 비교한 지표. 시뮬 채점용이며 로봇 로직은 이걸 참조하지 않는다."""
    occ_true = (truth_id >= 0)
    occ_est = np.isin(grid, [1, 3])   # 값 2(미확정)는 이제 없음
    inter = int((occ_true & occ_est).sum())
    union = int((occ_true | occ_est).sum())

    merged = 0
    covered = set()
    center_errs = []
    confirmed = 0
    for cid, members in cluster_cells.items():
        true_ids = {int(truth_id[y][x]) for (x, y) in members if truth_id[y][x] >= 0}
        covered |= true_ids
        if len(true_ids) > 1:
            merged += 1
        ri = obstacle_render_info.get(cid)
        if ri is not None and ri[5] == 1:
            confirmed += 1
        if ri is None or len(true_ids) != 1:
            continue
        counts = {}
        for (x, y) in members:
            t = int(truth_id[y][x])
            if t >= 0:
                counts[t] = counts.get(t, 0) + 1
        if not counts:
            continue
        best_t = max(counts, key=counts.get)
        ox, oy = obstacle_ids[best_t]
        center_errs.append(math.hypot(ri[1] - ox, ri[2] - oy))

    phantom = int(((grid == 3) & (truth_id < 0)).sum())

    if CONTINUOUS_SCAN:
        t_est = sim_time
    else:
        t_est = (rays_fired * STATIONARY_RAY_S
                 + moves * CELL_SIZE_M / ROBOT_SPEED_MPS
                 + turn_deg / ROBOT_TURN_DPS)

    return {
        "steps": step,
        "moves": moves,
        "sim_time_s": t_est,
        "rays": rays_fired,
        "confirm_done": confirm_done,
        "confirm_gained": confirm_gained,
        "returned": 1 if returned else 0,
        "collisions": collisions,
        "turns": turns,
        "turn_deg": turn_deg,
        "aim_turns": aim_turns,
        "aim_turn_deg": aim_turn_deg,
        "aim_blocked": aim_blocked,
        "phantom": phantom,
        "unknown_left": int((grid == -1).sum()),
        "pink": int((grid == 4).sum()),
        "iou": (inter / union) if union else 0.0,
        "obstacles": len(hidden_obstacles),
        "clusters": len(cluster_cells),
        "confirmed": confirmed,
        "merged": merged,
        "missed": len(hidden_obstacles) - len(covered),
        "center_err": (sum(center_errs) / len(center_errs)) if center_errs else float('nan'),
        "max_obs": max((cluster_obs_count(c) for c in cluster_cells), default=0),
        "rescuee_placed": rescuee_truth is not None,
        "rescuee_discovered": rescuee_discovered,
        "rescuee_discover_step": rescuee_discover_step,
    }


# =============================================================================
# =============================================================================
# 17. 메인 루프
# =============================================================================
# =============================================================================

def _next_phase_after_survey():
    if CONFIRM_PHASE:
        return "confirm"
    if RETURN_HOME:
        return "return"
    return "done"


def run(seed, visualize=True, verbose=True, pause=0.05):
    global robot_x, robot_y, robot_angle, step, collisions, grid
    global turns, turn_deg, moves
    global phase, return_path, confirm_cid, confirm_done, confirm_gained
    global returned, rescuee_discovered, rescuee_discover_step

    reset_world(seed)
    viz = _Viz(seed) if visualize else None

    committed = None
    commit_steps = 0
    cleanup_phase = False
    last_progress = 0
    prev_rem = 10 ** 9
    reason = "완료"
    survey_note = ""
    moved_last_step = True
    confirm_visits = {}
    confirm_start = 0

    while True:
        step += 1
        if step > MAX_STEPS:
            reason = f"MAX_STEPS({MAX_STEPS}) 초과"
            break

        if CONTINUOUS_SCAN:
            if step == 1:
                sweep_in_place(FULL_SWEEP_S)
            elif not moved_last_step:
                sweep_in_place(IDLE_SWEEP_S)
        else:
            for angle in get_scan_angles(robot_angle):
                hit_cell, dist = real_scan(grid, robot_x, robot_y, angle)
                if hit_cell is not None:
                    record_hit(hit_cell, angle, dist, robot_x, robot_y)

        # ── 요구조자 발견 체크 (§1-12) ──────────────────────────────────
        # ★ 정답(rescuee_truth)을 읽는 건 '지금 보이는가'를 판정할 때뿐이다.
        #   위치 자체를 로봇 로직(경로계획 등) 어디에도 넘기지 않는다 -
        #   발견되기 전까지는 이 블록 밖에서 rescuee_truth 를 참조하는 코드가
        #   없어야 한다(hidden_obstacles/truth_id 와 동일한 격리 원칙).
        #   판정 기준은 다른 곳과 동일하게 사거리+가시선(has_line_of_sight) -
        #   ToF 가 실제로 그 지점까지 뚫려 있어야 '봤다'고 인정한다.
        if rescuee_truth is not None and not rescuee_discovered:
            tx, ty = rescuee_truth
            if math.hypot(tx - robot_x, ty - robot_y) <= SCAN_RANGE \
                    and has_line_of_sight(grid, robot_x, robot_y, tx, ty):
                rescuee_discovered = True
                rescuee_discover_step = step

        nu0 = find_nearest_unknown(grid, robot_x, robot_y)
        if nu0 is not None and math.hypot(nu0[0] - robot_x, nu0[1] - robot_y) <= SCAN_RANGE \
                and has_line_of_sight(grid, robot_x, robot_y, nu0[0], nu0[1]):
            aim0 = math.degrees(math.atan2(nu0[1] - robot_y, nu0[0] - robot_x))
            if CONTINUOUS_SCAN:
                aim_sweep(grid, aim0)
            else:
                scan_toward(grid, robot_x, robot_y, aim0)

        if step % CLUSTER_EVERY == 0 or step == 1:
            rebuild_clusters()

        for cid in list(cluster_cells.keys()):
            if cid in obstacle_render_info:
                continue    # ★이미 확정되어 그려짐 - 더 갱신하지 않는다(안정화).
                            #   미확정 상태가 없어졌으니 확정 후에는 굳이
                            #   매번 볼록껍질을 다시 계산해 grid 를 들쑤실
                            #   이유가 없다 - 이게 §17 실측에서 나타났던
                            #   "볼록껍질 churn 으로 스텝 증가" 문제의 원인이었다.
            c = cluster_obs_count(cid)
            if c >= SHAPE_MIN_OBS and c - last_obs_count.get(cid, 0) >= 6:
                last_obs_count[cid] = c
                update_obstacle_shape(grid, cid)

        for cid in list(cluster_cells.keys()):
            if evaluate_confidence(cid) < 0.5:
                low_confidence_obstacles.add(cid)
            else:
                low_confidence_obstacles.discard(cid)

        if phase in ("explore", "cleanup") and int((grid == -1).sum()) == 0:
            phase = _next_phase_after_survey()
            confirm_start = step

        if phase == "done":
            break

        reach = bfs_from(grid, robot_x, robot_y, robot_angle)
        frontiers = (find_frontiers(grid)
                     if phase in ("explore", "cleanup") else [])
        reachable = [f for f in frontiers
                     if f in reach and reach[f][1] > 0 and f not in unreachable_unknowns]

        target = path = None

        if phase == "explore" and reachable:
            committed = None
            target = min(reachable, key=lambda f: reach[f][1])
            path = path_from(reach, target)

        elif phase in ("explore", "cleanup"):
            if phase == "explore":
                phase = "cleanup"
                cleanup_phase = True
                last_progress = step

            cur_rem = int((grid == -1).sum())
            if cur_rem < prev_rem:
                last_progress = step
            prev_rem = cur_rem
            if step - last_progress > STALL_LIMIT:
                grid[grid == -1] = 4
                survey_note = "정체로 잔여 미탐색 포기"
                phase = _next_phase_after_survey()
                confirm_start = step

            elif step % ABANDON_EVERY == 0:
                budget = ABANDON_BUDGET
                ys, xs = np.where(grid == -1)
                for x, y in zip(xs.tolist(), ys.tolist()):
                    if budget <= 0:
                        break
                    if (x, y) in unreachable_unknowns:
                        continue
                    budget -= 1
                    if not can_observe(grid, reach, x, y):
                        grid[y][x] = 4
                        unreachable_unknowns.add((x, y))

            if phase == "cleanup" and int((grid == -1).sum()) == 0:
                phase = _next_phase_after_survey()
                confirm_start = step

            if phase == "cleanup":
                if committed is None or grid[committed[1]][committed[0]] != -1 \
                        or committed in unreachable_unknowns:
                    committed = find_nearest_unknown(grid, robot_x, robot_y)
                    commit_steps = 0
                if committed is None:
                    phase = _next_phase_after_survey()
                    confirm_start = step
                else:
                    commit_steps += 1
                    tx, ty = committed

                    if math.hypot(tx - robot_x, ty - robot_y) <= SCAN_RANGE \
                            and has_line_of_sight(grid, robot_x, robot_y, tx, ty):
                        aim_t = math.degrees(math.atan2(ty - robot_y, tx - robot_x))
                        if CONTINUOUS_SCAN:
                            aim_sweep(grid, aim_t)
                        else:
                            scan_toward(grid, robot_x, robot_y, aim_t)
                        if grid[ty][tx] == -1:
                            grid[ty][tx] = 4
                            unreachable_unknowns.add((tx, ty))
                        committed = None
                    elif commit_steps > COMMIT_LIMIT:
                        grid[ty][tx] = 4
                        unreachable_unknowns.add((tx, ty))
                        committed = None
                    else:
                        op = find_observation_point(grid, reach, tx, ty)
                        path = path_from(reach, op) if op else None
                        if not path:
                            grid[ty][tx] = 4
                            unreachable_unknowns.add((tx, ty))
                            committed = None
                        else:
                            target = op

        elif phase == "confirm":
            # ★미확정 상태 삭제로 pending 클러스터는 obstacle_render_info 가
            # 없다(needs_confirm 정의 자체가 "없으면 확인 대상"이므로).
            # 그래서 중심좌표는 cluster_center_estimate() 로 따로 구한다 -
            # 히트 셀 평균이라 볼록껍질 중심만큼 정밀하진 않지만, 접근점을
            # 잡는 용도로는 충분하다.
            pending = [c for c in cluster_cells
                       if needs_confirm(c)
                       and confirm_visits.get(c, 0) < CONFIRM_MAX_VISITS
                       and cluster_center_estimate(c) is not None]

            if not pending or step - confirm_start > CONFIRM_STEP_BUDGET:
                confirm_cid = None
                phase = "return" if RETURN_HOME else "done"

            else:
                if confirm_cid not in pending:
                    confirm_cid = min(
                        pending,
                        key=lambda c: math.hypot(
                            cluster_center_estimate(c)[0] - robot_x,
                            cluster_center_estimate(c)[1] - robot_y))
                cid = confirm_cid
                center = cluster_center_estimate(cid)
                if center is None:
                    confirm_visits[cid] = CONFIRM_MAX_VISITS
                    confirm_cid = None
                else:
                    ccx, ccy = center
                    bearing = viewpoint_gap_bearing(cluster_obs(cid), ccx, ccy)
                    op = find_confirm_point(grid, reach, ccx, ccy, bearing)

                    if op is None:
                        confirm_visits[cid] = CONFIRM_MAX_VISITS
                        confirm_cid = None
                    elif math.hypot(op[0] - robot_x, op[1] - robot_y) < 1.5:
                        aim_c = math.degrees(math.atan2(ccy - robot_y,
                                                        ccx - robot_x))
                        if CONTINUOUS_SCAN:
                            aim_sweep(grid, aim_c)
                        else:
                            scan_toward(grid, robot_x, robot_y, aim_c)
                        rebuild_clusters()
                        if cid in cluster_cells:
                            last_obs_count[cid] = cluster_obs_count(cid)
                            update_obstacle_shape(grid, cid)
                        confirm_visits[cid] = confirm_visits.get(cid, 0) + 1
                        confirm_done += 1
                        # pending 은 애초에 '미확정'이었으므로, 방문 후
                        # obstacle_render_info 에 들어왔다면 이번에 새로
                        # 확정된 것이다.
                        if cid in obstacle_render_info:
                            confirm_gained += 1
                        confirm_cid = None
                    else:
                        target = op
                        path = path_from(reach, op)
                        if not path:
                            confirm_visits[cid] = CONFIRM_MAX_VISITS
                            confirm_cid = None

        elif phase == "return":
            if (robot_x, robot_y) == home:
                returned = True
                return_path = []
                reason = "복귀 완료"
                phase = "done"
            else:
                path = path_from(reach, home)
                if path is None:
                    saved = SAFE_BLOCK_ESTIMATED
                    globals()['SAFE_BLOCK_ESTIMATED'] = False
                    reach = bfs_from(grid, robot_x, robot_y, robot_angle)
                    globals()['SAFE_BLOCK_ESTIMATED'] = saved
                    path = path_from(reach, home)
                if path is None:
                    reason = "복귀 경로 없음"
                    phase = "done"
                else:
                    target = home
                    return_path = [(robot_x, robot_y)] + list(path)

        moved_last_step = False
        if path:
            prev_x, prev_y = robot_x, robot_y
            nxt_x, nxt_y = path[0]
            hits = bump_cells(prev_x, prev_y, nxt_x, nxt_y)
            if hits:
                collisions += 1
                for bx, by in hits:
                    grid[by][bx] = 3
                    cell_owner.pop((bx, by), None)
            else:
                dx, dy = nxt_x - prev_x, nxt_y - prev_y
                new_angle = math.degrees(math.atan2(dy, dx))
                diff = (new_angle - robot_angle + 180) % 360 - 180

                if abs(diff) > 5:
                    turns += 1
                    turn_deg += abs(diff)
                    if CONTINUOUS_SCAN:
                        sweep_while_turning(prev_x, prev_y, robot_angle, diff)
                    send_command({"cmd": "turn_left" if diff > 0 else "turn_right",
                                  "speed": 100, "duration_ms": int(abs(diff) * 10)})
                robot_angle = new_angle

                if CONTINUOUS_SCAN:
                    sweep_while_moving(prev_x, prev_y, nxt_x, nxt_y, new_angle)
                send_command({"cmd": "forward", "speed": 100, "duration_ms": 200})

                robot_x, robot_y = nxt_x, nxt_y
                moves += 1
                moved_last_step = True

        if viz:
            viz.progress(target, frontiers, phase != "explore")

    if USE_CONE_TRIM:
        _flush_sweep_buffer(end_is_limit=False)

    result = evaluate()
    result["seed"] = seed
    result["reason"] = reason + ((" / " + survey_note) if survey_note else "")
    if verbose:
        print(f"[seed {seed:4d}] {reason} | 스텝 {result['steps']:4d} "
              f"| 충돌 {result['collisions']:3d} | 회전 {result['turns']:3d}회"
              f"({result['turn_deg']:.0f}도, 조준 {result['aim_turns']:3d}회"
              f"/{result['aim_turn_deg']:.0f}도) | 유령 {result['phantom']:3d} "
              f"| 미탐색 {result['unknown_left']:4d} "
              f"| 핑크 {result['pink']:4d} | IoU {result['iou']:.3f} "
              f"| 클러스터 {result['clusters']:2d}/{result['obstacles']:2d} "
              f"(확정 {result['confirmed']}, 병합 {result['merged']}, "
              f"놓침 {result['missed']}) "
              f"| 중심오차 {result['center_err']:.2f} "
              f"| 확정승격 +{result['confirm_gained']} "
              f"| 복귀 {'O' if result['returned'] else 'X'} "
              f"| 실물 {result['sim_time_s']/60:.1f}분")
    if viz:
        viz.final()
    return result


# =============================================================================
# =============================================================================
# 18. 시각화
# =============================================================================
# =============================================================================

class _Viz:

    def __init__(self, seed):
        import matplotlib.pyplot as plt
        from matplotlib.colors import ListedColormap
        self.plt = plt
        plt.rcParams['font.family'] = 'sans-serif'
        plt.rcParams['font.sans-serif'] = ['Malgun Gothic', 'NanumGothic',
                                           'Noto Sans CJK KR', 'AppleGothic',
                                           'DejaVu Sans']
        plt.rcParams['axes.unicode_minus'] = False
        self.cmap = ListedColormap(['lightgray', 'white', 'black',
                                    'yellow', 'orange', 'pink'])
        plt.ion()
        self.fig = plt.figure(figsize=(7, 7))
        self.selected = None
        self.fig.canvas.mpl_connect('button_press_event', self._on_click)
        self._show_truth(seed)

    def _on_click(self, event):
        if event.inaxes is None or event.xdata is None or event.ydata is None:
            return
        self.selected = (int(round(event.xdata)), int(round(event.ydata)))

    def _show_truth(self, seed):
        from matplotlib.colors import ListedColormap
        from matplotlib.patches import Circle, Rectangle, Polygon
        plt = self.plt
        fig = plt.figure(figsize=(6, 6))
        ax = fig.add_subplot(111)

        if not TRUTH_SMOOTH:
            ax.imshow((truth_id >= 0).astype(int),
                      cmap=ListedColormap(['white', 'black']),
                      origin='lower', vmin=0, vmax=1)
        else:
            raster_color = '#d9d9d9' if TRUTH_SHOW_RASTER else 'white'
            ax.imshow((truth_id >= 0).astype(int),
                      cmap=ListedColormap(['white', raster_color]),
                      origin='lower', vmin=0, vmax=1)
            for (ox, oy), info in hidden_obstacles.items():
                half = obstacle_half(info['size'])
                style = dict(facecolor='black', edgecolor='none', zorder=3)
                if info['shape'] == 'circle':
                    p = Circle((ox, oy), radius=half, **style)
                elif info['shape'] == 'triangle':
                    p = Polygon([(ox, oy + half), (ox - half, oy - half),
                                 (ox + half, oy - half)], closed=True, **style)
                else:
                    p = Rectangle((ox - half, oy - half), 2 * half, 2 * half, **style)
                ax.add_patch(p)
            ax.set_xlim(-0.5, GRID_SIZE - 0.5)
            ax.set_ylim(-0.5, GRID_SIZE - 0.5)
            ax.set_aspect('equal')

        ax.set_title(f'실제 정답 지도 (seed {seed})')
        fig.tight_layout()
        plt.figure(self.fig.number)

    def _display_grid(self):
        disp = grid.copy()
        for oid in obstacle_render_info:
            for x, y in last_filled.get(oid, ()):
                if disp[y][x] == 1:
                    disp[y][x] = 0
        return disp

    def _patches(self):
        """볼록껍질을 그대로 벡터 폴리곤으로 그린다.

        ★볼록껍질 전환: 예전엔 shape 값(circle/triangle/rect)에 따라
        Circle/Polygon/Rectangle 을 분기했는데, 이제 shape 은 항상 "hull"
        이라 분기 없이 obstacle_hull 의 좌표를 그대로 Polygon 에 넣는다."""
        from matplotlib.patches import Polygon
        ax = self.plt.gca()
        for cid, (shape, cx, cy, half, rot_deg, value,
                  draw_w, draw_h) in obstacle_render_info.items():
            color = 'black' if value == 1 else 'yellow'
            hull = obstacle_hull.get(cid)
            if not hull:
                continue
            p = Polygon(hull, closed=True, facecolor=color,
                       edgecolor='none', zorder=3)
            ax.add_patch(p)

    def _info_box(self):
        if self.selected is None:
            return
        sx, sy = self.selected
        if not (0 <= sx < GRID_SIZE and 0 <= sy < GRID_SIZE):
            return
        v = grid[sy][sx]
        if v == -1:
            txt = f"위치 : ({sx}, {sy})\n상태: 미탐색"
        elif v == 0:
            txt = f"위치 : ({sx}, {sy})\n상태 : 빈공간"
        elif v == 4:
            txt = f"위치 : ({sx}, {sy})\n상태: 확인 불가(포기)"
        else:
            status = {3: "직접 관측", 1: "확정됨"}.get(int(v), "추정 중(미확정)")
            cid = None
            for k, cells in last_filled.items():
                if (sx, sy) in set(cells):
                    cid = k
                    break
            if cid is None and (sx, sy) in hit_obs:
                cid = next((k for k, m in cluster_cells.items() if (sx, sy) in m), None)
            if cid is not None:
                obs = cluster_obs(cid)
                ri = obstacle_render_info.get(cid)
                ray_sp = circular_spread([o[0] for o in obs])
                vp_sp = viewpoint_spread(obs, ri[1], ri[2]) if ri else 0.0
                nvp = len({(rx, ry) for _, _, rx, ry in obs})
                hull_n = len(obstacle_hull.get(cid, ()))
                txt = (f"위치 : ({sx}, {sy})\n상태: 장애물({status})\n"
                       f"클러스터 #{cid} (히트셀 {len(cluster_cells.get(cid, ()))}개)\n"
                       f"볼록껍질 꼭짓점: {hull_n}개\n"
                       f"관측횟수: {len(obs)} (관측지점 {nvp}곳)\n"
                       f"광선각 커버리지: {ray_sp:.0f}도\n"
                       f"관측지점 커버리지: {vp_sp:.0f}도 (확정 기준 {CONFIRM_SPREAD}도)\n"
                       f"신뢰도: {evaluate_confidence(cid):.1f}")
            else:
                txt = f"위치 : ({sx}, {sy})\n상태: 장애물({status})\n관측횟수: 기록없음"
        self.plt.gcf().text(0.02, 0.98, txt, fontsize=10, verticalalignment='top',
                            bbox=dict(boxstyle='round', facecolor='white',
                                      edgecolor='black'))

    def progress(self, target, frontiers, cleanup_phase):
        plt = self.plt
        plt.clf()
        plt.imshow(self._display_grid(), cmap=self.cmap, origin='lower', vmin=-1, vmax=4)
        self._patches()
        for c, lab in (('orange', '직접 관측'), ('yellow', '추정 중(미확정)'),
                       ('black', '확정됨'), ('pink', '확인 불가')):
            plt.plot([], [], marker='s', color=c, markersize=8, label=lab, linestyle='None')
        if frontiers:
            plt.scatter([f[0] for f in frontiers], [f[1] for f in frontiers], c='blue', s=3)
        if target is not None:
            plt.scatter(target[0], target[1], c='green', s=80, marker='*')
        plt.scatter(robot_x, robot_y, c='red', s=80, marker='^')
        plt.legend(loc='upper right')
        phase = 'cleanup' if cleanup_phase else '탐사'
        plt.title(f'{phase} 중 (스텝 : {step}) 충돌 : {collisions}회 '
                  f'미확정 장애물 : {len(low_confidence_obstacles)}개')
        self._info_box()
        plt.pause(0.05)

    def final(self):
        plt = self.plt
        plt.clf()
        plt.imshow(self._display_grid(), cmap=self.cmap, origin='lower', vmin=-1, vmax=4)
        self._patches()
        plt.scatter(robot_x, robot_y, c='red', s=80, marker='^')
        plt.title(f'탐사 완료 (총 {step}스텝, 충돌 {collisions}회)')
        plt.ioff()
        plt.show()


# =============================================================================
# =============================================================================
# 19. 엔트리포인트
# =============================================================================
# =============================================================================
#
# 사용법:
#   py path_planner_sim2.py                        시드 14 시각화 실행
#   py path_planner_sim2.py --seeds 12 --headless  배치 채점
#
# ★볼록껍질 전환: --shape-v1/--shape-rule/--no-hold-small/--vp-spread 등
# 형태 분류 전용 CLI 플래그는 전부 제거했다. 이제 물을 게 없다.

def set_scan_range(v):
    globals()['SCAN_RANGE'] = v
    for fn in (real_scan, has_line_of_sight, can_observe, find_observation_point):
        d = list(fn.__defaults__)
        d[-1] = v
        fn.__defaults__ = tuple(d)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--seed', type=int, default=14)
    ap.add_argument('--seeds', type=int, default=0,
                    help='N>0 이면 seed..seed+N-1 을 헤드리스로 배치 실행')
    ap.add_argument('--headless', action='store_true')
    ap.add_argument('--min-sep', type=int, default=None)
    ap.add_argument('--los-block-confirmed', dest='los', action='store_true',
                    default=None)
    ap.add_argument('--no-los-block-confirmed', dest='los', action='store_false')
    ap.add_argument('--no-logodds', action='store_true')
    ap.add_argument('--no-safe-block', action='store_true')
    ap.add_argument('--turn-cost', type=float, default=None)
    ap.add_argument('--false-hit', type=float, default=None)
    ap.add_argument('--miss', type=float, default=None)
    ap.add_argument('--servo-off', type=float, default=None)
    ap.add_argument('--no-servo-limit', action='store_true')
    ap.add_argument('--no-turn-to-aim', action='store_true')
    ap.add_argument('--scan-range', type=int, default=None)
    ap.add_argument('--stationary-scan', action='store_true')
    ap.add_argument('--robot-speed', type=float, default=None)
    ap.add_argument('--no-confirm', action='store_true')
    ap.add_argument('--no-return', action='store_true')
    ap.add_argument('--no-cone-trim', action='store_true')
    ap.add_argument('--cone-trim-deg', type=float, default=None)
    ap.add_argument('--no-cone-model', action='store_true')
    ap.add_argument('--cone-subrays', type=int, default=None)
    args = ap.parse_args()

    if args.no_confirm:
        globals()['CONFIRM_PHASE'] = False
    if args.no_return:
        globals()['RETURN_HOME'] = False

    if args.scan_range is not None:
        set_scan_range(args.scan_range)
    if args.stationary_scan:
        globals()['CONTINUOUS_SCAN'] = False
    if args.robot_speed is not None:
        globals()['ROBOT_SPEED_MPS'] = args.robot_speed

    if args.no_logodds:
        globals()['USE_LOGODDS'] = False
    if args.no_safe_block:
        globals()['SAFE_BLOCK_ESTIMATED'] = False
    if args.turn_cost is not None:
        globals()['TURN_COST'] = args.turn_cost
    if args.false_hit is not None:
        globals()['SENSOR_FALSE_HIT'] = args.false_hit
    if args.miss is not None:
        globals()['SENSOR_MISS'] = args.miss
    if args.min_sep is not None:
        globals()['OBSTACLE_MIN_SEP'] = args.min_sep
    if args.los is not None:
        globals()['LOS_BLOCK_CONFIRMED'] = args.los
    if args.no_cone_trim:
        globals()['USE_CONE_TRIM'] = False
    if args.cone_trim_deg is not None:
        globals()['CONE_TRIM_DEG'] = args.cone_trim_deg
    if args.no_cone_model:
        globals()['USE_CONE_MODEL'] = False
    if args.cone_subrays is not None:
        globals()['CONE_SUBRAYS'] = args.cone_subrays

    if args.servo_off is not None:
        globals()['SERVO_MAX_OFF'] = args.servo_off
        globals()['CAMERA_FOV'] = 2 * args.servo_off
    if args.no_servo_limit:
        globals()['LIMIT_SCAN_TO_SERVO'] = False
    if args.no_turn_to_aim:
        globals()['TURN_TO_AIM'] = False

    if args.seeds > 0:
        results = []
        for s in range(args.seed, args.seed + args.seeds):
            try:
                results.append(run(s, visualize=False))
            except KeyboardInterrupt:
                raise
            except Exception as e:                      # noqa: BLE001
                print(f"[seed {s:4d}] 예외: {type(e).__name__}: {e}")
        if results:
            def avg(k):
                vals = [r[k] for r in results if not (isinstance(r[k], float)
                                                      and math.isnan(r[k]))]
                return sum(vals) / len(vals) if vals else float('nan')
            print("-" * 100)
            print(f"{len(results)}개 시드 요약 (볼록껍질 방식, MIN_SEP={OBSTACLE_MIN_SEP}, "
                  f"LOS_BLOCK={LOS_BLOCK_CONFIRMED}, LOGODDS={USE_LOGODDS}, "
                  f"SAFE_BLOCK={SAFE_BLOCK_ESTIMATED}, TURN_COST={TURN_COST}, "
                  f"오측률={SENSOR_FALSE_HIT})")
            print(f"  서보제약={LIMIT_SCAN_TO_SERVO} (±{SERVO_MAX_OFF}도, "
                  f"FOV {CAMERA_FOV}도), 조준회전={TURN_TO_AIM}")
            print(f"  원뿔모델={USE_CONE_MODEL} ({CONE_SUBRAYS}표본) "
                  f"| 원뿔절단={USE_CONE_TRIM} ({CONE_TRIM_DEG}도, "
                  f"연속스윕={CONTINUOUS_SCAN})")
            print(f"  스텝 {avg('steps'):.1f} | 충돌 {avg('collisions'):.2f} "
                  f"(발생 시드 {sum(1 for r in results if r['collisions']):d}개) "
                  f"| 잔여 미탐색 {avg('unknown_left'):.1f} | 핑크 {avg('pink'):.1f}")
            print(f"  회전 {avg('turns'):.1f}회 / {avg('turn_deg'):.0f}도 "
                  f"(조준 {avg('aim_turns'):.1f}회 / {avg('aim_turn_deg'):.0f}도, "
                  f"스캔포기 {avg('aim_blocked'):.1f}) "
                  f"| 유령셀 {avg('phantom'):.1f}개")
            print(f"  클러스터 {avg('clusters'):.1f}/{avg('obstacles'):.0f} "
                  f"| 확정 {avg('confirmed'):.1f} "
                  f"| 병합 {avg('merged'):.2f} | 놓침 {avg('missed'):.2f}")
            print(f"  IoU {avg('iou'):.3f} | 중심오차 {avg('center_err'):.2f}칸")
        return

    try:
        run(args.seed, visualize=not args.headless)
    except KeyboardInterrupt:
        print("\n중단됨")


if __name__ == '__main__':
    main()
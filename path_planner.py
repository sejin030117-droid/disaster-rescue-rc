#	격자 A* 경로계획 핵심 — 16방향 이동, 안전맵/여유공간맵, 프런티어 탐색
import heapq
import math
from collections import OrderedDict

import numpy as np

GRID_SIZE = 60
CELL_SIZE = 0.05
ROBOT_SIZE_M = 0.25
ROBOT_WIDTH_CELLS = int(ROBOT_SIZE_M / CELL_SIZE)  # 5

# ── compute_clearance_map/compute_safe_map 캐시 ────────────────────────
# ★[성능] 둘 다 grid 내용만 보고 값이 정해지는 순수함수라, 같은 grid 가
# 다시 들어오면 항상 같은 결과다(안전하게 캐시 가능). 실측상 이 둘이
# find_path() 전체 시간의 사실상 전부(맵 계산이 A* 탐색보다 훨씬 무겁다)
# 를 차지하는데, 정확히 같은 grid 로 반복 호출되는 경우가 흔하다:
#   - rescue_planner.plan_rescue_paths() 는 안전/위험 두 grid 를 만드는데,
#     아직 가스 실측이 임계값을 넘은 적이 없으면 두 grid 가 바이트 단위로
#     완전히 같다.
#   - path_planner_sim2.place_rescuee_truths() 는 배치 시도마다(최대 500회
#     x 인원수) 도달 가능성을 find_path 로 확인하는데, 그 안에서 쓰는
#     test_grid 는 루프 내내 단 한 번도 안 바뀐다 - 즉 첫 호출만 계산하고
#     나머지 수백 번은 전부 캐시로 공짜다.
#   - 미션 후반(탐사 끝, "return"/"done" 단계)에는 grid 가 더 안 바뀌는데
#     구조경로는 RESCUE_REPLAN_EVERY 마다 계속 재계산을 시도하므로, 이
#     구간에서 캐시 적중률이 특히 높다.
# 캐시 키가 grid 내용 자체(바이트)라 잘못된 결과가 나올 여지가 없다 -
# 크기만 작게 유지하면(오래 쓰던 grid 는 밀려나게) 메모리도 안전하다.
_MAP_CACHE_SIZE = 16
_clearance_cache: "OrderedDict" = OrderedDict()
_safe_cache: "OrderedDict" = OrderedDict()


def _cache_lookup(cache, key):
    if key is None or key not in cache:
        return None
    cache.move_to_end(key)
    return cache[key]


def _cache_store(cache, key, value):
    if key is None:
        return
    cache[key] = value
    cache.move_to_end(key)
    if len(cache) > _MAP_CACHE_SIZE:
        cache.popitem(last=False)


def _grid_cache_key(grid, *extra):
    """grid.tobytes() 를 키로 쓴다 - numpy 배열이 아니면(예: 순수 파이썬
    리스트를 넘기는 호출부) 캐시를 그냥 안 쓴다(None 반환 -> 매번 재계산,
    이전과 동일하게 항상 정답이지만 느림). 크래시를 내느니 캐시 없이
    도는 쪽을 택한다."""
    try:
        return (grid.tobytes(), *extra)
    except AttributeError:
        return None


def is_safe_cell(grid, x, y, margin=None):
    if margin is None:
        margin = ROBOT_WIDTH_CELLS // 2
    for dx in range(-margin, margin + 1):
        for dy in range(-margin, margin + 1):
            nx, ny = x + dx, y + dy
            if not (0 <= nx < GRID_SIZE and 0 <= ny < GRID_SIZE):
                return False, f"map out ({nx},{ny})"
            if grid[ny][nx] == 3:
                return False, f"obstacle ({nx},{ny})"
    return True, None


# 4방향(상하좌우) + 4방향(대각). 대각은 실제 거리가 sqrt(2) 이므로 비용을 따로 준다.
STRAIGHT_DIRS = [(-1, 0), (1, 0), (0, -1), (0, 1)]
DIAGONAL_DIRS = [(-1, -1), (-1, 1), (1, -1), (1, 1)]
# 16방향용 추가 오프셋: (2,1) 계열 -> 26.57도, 63.43도
KNIGHT_DIRS = [(2, 1), (2, -1), (-2, 1), (-2, -1),
               (1, 2), (-1, 2), (1, -2), (-1, -2)]
DIAG_COST = 1.41421356

# 16방향 이동 목록 (좌표오프셋, 비용). 1칸 이동이 그대로 남아있어
# 좁은 곳에서는 A*가 알아서 1칸 이동을 고른다.
MOVES_16 = ([((dx, dy), 1.0) for dx, dy in STRAIGHT_DIRS] +
            [((dx, dy), DIAG_COST) for dx, dy in DIAGONAL_DIRS] +
            [((dx, dy), math.hypot(dx, dy)) for dx, dy in KNIGHT_DIRS])

# 경유 칸 검사 간격(칸). 2칸 점프는 중간 칸을 지나가므로 촘촘히 봐야 한다.
SEG_STEP = 0.3

# 안전 여유 비용: A*는 '안전 판정만 통과하면' 최단거리를 고르므로 벽에 바짝 붙는다.
# 실행 오차(회전/직진)가 조금만 있어도 마진을 침범하므로, 여유가 적은 칸에 벌점을 준다.
CLEARANCE_WANT = 4      # 이 정도 여유(칸)는 확보하고 싶다
CLEARANCE_COST = 1.2    # 여유 1칸 부족할 때마다 더해지는 비용


def compute_clearance_map(grid, want=CLEARANCE_WANT):
    """모든 칸의 '장애물까지 거리'를 한 번에 계산(다중소스 BFS).
    이웃마다 clearance_of 를 부르면 노드당 16회 x 9x9 스캔이라 매우 느리다.
    경로탐색 시작 시 한 번만 만들어 재사용한다.

    ★[성능] grid 내용이 이전 호출과 똑같으면 캐시에서 즉시 반환한다 -
    파일 상단 캐시 설명 참고."""
    key = _grid_cache_key(grid, want)
    cached = _cache_lookup(_clearance_cache, key)
    if cached is not None:
        return cached

    from collections import deque
    cm = [[want] * GRID_SIZE for _ in range(GRID_SIZE)]
    dq = deque()
    for y in range(GRID_SIZE):
        for x in range(GRID_SIZE):
            if grid[y][x] == 3:
                cm[y][x] = 0
                dq.append((x, y))
    while dq:
        x, y = dq.popleft()
        d = cm[y][x]
        if d >= want:
            continue
        for dx, dy in STRAIGHT_DIRS + DIAGONAL_DIRS:
            nx, ny = x + dx, y + dy
            if 0 <= nx < GRID_SIZE and 0 <= ny < GRID_SIZE and cm[ny][nx] > d + 1:
                cm[ny][nx] = d + 1
                dq.append((nx, ny))
    _cache_store(_clearance_cache, key, cm)
    return cm


def clearance_of(grid, x, y, want=CLEARANCE_WANT):
    """(x,y)가 장애물(3)에서 몇 칸 떨어졌는지. want 이상이면 want로 자름(빠르게)."""
    for r in range(1, want + 1):
        for dx in range(-r, r + 1):
            for dy in (-r, r):
                nx, ny = x + dx, y + dy
                if 0 <= nx < GRID_SIZE and 0 <= ny < GRID_SIZE and grid[ny][nx] == 3:
                    return r - 1
        for dy in range(-r + 1, r):
            for dx in (-r, r):
                nx, ny = x + dx, y + dy
                if 0 <= nx < GRID_SIZE and 0 <= ny < GRID_SIZE and grid[ny][nx] == 3:
                    return r - 1
    return want


def get_neighbors(x, y):
    """8방향 인접 좌표. 프런티어 판정 등 '맞닿음'이 필요한 곳에서 쓴다."""
    return [(x + dx, y + dy) for dx, dy in STRAIGHT_DIRS + DIAGONAL_DIRS]


def get_neighbors_cost(x, y):
    """16방향 이동 후보를 (좌표, 비용)으로 반환."""
    return [((x + dx, y + dy), c) for (dx, dy), c in MOVES_16]


def compute_safe_map(grid):
    """모든 칸에 대해 is_safe_cell 을 미리 계산(누적합 사용).
    segment_safe 가 매번 5x5 스캔을 하면 너무 느려서, 한 번만 만들어 O(1) 조회한다.

    ★[성능] compute_clearance_map 과 동일한 이유로 grid 내용 기준 캐시."""
    key = _grid_cache_key(grid)
    cached = _cache_lookup(_safe_cache, key)
    if cached is not None:
        return cached

    m = ROBOT_WIDTH_CELLS // 2
    # blocked 누적합
    ps = [[0] * (GRID_SIZE + 1) for _ in range(GRID_SIZE + 1)]
    for y in range(GRID_SIZE):
        rs = 0
        for x in range(GRID_SIZE):
            rs += 1 if grid[y][x] == 3 else 0
            ps[y + 1][x + 1] = ps[y][x + 1] + rs
    safe = [[False] * GRID_SIZE for _ in range(GRID_SIZE)]
    for y in range(GRID_SIZE):
        for x in range(GRID_SIZE):
            if x - m < 0 or y - m < 0 or x + m >= GRID_SIZE or y + m >= GRID_SIZE:
                continue                      # 지도 밖으로 몸통이 나감
            x0, y0, x1, y1 = x - m, y - m, x + m, y + m
            tot = ps[y1 + 1][x1 + 1] - ps[y0][x1 + 1] - ps[y1 + 1][x0] + ps[y0][x0]
            safe[y][x] = (tot == 0)
    _cache_store(_safe_cache, key, safe)
    return safe


def segment_safe(grid, x0, y0, x1, y1, step=SEG_STEP, safe_map=None):
    """(x0,y0)->(x1,y1) 직선을 실제로 지나갈 때 로봇 몸통이 계속 안전한지.
    2칸 점프는 웨이포인트가 아닌 '경유 칸'을 통과하므로 반드시 확인해야 한다.
    (예: (10,10)->(12,11) 은 (11,10),(11,11) 을 지나감)
    정수 칸만 보면 벽을 스치는 경로를 통과로 오판하므로 0.3칸 간격으로 표본을 뜬다."""
    dist = math.hypot(x1 - x0, y1 - y0)
    n = max(2, int(dist / step))
    for i in range(n + 1):
        t = i / n
        cx = int(round(x0 + (x1 - x0) * t))
        cy = int(round(y0 + (y1 - y0) * t))
        if not (0 <= cx < GRID_SIZE and 0 <= cy < GRID_SIZE):
            return False
        if safe_map is not None:
            if not safe_map[cy][cx]:
                return False
        elif not is_safe_cell(grid, cx, cy)[0]:
            return False
    return True


def can_move_diagonal(grid, x, y, nx, ny):
    """구버전 호환. segment_safe 가 더 일반적인 검사라 그쪽으로 위임한다."""
    return segment_safe(grid, x, y, nx, ny)


def find_frontiers(grid):
    """빈공간(0) 중 미탐색(-1)과 8방향으로 맞닿은 칸 = 다음 탐사 목표 후보.

    ★성능: 예전엔 60x60 전 칸을 파이썬 이중루프로 훑으며 칸마다 이웃 8개를
      다시 확인했다(실측 376콜/3.4초). 매 스텝 호출되는 함수라 numpy 로
      벡터화했다 - 미탐색 마스크를 8방향으로 한 칸씩 밀어(shift) OR 누적하면
      '미탐색과 인접' 마스크가 한 번에 나온다.

    ★ 반환 순서는 예전과 동일하다(y 오름차순, 같은 y 안에서 x 오름차순).
      np.where 가 row-major 로 반환하는 순서가 원래 이중루프(y 바깥/x 안쪽)와
      같다. 호출부가 min(...) 으로 최소를 고를 때 동점 처리가 순서에 의존하므로
      이게 어긋나면 탐사 경로가 미묘하게 달라진다.

    ★ 경계 밖 이웃은 '미탐색 아님'으로 친다 - 밀어낸 자리를 False 로 채우면
      원래 코드의 범위검사(0 <= nx < GRID_SIZE ...)와 결과가 같다."""
    g = np.asarray(grid)
    unk = (g == -1)
    near_unk = np.zeros_like(unk)
    for dy in (-1, 0, 1):
        for dx in (-1, 0, 1):
            if dx == 0 and dy == 0:
                continue
            sh = np.zeros_like(unk)
            # (dy, dx) 만큼 밀어서 겹치는 부분만 복사 (바깥은 False 유지)
            ys_dst = slice(max(0, dy), GRID_SIZE + min(0, dy))
            xs_dst = slice(max(0, dx), GRID_SIZE + min(0, dx))
            ys_src = slice(max(0, -dy), GRID_SIZE + min(0, -dy))
            xs_src = slice(max(0, -dx), GRID_SIZE + min(0, -dx))
            sh[ys_dst, xs_dst] = unk[ys_src, xs_src]
            near_unk |= sh
    ys, xs = np.where((g == 0) & near_unk)
    return list(zip(xs.tolist(), ys.tolist()))


_MAX_PATH_CACHE_SIZE = 32
_path_cache: "OrderedDict" = OrderedDict()
_PATH_CACHE_MISS = object()   # ★ None 은 "경로 없음"이라는 정상적인 캐시된
                              #   값일 수 있으므로, "캐시에 아예 없음"과
                              #   구분하려면 별도 sentinel 이 필요하다.


def find_path(grid, start, goal):
    """16방향 가중치 A*. 경유 칸까지 segment_safe 로 검사한다.

    ★[성능] 실측 확인 - compute_clearance_map/compute_safe_map 을 캐시해도
    A* 탐색 자체가 여전히 grid 하나당 6~12ms 걸린다(60x60, 16방향 확장 +
    구간마다 segment_safe 샘플링이 노드당 비용이 큼). 그런데 구조경로는
    같은 grid 로 반복 호출되는 경우가 흔하다(가스 실측 전 안전/위험 grid
    가 동일, 미션 후반 grid 가 안 변해도 RESCUE_REPLAN_EVERY 마다 계속
    재계산 시도). find_path 는 (grid, start, goal) 이 같으면 항상 같은
    결과를 내는 순수함수이므로, 탐색 결과 자체를 캐시해서 맵 계산은 물론
    A* 탐색까지 통째로 건너뛴다. 캐시 히트 시에도 반환값은 항상 사본을
    준다 - 호출부가 실수로 내용을 바꾸면 캐시가 오염되는 걸 막기 위해."""
    key = _grid_cache_key(grid, start, goal)
    if key is not None:
        cached = _path_cache.get(key, _PATH_CACHE_MISS)
        if cached is not _PATH_CACHE_MISS:
            _path_cache.move_to_end(key)
            return list(cached) if cached is not None else None

    result = _find_path_uncached(grid, start, goal)
    if key is not None:
        _path_cache[key] = result
        _path_cache.move_to_end(key)
        if len(_path_cache) > _MAX_PATH_CACHE_SIZE:
            _path_cache.popitem(last=False)
    return list(result) if result is not None else None


def _find_path_uncached(grid, start, goal):
    cmap = compute_clearance_map(grid)
    smap = compute_safe_map(grid)
    open_set = [(0.0, start)]
    came_from = {}
    g_score = {start: 0.0}
    closed = set()
    while open_set:
        _, current = heapq.heappop(open_set)
        if current in closed:
            continue
        closed.add(current)
        if current == goal:
            return _reconstruct(came_from, current)
        cx, cy = current
        for (nx, ny), step_cost in get_neighbors_cost(cx, cy):
            if not (0 <= nx < GRID_SIZE and 0 <= ny < GRID_SIZE):
                continue
            if not segment_safe(grid, cx, cy, nx, ny, safe_map=smap):
                continue
            # 여유가 부족한 칸은 벌점 -> 벽에 바짝 붙는 경로를 피한다
            lack = CLEARANCE_WANT - cmap[ny][nx]
            penalty = CLEARANCE_COST * lack if lack > 0 else 0.0
            tentative_g = g_score[current] + step_cost + penalty
            if tentative_g < g_score.get((nx, ny), float('inf')):
                g_score[(nx, ny)] = tentative_g
                dx = abs(nx - goal[0]); dy = abs(ny - goal[1])
                h = math.hypot(dx, dy)          # 16방향이므로 유클리드 휴리스틱
                heapq.heappush(open_set, (tentative_g + h, (nx, ny)))
                came_from[(nx, ny)] = current
    return None


def _reconstruct(came_from, current):
    path = []
    while current in came_from:
        path.append(current)
        current = came_from[current]
    path.reverse()
    return path


def find_reachable_frontier(grid, robot_x, robot_y, frontiers):
    sorted_frontiers = sorted(
        frontiers,
        key=lambda f: abs(f[0] - robot_x) + abs(f[1] - robot_y)
    )
    for (fx, fy) in sorted_frontiers:
        path = find_path(grid, (robot_x, robot_y), (fx, fy))
        if path:
            return (fx, fy), path
    return None, None


def path_to_commands(path, robot_angle):
    commands = []
    prev_angle = robot_angle
    for i in range(len(path) - 1):
        x1, y1 = path[i]
        x2, y2 = path[i + 1]
        dx, dy = x2 - x1, y2 - y1
        target_angle = math.degrees(math.atan2(dy, dx))
        angle_diff = target_angle - prev_angle
        angle_diff = (angle_diff + 180) % 360 - 180
        if abs(angle_diff) > 5:
            direction = "turn_left" if angle_diff > 0 else "turn_right"
            commands.append({"cmd": direction, "speed": 100, "duration_ms": int(abs(angle_diff) * 10)})
        commands.append({"cmd": "forward", "speed": 100, "duration_ms": 200})
        prev_angle = target_angle
    return commands
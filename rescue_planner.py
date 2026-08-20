"""
rescue_planner.py -- 요구조자 구출을 위한 해저드 인식 경로 탐색 (2경로)

★ 요구조자 '위치'는 이 모듈이 정하지 않는다.
  예전엔 여기서 탐사된 자유공간 중 무작위로 배치했는데, 이제 요구조자(들)는
  hidden_obstacles 처럼 path_planner_sim2.py 의 정답(rescuee_truths)으로
  시뮬레이션 시작 시점에 미리 정해지고, 로봇이 실제로 발견(가시선+사거리)
  했을 때만 (한 명씩) 이 모듈로 넘어온다 - path_planner_sim2.place_rescuee_truths()
  참고. 이 파일은 "이미 발견된 (한) 위치까지 어떻게 갈까"만 책임진다 -
  요구조자가 여러 명이면 호출하는 쪽(sim_bridge.py)이 발견된 사람 수만큼
  이 함수를 반복 호출한다.

★ 정답 미참조 원칙은 여전히 지킨다.
  sim_bridge.ScalarField.value_at() 은 "정답"(로봇이 안 가본 칸도 계산해
  낼 수 있는 값)이다. 이 모듈은 그 함수를 쓰지 않는다 - 오직 로봇이 실제로
  밟아서 측정한 칸(measured_t/measured_g 딕셔너리)만 본다. path_planner_sim2.py
  가 truth_id 를 로봇 로직에서 격리하는 것과 완전히 같은 이유다.

  따라서: 로봇이 밟아보지 않은 칸은 위험 여부를 모른다. 이 모듈은 그런 칸을
  '위험 없음'으로 낙관한다(막지 않는다) - 안전 쪽으로 보수적인 가정이
  아니라는 점을 반드시 인지할 것. 실측 커버리지가 좁으면(로봇이 지나간
  궤적 근처만 알고 있으면) 이 가정이 위험할 수 있다. 근본 해법은 탐사
  중 궤적을 넓히는 것이지, 이 모듈이 대신 추정해주는 게 아니다.

[2경로]
  safe  : 고온+가스 칸 모두 회피 - 제일 안전한 경로
  risky : 고온만 회피, 가스 칸은 통과 허용 - 위험도가 약간 있지만
          (가스는 있어도 온도가 그리 높지 않으므로) 가도 상관없는 경로

  둘 다 매번 계산해서 같이 반환한다 - 예전처럼 하나를 골라주는 게 아니라,
  화면에 두 경로를 동시에 보여주고 판단은 보는 사람이 하게 한다.
  각각 None 일 수 있다(그 조건으로는 경로가 아예 없음) - 단 plan_rescue_paths()
  는 둘 다 None 이면 확인불가(4) 구간까지 감수하고 한 번 더 계산해보므로,
  실제로 둘 다 None 이 나오는 건 확정 장애물/화재로 진짜 막힌 경우뿐이다.

  판정 임계값(FIRE_TEMP_C=50, GAS_PPM=40)은 sim_bridge.DashViz._hazards() 의
  가스 위험구역 판정 기준과 동일하게 맞췄다. 화재(불) 표시(지도의 원)도
  같은 50도를 쓴다(sim_bridge.FIRE_CIRCLE_TEMP_C) - 맨몸 인간이 짧게라도
  버티기 애매해지는 지점을 기준으로 잡았다(근거는 sim_bridge.py 주석
  참고). 카드 "위험" 표시 / 지도 화재원 / 실제 경로 회피가 전부 50도에서
  함께 발동한다 - 셋을 다르게 두면 "지도엔 안 뜨는데 왜 이 경로를
  피하지?" 같은 혼란이 생긴다. 값을 바꿀 때는 여기, sim_bridge.py 의
  FIRE_CIRCLE_TEMP_C, dashboard.py 의 온도 카드 danger 임계값 셋을 같이
  바꿀 것(단일 출처가 세 곳에 따로 적혀 있는 상태라 하나만 바꾸면
  어긋난다 - robot_config.py 스타일의 진짜 단일 상수 모듈로 옮기는 게
  더 안전할 수 있다).
"""
import path_planner as P

# ★[버그수정] sim_bridge.py/dashboard.py 는 robot_config 가 없어도(시뮬
# 단독 실행 워크플로우) 안 죽도록 getattr+fallback 을 쓰는데, 이 파일만
# 직접 import 라 ImportError 로 죽었다(CLAUDE.md §11 인계 이슈).
try:
    import robot_config as _RC
except ImportError:
    _RC = None
FIRE_TEMP_C = getattr(_RC, "FIRE_TEMP_C", 50.0)
GAS_PPM = getattr(_RC, "GAS_PPM", 40.0)

# sim_bridge.DashViz._hazards() / FIRE_CIRCLE_TEMP_C 와 동일



def _hazard_blocked_grid(grid, measured_t, measured_g, block_gas, block_temp,
                         start, target, block_unconfirmed=True):
    """grid 사본에서 해저드 칸을 값 3(장애물)으로 덮어써 find_path 가
    피해가게 만든다. path_planner_sim2.planning_grid() 가 추정채움(1)을
    3으로 바꿔 넘기는 것과 같은 패턴 - 새 차단 규칙이 필요할 때마다
    A* 를 새로 짜지 않고 '입력 격자를 바꿔치기'하는 방식을 그대로 따른다."""
    g = grid.copy()
    # ★[버그수정] 확정 장애물(1, 볼록껍질로 확정됐지만 직접관측(3)은
    # 아닌 칸)도 막아야 한다 - 위 docstring 이 "planning_grid() 와 같은
    # 패턴"이라 주장했지만 실제로는 빠져 있었다. find_path 의 안전판정은
    # 값 3만 막힌 것으로 본다(path_planner.compute_safe_map 참고)라서,
    # 이걸 안 하면 구조 경로가 확정된 장애물을 그냥 통과해버릴 수 있었다.
    g[g == 1] = 3
    # ★[버그수정 - 사용자 실기 테스트에서 발견] 완전히 안 가본 칸(-1,
    # 미탐색)도 항상 막는다. 이건 block_unconfirmed 게이트 밖에서, 절대
    # 폴백으로도 안 풀어준다 - "확인불가"(4)는 최소한 관측을 시도라도
    # 해봤지만, -1은 로봇이 그 칸에 대해 아무 정보도 없는 상태다.
    # 여기를 막지 않으면 A*가 "가본 적도 없는 칸"을 뻥 뚫린 빈 공간
    # 취급해서 최단거리로 뚫고 지나가버린다 - 실제로 지나갈 수 있는지
    # 조차 모르는 길을 "최소경로"라고 안내하는 셈이라 잘못됐다.
    g[g == -1] = 3
    if block_unconfirmed:
        # 확인불가(4, 핑크) 칸도 장애물로 취급해서 회피한다 - 로봇이 어떤
        # 위치에서도 시야/경로를 확보 못 해 관측을 포기한 칸이다. 안이
        # 비어 있을 수도 있지만(§ToF 사각지대 - 장애물 사이 좁은 틈은
        # 각도가 안 맞으면 ToF 로 못 뚫어본다) 문이 있는지 없는지 로봇은
        # 알 방법이 없으므로, 기본은 벽으로 보고 안전 쪽으로 우회시킨다.
        g[g == 4] = 3
    if block_temp:
        for (x, y), v in measured_t.items():
            if v > FIRE_TEMP_C:
                g[y][x] = 3
    if block_gas:
        for (x, y), v in measured_g.items():
            if v > GAS_PPM:
                g[y][x] = 3
    # ★[버그수정] start/target 칸 자체는 위 모든 차단 규칙에서 항상 예외.
    # target(요구조자 위치)은 이미 "거기 사람이 있다"고 발견 판정으로
    # 확인된 칸이라 - 나중에 탐사 알고리즘이 그 칸을 확인불가(4)로
    # 재분류하거나, 요구조자가 서 있는 자리 자체의 측정 온도/가스가
    # 임계값을 넘어도(위험 지역 한복판에 있는 상황) - 실제로 장애물일
    # 수는 없다. 이 두 칸까지 막히면 A*가 출발/도착 지점 자체에 발을
    # 못 붙여 안전경로/위험경로 둘 다 "없음"으로 나온다.
    sx, sy = start
    tx, ty = target
    g[sy, sx] = 0
    g[ty, tx] = 0
    return g


def plan_rescue_paths(grid, measured_t, measured_g, start, target):
    """(path_safe, path_risky) 를 반환한다.

    - path_safe  : 고온+가스 모두 회피
    - path_risky : 고온만 회피, 가스는 통과 허용

    둘 다 매번 새로 계산한다 - 탐사가 진행될수록 measured_t/g 커버리지가
    넓어지므로, 호출할 때마다 그 시점까지 알려진 정보로 다시 그려진다
    (sim_bridge.DashViz 가 주기적으로 다시 부른다).

    ★[버그수정] 요구조자를 발견했다는 것 자체가 로봇→사람 사이 실제
    시야가 뚫려 있었다는 증거다(path_planner_sim2._ray_sees_rescuee 가
    진짜 장애물(truth_id) 기준으로 가림을 판정한다) - 즉 완전히 밀폐된
    방에 있을 가능성은 낮다. 그런데도 안전/위험 두 경로가 '둘 다' 없음
    으로 나오는 경우는 실제로 길이 없어서가 아니라, 확인불가(4, ToF가
    미처 못 뚫어본 좁은 틈 등)를 너무 보수적으로 막아버린 부작용인
    경우가 많았다. 그래서 1차는 평소처럼 확인불가를 막고 계산하되,
    '둘 다' 없으면 확인불가를 뚫어보는 걸 허용하고 한 번 더 계산한다 -
    구조가 아예 불가능한 것처럼 보여주는 것보다는, 불확실한 구간을
    감수하고서라도 경로를 보여주는 쪽이 낫다는 판단이다.

    ★단, 완전 미탐색(-1)은 이 폴백에서도 절대 안 풀어준다(사용자 요청) -
    "확인불가"는 최소한 관측을 시도라도 해본 칸이지만 미탐색은 로봇이
    그 칸에 대해 아무것도 모른다. 안 가본 곳을 지나가는 "최소경로"를
    보여주는 건 실제로 지나갈 수 있는지조차 모르는 길을 안내하는
    셈이라, 확인불가처럼 감수하고 뚫어주면 안 된다. 그래서 폴백을
    써도 둘 다 None 이면 - 확정 장애물(1/3)·화재로 막혔거나, 아직 로봇이
    그쪽을 전혀 탐사하지 못해 안전하게 지나갈 수 있는지 자체를 모르는
    경우다. 두 경우 다 "지금은 정말 아는 경로가 없다"가 맞는 답이다."""
    def _compute(block_unconfirmed):
        g_safe = _hazard_blocked_grid(grid, measured_t, measured_g,
                                      block_gas=True, block_temp=True,
                                      start=start, target=target,
                                      block_unconfirmed=block_unconfirmed)
        g_risky = _hazard_blocked_grid(grid, measured_t, measured_g,
                                       block_gas=False, block_temp=True,
                                       start=start, target=target,
                                       block_unconfirmed=block_unconfirmed)
        return P.find_path(g_safe, start, target), P.find_path(g_risky, start, target)

    path_safe, path_risky = _compute(block_unconfirmed=True)
    if path_safe is None and path_risky is None:
        path_safe, path_risky = _compute(block_unconfirmed=False)
    return path_safe, path_risky
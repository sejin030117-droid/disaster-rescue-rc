"""
rescue_planner.py -- 요구조자 구출을 위한 해저드 인식 경로 탐색 (2경로)

★ 요구조자 '위치'는 이 모듈이 정하지 않는다.
  예전엔 여기서 탐사된 자유공간 중 무작위로 배치했는데, 이제 요구조자는
  hidden_obstacles 처럼 path_planner_sim2.py 의 정답(rescuee_truth)으로
  시뮬레이션 시작 시점에 미리 정해지고, 로봇이 실제로 발견(가시선+사거리)
  했을 때만 이 모듈로 넘어온다 - path_planner_sim2.place_rescuee_truth()
  참고. 이 파일은 "이미 발견된 위치까지 어떻게 갈까"만 책임진다.

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
  각각 None 일 수 있다(그 조건으로는 경로가 아예 없음).

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

FIRE_TEMP_C = 50.0      # sim_bridge.DashViz._hazards() / FIRE_CIRCLE_TEMP_C 와 동일
GAS_PPM = 40.0


def _hazard_blocked_grid(grid, measured_t, measured_g, block_gas, block_temp):
    """grid 사본에서 해저드 칸을 값 3(장애물)으로 덮어써 find_path 가
    피해가게 만든다. path_planner_sim2.planning_grid() 가 추정채움(1)을
    3으로 바꿔 넘기는 것과 같은 패턴 - 새 차단 규칙이 필요할 때마다
    A* 를 새로 짜지 않고 '입력 격자를 바꿔치기'하는 방식을 그대로 따른다."""
    g = grid.copy()
    if block_temp:
        for (x, y), v in measured_t.items():
            if v > FIRE_TEMP_C:
                g[y][x] = 3
    if block_gas:
        for (x, y), v in measured_g.items():
            if v > GAS_PPM:
                g[y][x] = 3
    return g


def plan_rescue_paths(grid, measured_t, measured_g, start, target):
    """(path_safe, path_risky) 를 반환한다. 각각 None 일 수 있다.

    - path_safe  : 고온+가스 모두 회피
    - path_risky : 고온만 회피, 가스는 통과 허용

    둘 다 매번 새로 계산한다 - 탐사가 진행될수록 measured_t/g 커버리지가
    넓어지므로, 호출할 때마다 그 시점까지 알려진 정보로 다시 그려진다
    (sim_bridge.DashViz 가 주기적으로 다시 부른다)."""
    g_safe = _hazard_blocked_grid(grid, measured_t, measured_g,
                                  block_gas=True, block_temp=True)
    g_risky = _hazard_blocked_grid(grid, measured_t, measured_g,
                                   block_gas=False, block_temp=True)

    path_safe = P.find_path(g_safe, start, target)
    path_risky = P.find_path(g_risky, start, target)
    return path_safe, path_risky
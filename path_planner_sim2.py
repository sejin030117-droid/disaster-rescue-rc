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

[★수정5 - 서보 가동범위 제약]
  예전에는 조준 스캔이 '미탐색 칸의 절대 방위각'을 그대로 써서, 로봇 뒤쪽도
  공짜로 스캔했다. 서보로는 물리적으로 닿지 않는 방향인데 시뮬은 그냥 봤다.
  그래서 한 스텝에 사실상 앞 135도 + 임의 방향 135도를 보고 있었다.

  이제 scan_toward() 를 통해서만 조준 스캔을 한다. 서보 범위(±SERVO_MAX_OFF)
  밖을 겨냥하려면 몸통을 돌려야 하고, 그 회전이 turns / turn_deg 에 잡힌다.
  개별 광선도 reachable() 로 걸러 범위 밖은 쏘지 않는다.

  A/B:  --no-servo-limit  (예전 동작)  vs  기본값
        --no-turn-to-aim  (회전 못 하는 로봇의 한계)

[★수정9 - ToF 원뿔 각도 절단]
  VL53L0X 는 25도 원뿔로 쏘고 값 하나만 돌려준다. 거리는 맞지만 각도가
  틀린다 - 원뿔 가장자리가 물체에 스치기만 해도 히트가 나므로, 연속 히트
  구간(run)의 양 끝이 실제 경계보다 원뿔 반지름만큼 바깥으로 부푼다.

  테스트 하네스 검증(54조합 x 4방위): 절단 없음 IoU 0.264/반폭오차 +3.28,
  절단 12.5도(원뿔 반각) IoU 0.763/반폭오차 +0.10. 12.5도가 최적으로 확인됨.

  trim_cone() 이 이 보정을 한다. record_hit() 앞에 넣어서 형태 추정 입력만
  거르고, 안전판정에 쓰는 직접관측(값 3, observe_cell)은 건드리지 않는다 -
  원뿔 절단은 '어디가 물체 경계인지 형태 추정에 얼마나 신뢰할지'의 문제지,
  '거기 뭔가 있다는 사실 자체'는 원뿔 끝이 스친 히트도 안전판정에는 유효하다.

  CONTINUOUS_SCAN=True(기본, 연속 스윕) 경로에만 적용했다. sweep_during() 이
  서보 한 다리(leg, 물리 한계 끝~끝)를 자연스러운 절단 단위로 제공하기
  때문이다. CONTINUOUS_SCAN=False(정지 일괄, 레거시 A/B 비교용) 경로는
  실물에 안 올라가는 모드라 예전 동작(절단 없음)을 그대로 유지했다.

  A/B: --no-cone-trim (절단 끄기) / --cone-trim-deg (절단량 변경, 기본 12.5)

[목차]
   1. 설정값 (파라미터)
   2. 격자(grid) 값 체계          -- 문서
   3. 전역 상태
   4. 순수 기하 원시연산           [정답 X]
   5. 각도 통계 (원형 통계)        [정답 X]
   6. 회전각 적합                  [정답 X]
   7. 정답 월드 생성               [정답 O / 시뮬 전용]
   8. 센서 시뮬 -> 관측 기록       [경계]
   9. 관측 조회                    [정답 X]
  10. 클러스터링 (관측 묶기)       [정답 X]
  11. 형태 추정 -> 지도 반영       [정답 X]
  12. 경로 계획 (도달성)           [정답 X]
  13. 가시선 / 탐색 목표 선정      [정답 X]
  14. 충돌 범퍼                    [정답 O / 시뮬 전용]
  15. 모터 명령 (실기 접합부)
  16. 채점                         [정답 O / 시뮬 전용]
  17. 메인 루프
  18. 시각화
  19. 엔트리포인트

[데이터 흐름 한눈에]
  정답 월드(7) --광선--> 센서 시뮬(8) --히트셀--> 클러스터링(10)
      --관측묶음--> 형태 추정(11) --도형--> grid
      grid --> 경로 계획(12) + 목표 선정(13) --> 이동
      이동 --> 충돌 검사(14) --> 다시 센서 시뮬(8)
      (루프 종료 후) grid vs 정답 --> 채점(16)
"""

import argparse
import heapq
import math
import random

import numpy as np

from path_planner import (GRID_SIZE, ROBOT_WIDTH_CELLS, find_frontiers,
                          get_neighbors_cost, segment_safe,
                          compute_clearance_map, compute_safe_map,
                          CLEARANCE_WANT, CLEARANCE_COST)


# =============================================================================
# =============================================================================
#  1. 설정값 (파라미터)
# =============================================================================
# =============================================================================

# -----------------------------------------------------------------------------
# 1-1. 센서 / 스캔 - 실물 하드웨어와 반드시 맞춰야 하는 값들
# -----------------------------------------------------------------------------

SCAN_RANGE = 36          # real_scan / has_line_of_sight 가 공유하는 사거리(칸)
                         #
                         # [확정 근거]
                         #   robot_config.DIST_MAX_MM = 1800 (VL53L0X 유효 상한)
                         #   robot_config.CELL_SIZE   = 0.05m = 50mm
                         #   → 1800 / 50 = 36칸
                         #   튜닝 파라미터가 아니라 하드웨어 사실이다.
                         #
                         # [A/B 실측 - 시드 144~149, 6개]
                         #   25(기존) 스텝 244.0 핑크 10.0 IoU 0.951 확정 8.2
                         #   30       스텝 246.5 핑크  3.7 IoU 0.961 확정 8.2
                         #   36       스텝 248.3 핑크  5.7 IoU 0.968 확정 8.0
                         #   40       스텝 233.5 핑크  9.3 IoU 0.951 확정 7.7
                         #
                         # 두 가지가 드러났다:
                         #  (a) 사거리를 늘려도 스텝이 안 준다. 탐사를 막는 건
                         #      거리가 아니라 가림(occlusion)이다. 미탐색 칸
                         #      대부분은 멀어서가 아니라 장애물 뒤에 있어서
                         #      못 본 것이고, 그건 걸어가 돌아봐야만 풀린다.
                         #      → 전시회 시간 단축의 답은 사거리가 아니라
                         #        스캔 모델(정지 일괄 → 이동 중 연속)이다.
                         #  (b) 40 에서는 오히려 나빠진다. 멀리서 보면 장애물
                         #      주위를 돌지 않고 지나쳐 viewpoint_spread 가
                         #      안 올라가고, 확정 승격과 형태정확이 떨어진다.
                         #      사거리는 길수록 좋은 게 아니다.

# ---- 서보 가동범위 ★수정5 -------------------------------------------------
#
# robot_config.py 의 ESP32_SWEEP_MIN=25 / MAX=155, SERVO_CENTER=90 이므로
# 중심 기준 오프셋으로는 ±65도다. 몸통을 안 돌리면 이게 한계다.
#
# CAMERA_FOV 는 '한 번의 스윕이 덮는 각'이므로 서보 범위의 두 배가 되어야
# 앞뒤가 맞는다. 예전 135 는 서보 범위(±67.5)를 가정한 값이었는데,
# 실제 펌웨어 설정과 5도 어긋나 있었다.
SERVO_MAX_OFF = 65.0     # 중심(헤딩) 기준 좌우 최대 오프셋
CAMERA_FOV = 2 * SERVO_MAX_OFF   # 130. 1회 스윕 커버리지
SCAN_COUNT = 30          # 한 번의 FOV 스캔에서 쏘는 광선 개수

# 서보 범위 밖을 겨냥할 때 어떻게 할지
#   LIMIT_SCAN_TO_SERVO=False : 예전 동작. 어느 방향이든 공짜로 스캔(비현실적)
#   True + TURN_TO_AIM=True   : 몸통을 돌려서 본다. 회전 비용 발생 (실물에 맞음)
#   True + TURN_TO_AIM=False  : 아예 못 본다. 회전 못 하는 로봇의 한계 측정용
LIMIT_SCAN_TO_SERVO = True
TURN_TO_AIM = True

# ---- 연속 스윕 모델 ★수정6 ------------------------------------------------
#
# [무엇이 문제였나]
#   지금까지 시뮬은 "정지 → 65광선 일괄 → 1칸 이동"이었다. 실물 ESP32 는
#   MODE_SWEEP 으로 서보를 계속 왕복시키며 20ms 마다 한 점씩 흘려보낸다.
#   구조가 다르면 시뮬 수치를 믿을 수 없다. 특히 시간이 그렇다.
#
#   정지 일괄: 광선마다 서보 조준 + 안정화 + 샘플링 = 200~900ms
#              → 스텝당 30초, 250스텝이면 2시간. 전시회 불가.
#   연속 스윕: 광선당 20ms. 서보는 이동 중에도 계속 돈다.
#              → 250칸 x 0.33s + 90회전 x 1.2s = 약 3분.
#
# [파생되는 사실]
#   1칸 이동(0.33s) → 광선 16개, 서보 33도 전진
#   90도 회전(1.2s) → 광선 60개, 서보 120도 전진
#   즉 회전은 순수 비용이 아니라 '스캔 시간을 벌어주는' 동작이다.
#   대신 스텝당 광선이 65 → 16 으로 줄어 지도가 천천히 찬다.
CONTINUOUS_SCAN = True

CELL_SIZE_M = 0.05           # robot_config.CELL_SIZE 와 같게 유지
ROBOT_SPEED_MPS = 0.15       # [추정] 실측 필요. 1칸(5cm) = 0.33초
ROBOT_TURN_DPS = 75.0        # [추정] 90도에 1.2초. 팀원 펌웨어 coarse/fine 기준
SERVO_SWEEP_DPS = 100.0      # .ino SWEEP_TICK_MS=20 에 2도씩 → 100도/s
TOF_PERIOD_S = 0.020         # .ino 연속 모드 20ms

# 정지 상태에서 스윕만 할 때의 시간(초).
#  - 첫 스텝: 지도가 비어 있어 프런티어가 없으므로 한 바퀴 돌려 시동을 건다
#  - 이동할 경로가 없는 스텝: 가만히 있으면 지도가 안 변해 라이브락이 된다
FULL_SWEEP_S = None          # 아래에서 SERVO_MAX_OFF 로부터 계산
IDLE_SWEEP_S = 0.30

# 관측지점 양자화(칸). record_hit 이 (rx, ry) 로 중복을 거르는데, 이동 중
# 관측은 포즈가 실수라 모든 관측이 서로 다른 지점이 된다. 그러면
# MAX_VIEWPOINTS_PER_CELL(60)이 금방 차서 정작 반대편 관측이 못 들어온다.
VP_QUANT = 0.5

# 정지 일괄 모드의 광선당 소요 시간(초). 시간 비교를 위한 값이며
# CONTINUOUS_SCAN=False 일 때만 쓴다. aim 200~900ms 의 중앙값.
STATIONARY_RAY_S = 0.35

ROBOT_MARGIN = ROBOT_WIDTH_CELLS // 2      # 2 (몸통 5x5 의 반폭)

# ---- ToF 원뿔 각도 절단 ★수정9 --------------------------------------------
#
# VL53L0X 원뿔이 25도라 원뿔 가장자리가 물체에 스치기만 해도 히트가 나서,
# 히트 구간 양 끝이 실제 경계보다 부풀어 보인다. 그 부푼 만큼 양 끝을
# 잘라내면 반폭 오차가 크게 준다.
#
# [검증 결과 (테스트 하네스, 형태3 x 크기3 x 거리6 = 54조합, 4방위 관측)]
#   절단   반폭오차   IoU
#   0도    +3.28     0.264
#   10도   +0.62     0.586
#   12.5도 +0.10     0.763   <- 채택 (원뿔 반각과 정확히 일치)
#   20도   +0.07     0.710   (절단 과다로 표본 소실 시작)
#   25도   +0.00     0.841*  (*생존 16개만 집계. 36개 중 20개 소실)
#
# TOF_FOV_DEG 는 robot_config.TOF_CONE_DEG(25.0)와 반드시 같은 값으로 유지.
TOF_FOV_DEG = 25.0
CONE_TRIM_DEG = TOF_FOV_DEG / 2.0     # 12.5도
USE_CONE_TRIM = True                  # False 면 절단 없이(예전 동작, A/B용)

# ---- ToF 원뿔 물리 모델  ★수정10 ------------------------------------------
#
# [발견 - 실측 A/B 로 드러남]
#   USE_CONE_TRIM 을 넣고 실제로 돌려보니 IoU 가 오히려 나빠졌다
#   (절단 켬 0.79 vs 끔 0.98). 원인: real_scan 이 '폭 0도 레이저'만
#   시뮬레이션하고 있어서, 애초에 원뿔이 물체를 부풀려 보이게 하는 물리
#   현상 자체가 시뮬에 없었다. trim_cone 은 존재하지도 않는 문제를
#   보정하려다 멀쩡한 경계 데이터만 깎아내고 있었던 것.
#
#   실측 재현 조건: --no-cone-trim 만 켰을 때(원뿔모델 없이) 이 결과가 나왔다.
#
# USE_CONE_MODEL=True 면 real_scan 이 TOF_FOV_DEG 폭 원뿔 안에서
# CONE_SUBRAYS 개의 서브 광선을 쏴 최근접 거리를 취한다. 이게 켜져야
# 원뿔 부풀림이 실제로 발생하고, 그래야 USE_CONE_TRIM 이 보정할 대상이
# 생긴다. 둘을 함께 켜는 게(둘 다 기본 True) 실물에 가장 가까운 조건이다.
USE_CONE_MODEL = True
CONE_SUBRAYS = 9         # 원뿔 안 표본 개수. 지난 테스트 하네스(scan_cone_fov)와 동일

# -----------------------------------------------------------------------------
# 1-2. 탐사 종료 조건 - 무한루프 / 라이브락 방지용 상한들
# -----------------------------------------------------------------------------

MAX_STEPS = 1500         # 안전 상한
STALL_LIMIT = 90         # cleanup 에서 이만큼 진전 없으면 잔여 미탐색 포기
COMMIT_LIMIT = 80        # 한 목표를 이만큼 붙잡고 있으면 포기
ABANDON_EVERY = 8        # cleanup 배치 포기 스캔 주기(스텝)
ABANDON_BUDGET = 150     # 한 번의 배치 포기에서 검사할 최대 셀 수

# -----------------------------------------------------------------------------
# 1-3. 형태 판정 임계값 (실측 튜닝값)
# -----------------------------------------------------------------------------

THICKNESS_MIN = 0.55     # 이보다 얇으면 앞면만 본 것 -> 형태 판정 보류

# --- 형태 분류 규칙 v2  ★원을 삼각형으로 오인하던 문제 수정 ---
#
# [원인] 반지름 2 원은 셀 격자에 그리면 원이 아니라 '마름모'가 된다.
#     ··■··
#     ·■■■·
#     ■■■■■    <- 13칸짜리 마름모. circ=0.785, extent=0.500
#     ·■■■·
#     ··■··
# 그런데 마름모의 면적비(extent)는 삼각형과 똑같이 0.5 다. 이론값:
#     삼각형        circ 0.600 / extent 0.500
#     마름모(작은원) circ 0.785 / extent 0.500   <- extent 로 삼각형과 구분 불가
#     정사각형      circ 0.785 / extent 1.000   <- circ 로 마름모와 구분 불가
#     큰 원(r>=3)   circ 0.94  / extent 0.67
# 즉 두 지표 중 하나만으로는 어떤 쌍이든 반드시 겹친다. 둘을 순서대로 써야 하는데
# 예전 규칙은 순서가 반대였다:
#     예전: circ >= 0.89 이면 원, 아니면 extent 로 삼각형/사각형
#           -> 마름모(circ 0.785)가 원 판정에서 탈락하는 순간 삼각형으로 직행
#     신규: circ < 0.78 이면 삼각형, 아니면 extent 로 원/사각형
#           -> circ 는 삼각형(0.64~0.69)과 나머지(0.78~0.94)를 깨끗이 가른다
#
# [실측] 288표본(원/삼각/사각 x 반폭2,3,4 x 커버리지 60~360도)
#     예전 규칙: 정확도 78.1%, 원 96개 중 41개가 삼각형으로 오판
#     신규 규칙: 정확도 99.0%, 원->삼각형 오판 0건
CIRC_TRI_MAX = 0.78      # [v2 전용] 원형도가 이 미만이면 삼각형
EXTENT_CIRC_MAX = 0.78   # [v2 전용] 면적비가 이 이하면 원 / 초과면 사각형

# --- 형태 분류 규칙 v3  ★수정8: 회전 대응 ---
#
# [v2 가 무너진 지점]
#   v2 는 circ 로 삼각형을 먼저 거른 뒤 extent 로 원/사각형을 갈랐다.
#   정답 도형을 임의 각도로 회전시키자 형태정확이 1.00 -> 0.37 로 떨어졌다.
#   회전한 삼각형의 circ 최소가 0.745 인데 문턱이 0.78 이라 대부분 통과해
#   삼각형 판정 자체가 거의 발동하지 않았다.
#
# [순서를 뒤집는다]
#   최소외접 extent 로 재면 삼각형(0.62~0.80)과 나머지(0.79~0.98)가
#   훨씬 깨끗하게 갈린다. 그다음 원/사각형을 circ 로 나눈다.
#
#   [실측 269표본, 축정렬 133 + 회전 136]
#     v2 (0.78/0.78)  축정렬 0.72 / 회전 0.61
#     v3 (0.78/0.94)  축정렬 0.86 / 회전 0.84
#     삼각형은 축정렬 34/34, 회전 33/36 으로 거의 완벽해진다.
SHAPE_RULE = "v3"        # "v1" / "v2" / "v3"
EXTENT_TRI_MAX_V3 = 0.78  # [v3] 최소외접 면적비가 이 미만이면 삼각형
CIRC_CIRCLE_MIN = 0.94    # [v3] 삼각형이 아닐 때, 원형도가 이 이상이면 원

# 추정 반폭이 이 미만이면 원/사각형 구분을 포기하고 unknown 으로 둔다.
#
# 반폭 2 원(지름 20cm)은 5cm 격자에서 13칸짜리 '마름모'로 래스터화된다.
# 그런데 마름모는 45도 돌아간 정사각형과 기하학적으로 완전히 같다.
# 최소외접 extent 도 1.0 으로 같고, circ 도 원 0.890 / 사각형 0.873 으로 겹친다.
# 임계값 문제가 아니라 격자에 정보가 없는 것이다.
#   [실측] 반폭2 원 21표본 정확도 0.00 (전부 사각형 오판)
#          반폭3 원 0.76 / 반폭4 원 0.90
# 틀린 답을 자신 있게 내느니 모른다고 하는 편이 낫다.
# (삼각형 판정은 extent 로 먼저 하므로 이 규칙의 영향을 받지 않는다)
SHAPE_HOLD_SMALL_ROUND = True
SHAPE_MIN_HALF_FOR_ROUND = 2.5

# ★수정8: 면적비(extent)의 분모를 무엇으로 잡을지.
#   True  : 최소외접사각형. 회전에 불변이다.
#   False : 축정렬 bbox (예전). 물체가 축에 정렬돼 있을 때만 맞다.
# 실물 아레나에 물체를 반듯하게 놓는다는 보장이 없으므로 기본 True.
EXTENT_MIN_AREA = True

# 관측지점 방위각 커버리지가 이 미만이면 형태 판정을 아예 보류한다.
# 한쪽에서만 본 사각형은 모서리가 쐐기처럼 보여 원리적으로 삼각형과 구분되지 않는다.
# thickness 로는 못 걸러진다(모서리 뷰도 bbox 는 정사각형에 가까워 thickness=1.0).
# 실측 커버리지별 정확도: 0~90도 62% / 90~150도 85% / 150~210도 97% / 280도+ 100%
SHAPE_MIN_VP_SPREAD = 120

CIRC_MIN = 0.89          # [v1 전용] 원형도가 이 이상이면 원
EXTENT_TRI_MAX = 0.64    # [v1 전용] 면적비. 삼각형 0.58~0.62 / 사각형 0.66~1.00
ROT_FIT_MAX = 0.92       # 회전 채택 적합도(최적각 넓이 / 0도 넓이)
ENABLE_ROTATION = False  # 숨은 장애물이 전부 축정렬 생성이라 기본 끔.
                         # 실제 아레나에 비스듬히 놓는다면 True.

SHAPE_MIN_OBS = 5        # 형태 추정을 시도할 최소 관측 횟수
SHAPE_MIN_SPREAD = 63    # 형태 추정을 시도할 최소 각도 커버리지(도)
                         # - 광선각 기준 사전필터
CONFIRM_SPREAD = 180     # 이 이상이면 확정(검정), 아니면 미확정(노랑)
                         # [수정2] 이제 광선각이 아니라 '관측지점 방위각' 기준.

# -----------------------------------------------------------------------------
# 1-4. 점유격자 확률 모델 (log-odds)  ★실물 대응 수정 1
# -----------------------------------------------------------------------------
#
# 예전에는 grid 에 0/3 을 하드하게 찍었다. 그런데 real_scan 의 통과 처리가
#   if g[y][x] in (-1, 1, 2): g[y][x] = 0
# 이라 **값 3 을 덮어쓰지 않는다**. 시뮬은 정답이 안 변하니 맞는 동작이지만,
# 실물에서 ToF 오측(반사/원뿔이 모서리 걸침/사거리 초과)이 한 번이라도 나오면
# 그 칸은 영구 유령 장애물이 되고, compute_safe_map 이 값3을 막으므로
# 통행 불가 영역이 계속 누적된다.
#   실측 추정: 스윕 20ms 주기 -> 10분에 3만 판독. 오측률 1% 면
#   유령셀 287개, 로봇 반폭 확장 후 통행불가 면적 84%.
#
# 그래서 '증거를 누적'하는 방식으로 바꾼다. 히트는 +, 통과는 -.
# 오측 1회는 통과 관측 2~3회로 자연히 상쇄된다.
USE_LOGODDS = True       # False 면 예전 하드 값 방식 (A/B 비교용)
L_OCC = 0.85             # 히트 1회가 주는 '있다' 증거
L_FREE = -0.40           # 통과 1회가 주는 '없다' 증거
                         #  |L_OCC| > |L_FREE| 인 이유: 한 번이라도 맞았으면
                         #  뭔가 있을 가능성이 크다(정보량이 비대칭)
L_MIN, L_MAX = -4.0, 4.0  # 포화 상한. 오래된 정보가 한쪽으로 굳는 걸 막는다
L_THRESH = 1.5           # 이 이상이면 장애물(값 3)로 취급. L_OCC 기준 2회 히트

# -----------------------------------------------------------------------------
# 1-5. 센서 오차 모델  ★log-odds 효과를 시뮬에서 검증하기 위해 추가
# -----------------------------------------------------------------------------
#
# 지금까지 시뮬은 정답 격자를 그대로 읽어서 오차가 0 이었다.
# 그래서 log-odds 를 넣어도 효과가 드러나지 않는다 - 상쇄할 오측이 없으니까.
# 실물 ToF 의 두 가지 실패를 흉내내 A/B 비교가 가능하게 한다.
SENSOR_FALSE_HIT = 0.0   # 빈 칸인데 '맞았다'고 보고할 확률 (반사/원뿔 걸침)
SENSOR_MISS = 0.0        # 실제 장애물인데 그냥 통과시킬 확률 (흡수/각도)
                         # 실물 감각: 0.005~0.02 정도. 기본 0 = 예전과 동일

# -----------------------------------------------------------------------------
# 1-6. 경로 안전 정책  ★실물 대응 수정 3
# -----------------------------------------------------------------------------
#
# path_planner.compute_safe_map 은 값 3만 막는다. 즉 추정 채움(1,2) 속으로
# 경로가 그대로 뚫린다. 시뮬에서는 bump_cells 가 범퍼로 잡아주지만
# 실물에서는 진짜로 박는다.
SAFE_BLOCK_ESTIMATED = True    # 확정 추정(1)도 통행 불가로
SAFE_BLOCK_UNCONFIRMED = False # 미확정(2)까지 막을지. 추정 도형이 과대하게
                               # 그려지면 갇힐 수 있어 기본 끔

# -----------------------------------------------------------------------------
# 1-7. 회전 비용  ★실물 대응 수정 4
# -----------------------------------------------------------------------------
#
# 예전 비용 = 거리 + 여유공간 페널티. 회전 비용이 0 이었다.
# 그런데 실물에서 회전은 (a) 가장 오래 걸리고 (b) 오도메트리 오차의 주범이며
# (c) 제자리 회전 전류 스파이크로 BMS/BTS7960 래치를 유발한 이력이 있다.
# 16방향 이동은 경로가 지그재그로 나오기 쉬워 회전이 잦다.
#
# [근사임을 명시] 엄밀하게는 상태에 방향을 포함해야((x,y,dir) 3차원 다익스트라)
# 최적해가 보장된다. 그러면 상태가 16배(3600 -> 57,600)로 늘어 매 스텝
# 재계획하는 지금 구조에서는 너무 느리다. 여기서는 '부모에 기록된 진입 방향'만
# 보는 근사를 쓴다. 최적은 아니지만 회전 수를 크게 줄여준다.
#
# ★수정5 이후로는 스캔을 위한 회전(scan_toward)도 turns 에 잡힌다.
#   이동 경로 회전과 조준 회전이 합산되므로, 예전 수치와 직접 비교하면 안 된다.
TURN_COST = 1.2          # 90도 회전 = 직진 1.2칸에 해당하는 비용. 0 이면 예전 동작

# -----------------------------------------------------------------------------
# 1-8. 가시선(LoS) 정책
# -----------------------------------------------------------------------------

# [수정4] 가시선(has_line_of_sight)이 확정 장애물(1)을 막을지.
#   False: 값 3(직접관측)만 막는다. 3은 광선이 실제로 맞힌 표면 칸뿐이라
#          드문드문해서, 광선이 그 사이를 빠져나가 장애물 뒤 미탐색 칸을
#          '볼 수 있다'고 오판한다 -> 헛걸음 후에야 핑크 처리(시간 낭비).
#   True : 확정(1)도 막는다. 헛걸음은 줄지만, 추정 도형이 실제보다 크게
#          그려진 경우 비어있는 칸을 조기에 핑크로 확정해버린다.
#          unreachable_unknowns 는 한번 들어가면 안 빠지므로 이 손실은 영구적이다.
#   미확정(2)은 어느 쪽이든 투과시킨다 - 신뢰할 근거가 없다.
LOS_BLOCK_CONFIRMED = True

# -----------------------------------------------------------------------------
# 1-9. 클러스터링 (관측 -> 장애물 연결) 및 월드 생성 난이도
# -----------------------------------------------------------------------------

# 예전엔 real_scan 이 정답 격자를 보고 '이 히트는 몇 번 장애물'인지 알려줬다.
# 실기에는 그런 정보가 없다. 이제 히트 셀을 거리 기반으로 직접 묶는다.
CLUSTER_GAP = 2          # 체비셰프 거리 이 이하의 히트 셀은 같은 물체로 본다
CLUSTER_EVERY = 4        # 클러스터 재구성 주기(스텝)
MAX_VIEWPOINTS_PER_CELL = 60   # 한 히트셀당 저장할 관측지점 상한

# 장애물 사이 최소 간격(칸). 0 이면 예전처럼 서로 겹쳐서 배치된다.
# 붙어 있는 두 물체는 ToF 로는 원리적으로 분리할 수 없으므로, 0 으로 두면
# 클러스터링이 '실패'하는 게 아니라 애초에 정답이 하나인 상황을 테스트하게 된다.
# 실측: 간격 0 일 때 시드당 평균 5.6쌍이 물리적으로 인접했다.
OBSTACLE_MIN_SEP = 4
OBSTACLE_COUNT = 12

# -----------------------------------------------------------------------------
# 1-10. 시각화 (표시 전용 - 시뮬 로직과 무관)
# -----------------------------------------------------------------------------

# 정답 지도를 벡터 도형(매끈)으로 그릴지.
TRUTH_SMOOTH = True
# 매끈한 도형 아래에 '실제 ToF 가 맞히는 칸'(래스터)을 연한 회색으로 깔지.
TRUTH_SHOW_RASTER = True

# -----------------------------------------------------------------------------
# 1-11. 확정 순회 / 복귀 단계  ★수정7
# -----------------------------------------------------------------------------
#
# [왜 필요한가]
#   연속 스윕으로 바꾸고 나서 확정 승격이 12개 중 7~8개에 그친다.
#   로봇이 미탐색만 쫓아가느라 장애물 '주위를 도는' 일이 없어서, 관측지점
#   방위각 커버리지(viewpoint_spread)가 CONFIRM_SPREAD(180도)를 못 넘는다.
#   미탐색이 0 이 되었다고 임무가 끝난 게 아니다 - 지도는 다 그렸지만
#   그 지도 속 물체가 뭔지 절반은 확신이 없는 상태다.
#
#   그래서 탐사/정리가 끝난 뒤 '아직 확신 없는 클러스터'를 골라, 아직 안 본
#   방위로 찾아가 한 번 더 보는 단계를 넣는다.
CONFIRM_PHASE = True
CONFIRM_MAX_VISITS = 2       # 클러스터 하나당 재방문 시도 상한
CONFIRM_STEP_BUDGET = 300    # 확정 단계 전체 스텝 예산(무한루프 방지)
CONFIRM_STANDOFF = 8         # 클러스터 중심에서 이 정도 떨어진 곳을 노린다.
                             # 너무 붙으면 원뿔이 물체를 못 벗어나 좌우 끝을
                             # 못 보고, 너무 멀면 표면 표본이 성겨진다.
CONFIRM_BEARING_PENALTY = 0.05   # 원하는 방위에서 벗어난 만큼 비용 가산(도당)

# 임무 종료 후 시작 지점으로 복귀. 실물에서는 회수 지점이 곧 투입 지점이다.
RETURN_HOME = True


# =============================================================================
# =============================================================================
#  2. 격자(grid) 값 체계  -- 문서 (코드 없음)
# =============================================================================
# =============================================================================
#
#   -1 미탐색(회색)          : 아직 아무 정보 없음
#    0 빈공간(흰)            : 광선이 통과했다 = 직접 관측한 빈 곳
#    1 추정채움-확정(검정)    : 형태 추정 결과 + 여러 방향에서 확인됨
#    2 추정채움-미확정(노랑)  : 형태 추정 결과지만 한쪽에서만 봄
#    3 직접관측(주황)         : 광선이 실제로 맞은 표면 칸. 경로를 막는다.
#    4 확인불가(핑크)         : 관측 시도했으나 실패해 포기
#
# 우선순위 규칙: 관측이 추정을 이긴다.
#   real_scan 에서 광선이 통과하면 추정 채움(1,2)도 0 으로 덮어쓴다.
#
# 경로 안전판정(compute_safe_map)은 오직 3만 막는다.
# 즉 아직 못 본 장애물 속으로는 걸어 들어갈 수 있고, 시뮬에서는 bump_cells 가
# 범퍼 역할로 그걸 충돌로 잡는다(의도된 동작 - 섹션 14 참고). 실기 전환 시에는
# 최소한 값 1(확정)까지 막도록 compute_safe_map 을 손봐야 진짜로 박지 않는다.
#
# 값 4 는 되돌아가지 않는다: 모든 되돌림 코드가 `in (1, 2)` 조건이라
# 한번 핑크가 된 칸은 영구적이다. (섹션 13 find_nearest_unknown 주석 참고)


# =============================================================================
# =============================================================================
#  3. 전역 상태 (reset_world 로 초기화)
# =============================================================================
# =============================================================================

# ---- [정답 O] 시뮬만 아는 것 --------------------------------------------------
truth_id = None            # (y,x) -> 장애물 인덱스, 없으면 -1
obstacle_ids = []          # 인덱스 -> (ox, oy)
hidden_obstacles = {}      # (ox, oy) -> {"size", "shape"}

# ---- [정답 X] 로봇이 만들어가는 것 --------------------------------------------
grid = None                # 로봇의 지도. 값 체계는 섹션 2 참고
logodds = None             # 직접 관측 증거 누적(log-odds). grid 의 0/3 은 여기서 파생
sensor_rng = None          # 센서 오차 전용 난수(월드 생성 rng 와 분리해 재현성 유지)

turns = 0                  # 회전 명령 횟수      (이동 + 조준 합산)
turn_deg = 0.0             # 누적 회전각(도)
aim_turns = 0              # ★수정5: 그중 '조준을 위한 회전'만 따로 센다
aim_turn_deg = 0.0
aim_blocked = 0            # ★수정5: 서보 범위 밖이라 포기한 스캔 횟수

# ---- 연속 스윕 상태 ★수정6 ----
# 서보는 스텝 경계를 모른다. 계속 왕복하므로 상태를 지속시켜야 한다.
# '어떤 각도가 샘플링되는가'가 '서보가 마침 어디 있었는가'에 달리게 되는데,
# 그게 실물의 진짜 특성이다.
servo_off = 0.0            # 현재 서보 오프셋각(헤딩 기준)
servo_dir = 1               # 스윕 진행 방향 (+1 / -1)
sim_time = 0.0              # 시뮬 내 경과 시간(초)
rays_fired = 0              # 쏜 광선 총 개수
moves = 0                   # 실제로 이동한 칸 수

# ---- 원뿔 절단 스윕 버퍼 ★수정9 ----
# 서보 한 다리(leg, 물리 한계 ~ 물리 한계)를 절단 단위로 모은다.
# _leg_start_is_limit 는 '지금 쌓고 있는 다리가 서보 물리 한계에서 시작했는가'
# - 시뮬레이션 맨 첫 다리(중앙 0도에서 출발)만 False, 그 뒤로는 항상 True.
_sweep_buffer = []
_leg_start_is_limit = False

# ---- 단계 / 복귀 상태 ★수정7 ----
# phase 는 뷰가 읽을 수 있게 전역으로 둔다. _Viz.progress 의 인자를 늘리면
# matplotlib 판과 pygame 판 시그니처가 갈라지므로 그렇게 하지 않았다.
phase = "explore"          # explore / cleanup / confirm / return / done
home = (0, 0)               # 시작(=회수) 지점
return_path = []            # 복귀 경로. 그리기 전용
confirm_cid = None          # 지금 확인하러 가는 클러스터
confirm_done = 0            # 확정 단계에서 재관측을 마친 클러스터 수
confirm_gained = 0          # 그중 실제로 확정(값 1)으로 올라간 수
returned = False            # 복귀 성공 여부

robot_x = robot_y = 0
robot_angle = 0.0
step = 0
collisions = 0

hit_obs = {}                # (x,y) 히트셀 -> {(rx,ry): (angle, dist)}
cluster_cells = {}          # cid -> set(히트셀)
_next_cid = 0
low_confidence_obstacles = set()
unreachable_unknowns = set()
last_estimated_shape = {}   # cid -> "rect"/"circle"/"triangle"/"unknown"
obstacle_render_info = {}   # cid -> (shape, cx, cy, half, rot_deg, value, draw_w, draw_h)
last_filled = {}            # cid -> 이 클러스터가 현재 칠하고 있는 셀 목록
last_obs_count = {}         # cid -> 마지막 형태갱신 시점의 관측 수
cell_owner = {}              # (x,y) -> cid  ([버그3] 겹침 시 남의 칸 침범 방지)


# =============================================================================
# =============================================================================
#  4. 순수 기하 원시연산                                            [정답 X]
# =============================================================================
# =============================================================================
#
# 전역 상태도 안 보고 정답도 안 본다. 입력만 받아 계산하는 함수들.
# 여기 있는 함수는 정답 월드 생성(7)과 형태 추정(11) 양쪽에서 공용으로 쓴다.

# -----------------------------------------------------------------------------
# 4-1. 점 포함 판정
# -----------------------------------------------------------------------------

def point_in_triangle(px, py, x1, y1, x2, y2, x3, y3):
    """세 변에 대한 외적 부호가 모두 같으면 내부."""
    def sign(ax, ay, bx, by, cx, cy):
        return (ax - cx) * (by - cy) - (bx - cx) * (ay - cy)
    d1 = sign(px, py, x1, y1, x2, y2)
    d2 = sign(px, py, x2, y2, x3, y3)
    d3 = sign(px, py, x3, y3, x1, y1)
    has_neg = (d1 < 0) or (d2 < 0) or (d3 < 0)
    has_pos = (d1 > 0) or (d2 > 0) or (d3 > 0)
    return not (has_neg and has_pos)


# -----------------------------------------------------------------------------
# 4-2. 볼록껍질 / 다각형 측정
#      형태 추정(11)의 원형도(circularity)와 면적비(extent) 계산에 쓰인다.
# -----------------------------------------------------------------------------

def convex_hull(points):
    """모노톤 체인. 관측점들을 감싸는 최소 볼록다각형."""
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


def poly_area(h):
    """신발끈 공식."""
    n = len(h)
    a = 0.0
    for i in range(n):
        x1, y1 = h[i]
        x2, y2 = h[(i + 1) % n]
        a += x1 * y2 - x2 * y1
    return abs(a) / 2


def poly_perimeter(h):
    n = len(h)
    p = 0.0
    for i in range(n):
        x1, y1 = h[i]
        x2, y2 = h[(i + 1) % n]
        p += math.hypot(x2 - x1, y2 - y1)
    return p


# -----------------------------------------------------------------------------
# 4-3. 다각형 단순화 (Douglas-Peucker)
#      삼각형 회전각 추정(6)에서 껍질을 3개 꼭짓점으로 줄일 때만 쓴다.
# -----------------------------------------------------------------------------

def dp_simplify(poly, eps=1.4):
    def perp(p, a, b):
        if a == b:
            return math.hypot(p[0] - a[0], p[1] - a[1])
        num = abs((b[0] - a[0]) * (a[1] - p[1]) - (a[0] - p[0]) * (b[1] - a[1]))
        return num / math.hypot(b[0] - a[0], b[1] - a[1])

    def dp(pts):
        if len(pts) < 3:
            return pts
        dmax, idx = 0, 0
        for i in range(1, len(pts) - 1):
            d = perp(pts[i], pts[0], pts[-1])
            if d > dmax:
                dmax, idx = d, i
        if dmax > eps:
            return dp(pts[:idx + 1])[:-1] + dp(pts[idx:])
        return [pts[0], pts[-1]]
    if len(poly) < 3:
        return poly
    xs = [p[0] for p in poly]
    i0, i1 = xs.index(min(xs)), xs.index(max(xs))
    if i0 > i1:
        i0, i1 = i1, i0
    return dp(poly[i0:i1 + 1])[:-1] + dp(poly[i1:] + poly[:i0 + 1])[:-1]


# =============================================================================
# =============================================================================
#  5. 각도 통계 (원형 통계)                                         [정답 X]
# =============================================================================
# =============================================================================
#
# '얼마나 여러 방향에서 봤는가'를 재는 두 함수. 둘 다 circular_spread 를 쓰지만
# **무엇의 각도를 넣는지**가 다르고, 그 차이가 [수정2] 의 핵심이다.
#
#   circular_spread([광선각])        -> 값싼 사전 필터 (SHAPE_MIN_SPREAD, 63도)
#   viewpoint_spread(obs, cx, cy)   -> 확정 판정      (CONFIRM_SPREAD, 180도)

def circular_spread(angles):
    """각도 집합이 실제로 덮는 호의 크기(도).

    [버그2 수정] 예전 max - min 은 ±180 경계에서 완전히 깨진다.
    170도와 -170도는 실제 20도 차이인데 340 으로 계산돼, 한쪽 면만 본
    장애물이 CONFIRM_SPREAD(180)를 넘겨 '확정(검정)'으로 승격됐다.
    정렬 후 최대 빈 구간(wrap 포함)을 360에서 빼면 올바른 커버리지가 된다."""
    if len(angles) < 2:
        return 0.0
    a = sorted(x % 360.0 for x in angles)
    max_gap = 360.0 - (a[-1] - a[0])       # wrap-around 구간
    for i in range(len(a) - 1):
        max_gap = max(max_gap, a[i + 1] - a[i])
    return 360.0 - max_gap


def viewpoint_gap_bearing(obs, cx, cy):
    """관측지점 방위각에서 '가장 크게 비어 있는 구간'의 한가운데 방위(도). ★수정7

    viewpoint_spread 가 커버리지의 '크기'를 재는 함수라면, 이건 그 커버리지의
    구멍이 '어디'인지를 알려준다. 확정 순회에서 어느 방향으로 찾아가야
    새 정보를 얻는지 정하는 데 쓴다.

    관측지점이 하나뿐이면 정반대편이 답이다(그쪽을 전혀 못 봤으므로)."""
    vps = {(rx, ry) for _, _, rx, ry in obs}
    if not vps:
        return 0.0
    angs = sorted(math.degrees(math.atan2(ry - cy, rx - cx)) % 360.0
                  for rx, ry in vps)
    if len(angs) == 1:
        return (angs[0] + 180.0) % 360.0
    # wrap 구간부터 후보로 잡는다 (circular_spread 와 같은 방식)
    best_gap = 360.0 - (angs[-1] - angs[0])
    best_mid = (angs[-1] + best_gap / 2.0) % 360.0
    for i in range(len(angs) - 1):
        g = angs[i + 1] - angs[i]
        if g > best_gap:
            best_gap = g
            best_mid = (angs[i] + g / 2.0) % 360.0
    return best_mid


def needs_confirm(cid):
    """이 클러스터가 아직 확신이 없는가.  ★수정7

    두 경우다:
      - 형태 판정 자체를 보류(unknown): 앞면만 봐서 두께가 안 나옴
      - 형태는 정했지만 미확정(값 2): 관측지점 커버리지가 180도 미만
    둘 다 '반대편에서 한 번 더 보면' 풀릴 가능성이 있는 상태다."""
    ri = obstacle_render_info.get(cid)
    if ri is None:
        return False
    shape, _, _, _, _, value, _, _ = ri
    return (value != 1) or (shape == "unknown")


def viewpoint_spread(obs, cx, cy):
    """추정 중심에서 본 '관측 지점들'의 방위각 커버리지(도).

    [수정2] 확정(검정) 판정에는 광선각 스프레드를 쓰면 안 된다.
    record_hit 이 저장하는 angle_deg 는 스캔 광선의 절대 각도인데, 큰 장애물을
    가까이서 보면 관측 지점 하나에서도 좌우 끝 광선각이 크게 벌어진다.
    (반폭 4칸을 5칸 거리에서 보면 atan(4/5)=38.7도, 한 지점에서 약 77도 스팬)
    그래서 같은 쪽에 몰린 관측 지점 세 개만으로도 합쳐서 180도를 넘겨,
    뒷면을 한 번도 못 봤는데 확정으로 승격돼버렸다.

    '여러 방향에서 봤다'를 재려면 재야 하는 건 로봇이 어디에 서 있었는지다.
    같은 지점의 중복 관측은 한 번으로 센다."""
    vps = {(rx, ry) for _, _, rx, ry in obs}
    if len(vps) < 2:
        return 0.0
    return circular_spread([math.degrees(math.atan2(ry - cy, rx - cx))
                            for rx, ry in vps])


# =============================================================================
# =============================================================================
#  6. 회전각 적합 (ENABLE_ROTATION=True 일 때만 호출)               [정답 X]
# =============================================================================
# =============================================================================
#
# 지금은 ENABLE_ROTATION=False 라 이 섹션 전체가 죽어 있다.
# 시뮬 장애물이 전부 축정렬로 생성되므로 회전을 추정해봐야 이득이 없고,
# 사각형을 마름모로 오판할 위험만 있다. 실제 아레나에 비스듬히 놓는다면 켠다.

def _bbox_area_at(hull, ang_deg):
    """껍질을 -ang_deg 만큼 회전시킨 좌표계에서의 축정렬 bbox 면적."""
    c, s = math.cos(math.radians(-ang_deg)), math.sin(math.radians(-ang_deg))
    xs = [p[0] * c - p[1] * s for p in hull]
    ys = [p[0] * s + p[1] * c for p in hull]
    return (max(xs) - min(xs)) * (max(ys) - min(ys))


def min_area_rect_angle(hull):
    """최소외접사각형 각도(도)와 적합도 비율(최적각 넓이 / 0도 넓이).
    비율이 1에 가까우면 회전할 근거가 없다는 뜻(껍질이 이미 축정렬).
    ENABLE_ROTATION=False 인 동안에는 호출되지 않는다."""
    if len(hull) < 3:
        return 0.0, 1.0
    n = len(hull)
    best_area, best_ang = float('inf'), 0.0
    for i in range(n):
        x1, y1 = hull[i]
        x2, y2 = hull[(i + 1) % n]
        area = _bbox_area_at(hull, math.degrees(math.atan2(y2 - y1, x2 - x1)))
        if area < best_area:
            best_area = area
            best_ang = math.degrees(math.atan2(y2 - y1, x2 - x1))
    zero_area = _bbox_area_at(hull, 0.0)
    ratio = (best_area / zero_area) if zero_area > 0 else 1.0
    return best_ang % 90, ratio


def triangle_rotation(hull, half=1.0):
    """관측 껍질을 표준 삼각형 템플릿에 맞춰 회전각 추정.
    ENABLE_ROTATION=False 인 동안에는 호출되지 않는다."""
    tri = dp_simplify(hull)
    if len(tri) < 3:
        return 0.0
    tri = tri[:3]
    ocx = sum(p[0] for p in tri) / 3
    ocy = sum(p[1] for p in tri) / 3
    obs_ang = [math.atan2(p[1] - ocy, p[0] - ocx) for p in tri]
    template = [(0, half), (-half, -half), (half, -half)]
    tcx = sum(p[0] for p in template) / 3
    tcy = sum(p[1] for p in template) / 3
    tmpl_ang = [math.atan2(p[1] - tcy, p[0] - tcx) for p in template]
    best_rot, best_err = 0.0, float('inf')
    for shift in range(3):
        diffs = []
        for i in range(3):
            d = obs_ang[i] - tmpl_ang[(i + shift) % 3]
            while d > math.pi:
                d -= 2 * math.pi
            while d < -math.pi:
                d += 2 * math.pi
            diffs.append(d)
        mean_d = sum(diffs) / 3
        err = sum((d - mean_d) ** 2 for d in diffs)
        if err < best_err:
            best_err, best_rot = err, mean_d
    return math.degrees(best_rot) % 360


# =============================================================================
# =============================================================================
#  7. 정답 월드 생성                              [정답 O / 시뮬 전용]
# =============================================================================
# =============================================================================
#
# ★ 실기에는 이 섹션이 존재하지 않는다. 실제 아레나가 이 역할을 한다.
#   여기서 만든 truth_id / hidden_obstacles 를 로봇 로직이 절대 읽지 않는 것이
#   이 시뮬의 신뢰성 전부다. 읽어도 되는 곳은 딱 세 곳:
#     - real_scan (섹션 8): 물리 판정에만
#     - bump_cells (섹션 14): 범퍼
#     - evaluate (섹션 16): 채점

def obstacle_half(size):
    """장애물 반폭(칸).

    [버그1 수정] 예전엔 max(size // 2, 3) 였는데 size 가 1~4 라
    size // 2 <= 2 < 3 이므로 항상 3 이 나왔다. 즉 모든 장애물이 예외 없이
    7x7(또는 반지름 3)로 생성돼, 크기 다양성이 0 인 상태로 크기 추정기를
    검증한 셈이었다."""
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
    """장애물 배치. 로봇 시작 위치의 몸통(5x5) + 여유를 확실히 비워둔다.

    min_sep: 장애물 표면 사이 최소 간격(칸). 예전 코드에는 이 제약이 없어서
    장애물이 서로 겹치거나 맞닿게 생성됐고, 그 결과 ToF 관측만으로는
    분리가 불가능한 덩어리가 시드마다 대여섯 개씩 만들어졌다.
    정답 참조 association 을 쓰던 동안에는 이게 드러나지 않았다."""
    if min_sep is None:
        min_sep = OBSTACLE_MIN_SEP
    obstacles = {}
    attempts = 0
    while len(obstacles) < count and attempts < count * 200:
        attempts += 1
        size = rng.choice([2, 2, 3, 3, 4])       # -> 반폭 2/3/4, 폭 5/7/9
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
    """정답 점유 격자와 장애물 ID 격자를 한 번에 만든다.

    [버그6 수정] 예전 real_scan 은 광선의 셀마다 hidden_obstacles 를 전수
    순회했다(광선 25칸 x 30개 x 장애물 12개 = 스텝당 9000회 기하 판정).
    ID 격자를 미리 만들어두면 O(1) 조회로 끝난다."""
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


def reset_world(seed, robot_start=(10, 10)):
    """월드 + 로봇 + 모든 추정 상태를 초기화. 시드가 같으면 항상 같은 월드."""
    global grid, truth_id, obstacle_ids, hidden_obstacles
    global robot_x, robot_y, robot_angle, step, collisions
    global hit_obs, cluster_cells, _next_cid, low_confidence_obstacles
    global unreachable_unknowns, last_estimated_shape, obstacle_render_info
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
    # 한 바퀴(끝에서 끝까지) 스윕 시간. SERVO_MAX_OFF 나 서보 속도를 CLI 로
    # 바꿔도 따라오도록 여기서 계산한다.
    FULL_SWEEP_S = (2 * SERVO_MAX_OFF) / SERVO_SWEEP_DPS

    grid = np.full((GRID_SIZE, GRID_SIZE), -1, dtype=np.int64)
    logodds = np.zeros((GRID_SIZE, GRID_SIZE), dtype=np.float64)
    # 센서 오차용 난수는 월드 생성 rng 와 분리한다. 같이 쓰면 노이즈를 켜고 끌 때
    # 장애물 배치까지 바뀌어 A/B 비교가 성립하지 않는다.
    sensor_rng = random.Random(seed * 7919 + 13)
    hidden_obstacles = generate_random_obstacles(OBSTACLE_COUNT, robot_x, robot_y, rng)
    truth_id, obstacle_ids = build_truth(hidden_obstacles)

    hit_obs = {}
    cluster_cells = {}
    _next_cid = 0
    low_confidence_obstacles = set()
    unreachable_unknowns = set()
    last_estimated_shape = {}
    obstacle_render_info = {}
    last_filled = {}
    last_obs_count = {}
    cell_owner = {}

    # 시작 지점이 정말 안전한지 확인 (여기서 실패하면 생성 파라미터 문제)
    for dy in range(-ROBOT_MARGIN, ROBOT_MARGIN + 1):
        for dx in range(-ROBOT_MARGIN, ROBOT_MARGIN + 1):
            assert truth_id[robot_y + dy][robot_x + dx] < 0, "시작 지점이 장애물과 겹침"


# =============================================================================
# =============================================================================
#  8. 센서 시뮬 -> 관측 기록                                        [경계]
# =============================================================================
# =============================================================================
#
# ★ 실기 전환의 핵심 접합부.
#   real_scan 이 실물 ToF + 서보 스윕으로 교체될 자리다.
#   truth_id 를 읽지만 '거기 물체가 있다'는 물리 판정에만 쓰고,
#   **어느 물체인지는 로봇에게 넘기지 않는다.** 이 구분이 전부다.

def get_scan_angles(base_angle):
    """base_angle 을 중심으로 CAMERA_FOV 를 SCAN_COUNT 갈래로 나눈 각도 목록."""
    half_fov = CAMERA_FOV / 2
    return [base_angle - half_fov + (CAMERA_FOV / (SCAN_COUNT - 1)) * i
            for i in range(SCAN_COUNT)]


def servo_reachable(angle_deg):
    """이 절대각이 현재 헤딩에서 서보만으로 닿는가.  ★수정5

    실물 서보는 SERVO_CENTER ± SERVO_MAX_OFF 안에서만 움직인다.
    로봇 뒤쪽을 보려면 몸통을 돌려야 한다."""
    if not LIMIT_SCAN_TO_SERVO:
        return True
    d = (angle_deg - robot_angle + 180) % 360 - 180
    return abs(d) <= SERVO_MAX_OFF + 1e-6


def turn_body_to_reach(aim_deg):
    """aim_deg 가 서보 범위 밖이면 '딱 닿을 만큼만' 몸통을 돌린다.  ★수정5

    반환: 스캔을 진행해도 되면 True, 회전이 불가(TURN_TO_AIM=False)면 False.

    최소 회전만 하는 이유: aim 을 정면으로 삼으면 필요 이상으로 돌게 되고,
    실물에서 회전은 가장 비싼 동작이다. 서보 끝이 닿기만 하면 충분하다.

    ★수정6: 연속 스윕 모드에서는 회전하는 동안에도 ToF 가 계속 돈다.
      회전은 순수 비용이 아니라 관측 기회이기도 하다."""
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


# =============================================================================
#  8-a. ToF 원뿔 각도 절단                                          [정답 X]
# =============================================================================
#                                                                    ★수정9
# real_scan 이 물리적으로 히트를 얻는 것과, 그 히트가 '물체 경계'로서
# 신뢰할 만한지는 별개의 판단이다. 이 판단은 real_scan 안이 아니라 여기서
# 한 다리(leg) 전체를 보고 나서 한다 - 개별 광선만 봐서는 자기가 구간의
# 끝인지 알 수 없기 때문이다.

def trim_cone(rays, leg_is_full_sweep=True):
    """rays: 한 스윕 다리(leg)에서 각도 순으로 쌓인
        [(servo_off_deg, abs_angle_deg, dist_or_None, hit_cell_or_None, rx, ry), ...]

    히트가 연속되는 구간(run)을 찾아 양 끝을 CONE_TRIM_DEG(원뿔 반각,
    12.5도) 만큼 잘라낸다. 원뿔 가장자리가 물체를 스치기만 해도 히트가
    나므로, 구간 끝쪽 샘플은 실제 경계가 아니라 원뿔이 지어낸 위치일
    가능성이 높다.

    ★ run 이 이 다리(rays 리스트)의 맨 처음/맨 끝 샘플과 맞닿아 있고
      leg_is_full_sweep 이면 그쪽은 자르지 않는다. 서보가 물리 한계에
      닿아서 끊긴 것이지 물체 경계가 아니기 때문에, 그대로 자르면 존재하지
      않는 경계를 잘라내는 꼴이 된다. (float 각도 비교 대신 리스트 인덱스로
      판정한다 - 서보 반사 계산이 부동소수라 정확히 ±SERVO_MAX_OFF 에
      떨어지지 않는다.)"""
    idx_runs, cur = [], []
    for idx, r in enumerate(rays):
        if r[2] is not None:          # dist 필드
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
            continue                  # 구간이 원뿔보다 좁다 = 분해능 이하
        out += [r for _, r in run if lo <= r[0] <= hi]
    return out


def _flush_sweep_buffer(end_is_limit):
    """모아둔 스윕 다리를 trim_cone 으로 거른 뒤 record_hit 한다.

    end_is_limit: 이번에 버퍼를 비우는 이유가 '서보가 물리 한계에 닿아
    방향을 반전했기 때문'이면 True, 시뮬레이션이 끝나 강제로 비우는 것뿐이면
    False. 다음 다리가 물리 한계에서 '시작'하는지는 이번이 한계에서
    '끝났는지'로 정해진다(스윕은 끝~끝으로 이어지므로)."""
    global _sweep_buffer, _leg_start_is_limit
    if _sweep_buffer:
        full = _leg_start_is_limit and end_is_limit
        for r in trim_cone(_sweep_buffer, leg_is_full_sweep=full):
            _, angle, dist, hit_cell, x, y = r
            if hit_cell is not None:
                record_hit(hit_cell, angle, dist, x, y)
    _sweep_buffer = []
    _leg_start_is_limit = end_is_limit


# =============================================================================
#  8-b. 연속 스윕 모델  ★수정6                                     [경계]
# =============================================================================
#
# 실물 ESP32 MODE_SWEEP 을 흉내낸다. 서보는 스텝 경계와 무관하게 계속 왕복하고,
# ToF 는 TOF_PERIOD_S 마다 한 점씩 쏜다. 로봇이 움직이는 동안에도 마찬가지다.
#
# sweep_during 이 이 모델의 전부다. "이 동작이 duration_s 걸린다"만 알려주면
# 그 사이의 관측을 알아서 채운다. 이동이든 회전이든 제자리든 같은 함수를 쓴다.

def _advance_servo(dt):
    """서보를 dt 초만큼 전진시키고 현재 오프셋각을 반환.
    끝에 닿으면 반사시킨다(왕복). 반사 지점을 정확히 처리하지 않으면
    스윕 끝 각도만 과다 표본되어 지도가 부채꼴로 뭉친다."""
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
    """duration_s 동안 TOF_PERIOD_S 마다 광선 1개를 쏜다.

    pose_at(t) 이 그 시각(0..duration_s)의 (x, y, heading) 을 준다.
    이동 중이면 위치를 보간하고, 회전 중이면 헤딩만 바뀐다.

    ★ 관측 좌표가 실수라는 점이 중요하다. real_scan 은 floor 로 셀을
      판정하므로 실수 포즈를 그대로 받아도 되고, record_hit 은 VP_QUANT 로
      양자화해 중복을 거른다. 여기서 정수로 반올림해버리면 '이동 중 관측'의
      의미가 사라진다.

    ★수정9: real_scan 은 더 이상 히트를 바로 record_hit 하지 않는다
      (USE_CONE_TRIM=True 일 때). 여기서 _sweep_buffer 에 모으고, 서보가
      방향을 반전하는 순간(한 다리 종료) trim_cone 을 거쳐 record_hit 한다.
      USE_CONE_TRIM=False 면 예전처럼 광선마다 바로 기록한다(A/B 비교용)."""
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
            if servo_dir != prev_dir:            # 서보가 방향을 바꿨다 = 다리 종료
                _flush_sweep_buffer(end_is_limit=True)
        elif hit_cell is not None:
            record_hit(hit_cell, angle, dist, x, y)
    sim_time += duration_s


def sweep_while_turning(x, y, a0, delta_deg):
    """제자리 회전 중 스윕. 위치는 고정, 헤딩만 선형 변화."""
    dur = abs(delta_deg) / ROBOT_TURN_DPS
    if dur <= 0:
        return
    sweep_during(dur, lambda t: (x, y, a0 + delta_deg * (t / dur)))


def sweep_while_moving(x0, y0, x1, y1, heading):
    """직선 이동 중 스윕. 헤딩 고정, 위치만 보간."""
    dist_cells = math.hypot(x1 - x0, y1 - y0)
    dur = dist_cells * CELL_SIZE_M / ROBOT_SPEED_MPS
    if dur <= 0:
        return
    sweep_during(dur, lambda t: (x0 + (x1 - x0) * (t / dur),
                                 y0 + (y1 - y0) * (t / dur),
                                 heading))
    return dur


def sweep_in_place(duration_s):
    """정지 상태 스윕. 첫 스텝 시동 / 이동할 경로가 없는 스텝에 쓴다.

    가만히 있으면 지도가 안 변해 cleanup 이 라이브락에 빠진다."""
    sweep_during(duration_s, lambda t: (robot_x, robot_y, robot_angle))


def aim_sweep(g, aim_deg):
    """연속 모드의 '조준'. MODE_AIM 처럼 광선마다 겨냥하는 게 아니라,
    필요하면 몸통을 돌리고 제자리에서 한 바퀴 스윕한다.

    실물에서 광선마다 겨냥하면 200~900ms 씩 드는데, 그게 정지 일괄 모델이
    2시간이 걸리던 이유다. 연속 모드에서는 그러지 않는다."""
    if not turn_body_to_reach(aim_deg):
        return False
    sweep_in_place(FULL_SWEEP_S)
    return True


def scan_toward(g, rx, ry, aim_deg, extra=(-4, -2, 0, 2, 4)):
    """지정 방향으로 스윕한다. 서보 범위를 벗어나면 몸통을 돌린다.  ★수정5

    예전에는 조준 스캔이 aim_deg 를 그대로 써서 로봇 뒤쪽도 공짜로 훑었다.
    서보로는 물리적으로 닿지 않는 방향이라, 한 스텝에 앞 FOV + 임의 방향 FOV
    를 보는 셈이었다. 그래서 커버리지·스텝 수치가 전부 낙관적이었다.

    회전 후에도 get_scan_angles(aim_deg) 의 양 끝은 범위를 벗어날 수 있으므로
    (aim 이 헤딩에서 떨어져 있으면 스윕 끝이 SERVO_MAX_OFF 를 넘는다)
    광선 하나하나를 servo_reachable 로 다시 거른다. 이걸 빼먹으면
    '회전은 세는데 여전히 못 볼 각도를 본다'는 어중간한 상태가 된다.

    ★수정9: 이 함수는 CONTINUOUS_SCAN=False(정지 일괄, 레거시) 모드에서만
      쓰인다. 원뿔 절단은 CONTINUOUS_SCAN=True 경로에만 적용했으므로
      (모듈 docstring 참고) 여기서는 예전과 동일하게 광선마다 바로
      record_hit 한다. real_scan 이 더 이상 자체적으로 record_hit 을
      부르지 않으므로, 반환값을 받아 여기서 직접 불러줘야 한다."""
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
    """직접 관측 하나를 지도에 통합한다.  ★실물 대응 수정 1

    USE_LOGODDS=True 면 증거를 누적하고 임계값으로 0/3 을 파생시킨다.
    핵심은 **값 3 이 되돌아올 수 있다**는 것이다. 예전에는 한번 3이면 영원히
    3이라 오측 한 번이 영구 유령 장애물을 만들었다.

    관측은 추정(1,2)과 포기(4)를 이긴다 - 실제로 본 것이 추측보다 우선이다.
    핑크(4)도 덮어쓰는데, 관측된 칸은 0 이나 3 이 되지 -1 로 돌아가지 않으므로
    find_nearest_unknown 이 다시 노리는 일은 없다(무한루프 안 생김).

    ★수정9: 원뿔 절단은 여기(observe_cell)에는 적용하지 않는다. 안전판정에
      쓰는 직접관측(3)은 원뿔 끝이 스친 히트라도 '거기 뭔가 있다'는 사실
      자체는 유효하다 - 절단이 필요한 건 '경계가 정확히 어디냐'를 묻는
      형태 추정(record_hit 이후 파이프라인)뿐이다."""
    if not USE_LOGODDS:
        # ---- 예전 하드 값 방식 (A/B 비교용) ----
        if hit:
            g[y][x] = 3
        elif g[y][x] in (-1, 1, 2):
            g[y][x] = 0
        return
    lo = logodds[y][x] + (L_OCC if hit else L_FREE)
    logodds[y][x] = max(L_MIN, min(L_MAX, lo))
    g[y][x] = 3 if logodds[y][x] >= L_THRESH else 0


def _cast_ray_truth(rx, ry, angle_deg, max_range):
    """순수 광선 캐스팅. truth_id 만 보고, 노이즈 주입도 grid 마킹도 안 한다.
    ★수정10(원뿔 모델)에서 원뿔 안 서브 광선들을 값싸게 여러 번 쏘기 위해
    real_scan 에서 분리했다."""
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
    """원뿔(TOF_FOV_DEG 폭) 안에서 CONE_SUBRAYS개의 서브 광선을 쏴서 그중
    가장 가까운 히트의 (실제 반사 위치 셀, 거리) 를 구한다.  ★수정10

    [왜 최솟값인가] VL53L0X 는 원뿔 안에서 반사된 광자 히스토그램의 최강
    피크를 고르는데, 신호 세기가 1/거리^2 이라 원뿔 안에 물체가 있으면 가장
    가까운 표면이 항상 이긴다. 결과적으로 '원뿔 안 최소 거리'와 사실상 같다
    (플럭스 모델로 실험해도 결과가 거의 동일했음 - 지난 세션 검증).

    ★ 셀 위치는 '실제로 반사가 일어난 서브 광선'의 기하를 그대로 쓴다
    (명목 각도로 그 거리만큼 걸어간 칸이 아니다). 명목 각도 walk 로 찍으면
    실제 물체가 없는 칸에 유령 히트가 찍힌다 - 실측으로 확인된 버그
    (grid 유령셀이 히트 수만큼 거의 그대로 생겼었음). 반사가 실제로 어디서
    일어났는지는 물리적으로 그 지점이 맞으므로, 안전판정용 grid 마킹은
    거기가 정확하다. '명목 각도로 잘못 귀속'되는 건 record_hit 에 저장되는
    각도값(형태 추정용) 쪽이지, grid 위치가 아니다 - 이 둘을 헷갈리면 안 된다.

    노이즈 주입은 여기서 하지 않는다(순수 거리/위치 탐색). real_scan 이
    이 결과를 받아 한 번만 처리한다."""
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
    """ToF 측정 1회를 흉내낸다. 지나간 칸은 관측된 빈공간으로 기록.

    [수정] range(max_range) -> range(max_range + 1).
    예전에는 i=0..24 라 최대 24칸까지만 갔는데, has_line_of_sight 는
    dist <= 25 를 통과시킨다. 그 틈(거리 24~25칸) 셀은 '보인다'고 판정되고서
    스캔은 안 닿아, cleanup 에서 곧바로 핑크로 영구 확정됐다.
    빈 지도 실측: 사거리 내 1960셀 중 89셀(4.5%)이 정확히 겨냥해도 안 닿았고
    그중 83셀이 이 off-by-one 때문이었다.

    ★수정9: 반환값이 바뀌었다. 예전엔 히트 시 이 함수 안에서 바로 record_hit
      을 불렀는데, 이제는 (hit_cell, distance) 를 반환만 하고 record_hit 은
      호출부(sweep_during/scan_toward/run 의 정지모드 분기)가 대신한다.
      원뿔 절단(trim_cone)이 '이 히트가 구간 어디쯤인지'를 알아야 하는데,
      그건 개별 광선 하나만 봐서는 알 수 없고 한 다리(leg) 전체를 모아야
      판단할 수 있기 때문이다.

    ★수정10: USE_CONE_MODEL=True 면 각도 하나로 '폭 0도 레이저'를 쏘던 예전
      방식 대신, TOF_FOV_DEG 폭 원뿔 안에서 여러 서브 광선을 쏴 최근접
      거리 + 그 실제 반사 위치를 취한다(_cone_min_distance). 이게 없으면
      애초에 원뿔이 물체를 부풀려 보이게 하는 물리 현상 자체가 시뮬에 없어서,
      trim_cone 이 존재하지도 않는 문제를 보정하려다 멀쩡한 데이터만 깎아내는
      역효과를 냈다(실측 확인: 절단 켬 IoU 0.79 vs 끔 0.98).

      grid 마킹(observe_cell, 안전판정용)과 반환하는 hit_cell 은 **실제로
      반사가 일어난 서브 광선의 물리 위치**를 쓴다 - 명목 각도(angle_deg)로
      그 거리만큼 걸어간 칸이 아니다(실측으로 잡아낸 버그: 그렇게 하면
      실제 물체가 없는 칸에 유령 히트가 대량으로 찍힘). '명목 각도로 잘못
      귀속됨'은 record_hit 에 저장되는 각도값 쪽에서 일어나야 하는 것이고
      (호출부가 angle_deg 를 그대로 넘긴다), 그게 바로 원뿔 부풀림의 원인
      이며 trim_cone 이 사후에 보정하는 대상이다."""
    global rays_fired
    rays_fired += 1

    if USE_CONE_MODEL:
        hit_cell, dist = _cone_min_distance(rx, ry, angle_deg, max_range)
    else:
        hit_cell, dist = _cast_ray_truth(rx, ry, angle_deg, max_range)

    occupied = dist is not None
    # ---- 센서 오차 주입. 측정 1회에 1번 - i>=1(자기 발밑 제외)과 같은 취지로
    # dist>=1 일 때만 MISS 를 굴린다. ----
    if occupied and dist >= 1 and SENSOR_MISS > 0 and sensor_rng.random() < SENSOR_MISS:
        occupied = False
        dist = None
        hit_cell = None                 # 흡수/입사각 때문에 그냥 통과해버림
    elif (not occupied) and SENSOR_FALSE_HIT > 0 \
            and sensor_rng.random() < SENSOR_FALSE_HIT:
        occupied = True                 # 반사/원뿔이 모서리 걸침 -> 헛것을 봄
        dist = sensor_rng.randint(1, max_range)
        ar = math.radians(angle_deg)    # 헛것이라 실제 반사 위치가 없으므로
        hit_cell = (math.floor(rx + dist * math.cos(ar)),
                    math.floor(ry + dist * math.sin(ar)))   # 명목각 위치로 대체

    # ---- 빈 공간 마킹: 명목 각도 방향으로 dist 이전 칸까지 walk ----
    # 원뿔 전체가 그 거리까지는 비어 있었다는 뜻이므로, 대표로 명목 각도
    # 경로를 걸어가며 표시한다(원뿔 전체를 다 마킹하면 너무 비싸다).
    # ★ 히트 없을 때는 max_range+1 까지 걸어야 한다 - range(max_range) 로
    # 쓰면 위 docstring 에 적힌 off-by-one 버그(거리 24~25칸 틈)가 재발한다.
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
    """ToF 히트 하나를 기록한다. **어느 장애물인지는 모른다.**

    [정답참조 제거] 예전엔 real_scan 이 truth_id 를 보고 '이 히트는 몇 번
    장애물'이라고 알려줬다. 실기에는 그런 정보가 없다. 이제 히트 셀 단위로만
    쌓고, 묶는 일은 rebuild_clusters() 가 거리 기반으로 직접 한다.

    같은 관측지점에서 같은 셀을 다시 맞히면 새 정보가 없으므로 버린다.

    ★수정6: 관측지점을 VP_QUANT(0.5칸) 격자로 양자화한다.
      연속 스윕에서는 이동 중 관측이라 (rx, ry) 가 실수다. 양자화 없이는
      모든 관측이 서로 다른 지점으로 잡혀 MAX_VIEWPOINTS_PER_CELL(60)이
      금세 차고, 정작 나중에 반대편에서 본 관측이 못 들어온다.
      그러면 viewpoint_spread 가 안 올라가 확정 승격이 막힌다.
      정지 모드에서는 rx, ry 가 정수라 양자화가 항등이므로 영향이 없다.

    ★수정9: 호출 시점이 바뀌었다. 예전엔 real_scan 이 히트 즉시 불렀는데,
      이제는 (연속 모드에서) trim_cone 을 통과한 히트만 여기로 들어온다."""
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
    """클러스터의 전체 관측 목록 [(angle, dist, rx, ry), ...]."""
    out = []
    for cell in cluster_cells.get(cid, ()):
        for (rx, ry), (a, d) in hit_obs[cell].items():
            out.append((a, d, rx, ry))
    return out


def cluster_obs_count(cid):
    """관측 '횟수'(히트셀 수가 아니라 관측지점 수의 총합)."""
    return sum(len(hit_obs[c]) for c in cluster_cells.get(cid, ()))


def evaluate_confidence(cid):
    """관측 다양성만 보는 값싼 신뢰도(시각화용).

    여기는 광선각 스프레드를 그대로 쓴다. 기준이 SHAPE_MIN_SPREAD(63) 라
    CONFIRM_SPREAD(180) 만큼 과대평가에 민감하지 않다."""
    obs = cluster_obs(cid)
    if len(obs) < 2:
        return 0.0
    return 0.2 if circular_spread([o[0] for o in obs]) < SHAPE_MIN_SPREAD else 0.7


# =============================================================================
# =============================================================================
# 10. 클러스터링 (히트셀 -> 장애물 단위로 묶기)                     [정답 X]
# =============================================================================
# =============================================================================
#
# 실기의 DBSCAN 자리. 정답 라벨 없이 거리만 보고 묶는다.
# 붙어 있는 두 장애물이 하나로 병합되는 실패 모드가 여기서 그대로 재현되는데,
# 그게 이 설계의 목적이다(실물에서 실제로 일어나는 일이므로).

def _drop_cluster(cid):
    """사라진 클러스터(주로 이웃과 병합됨)가 칠해둔 칸을 되돌린다."""
    for cell in last_filled.pop(cid, ()):
        if cell_owner.get(cell) != cid:
            continue
        x, y = cell
        if grid[y][x] in (1, 2):
            grid[y][x] = -1
        cell_owner.pop(cell, None)
    obstacle_render_info.pop(cid, None)
    last_estimated_shape.pop(cid, None)
    last_obs_count.pop(cid, None)
    low_confidence_obstacles.discard(cid)


def rebuild_clusters():
    """히트 셀을 거리 기반으로 묶어 클러스터를 만든다. 정답 미참조.

    CLUSTER_GAP 이하로 붙어 있는 히트 셀은 같은 물체로 본다(union-find).
    실기의 DBSCAN 자리에 해당하며, 붙어 있는 두 장애물이 하나로 병합되는
    실패 모드가 여기서 그대로 재현된다 - 그게 이 변경의 목적이다.

    클러스터 ID 는 이전 프레임과 겹치는 칸이 가장 많은 쪽을 물려받아
    유지된다. 그래야 last_filled / cell_owner 되돌림이 계속 유효하다."""
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
                    continue                      # 각 쌍을 한 번만
                n = (x + dx, y + dy)
                if n in cellset:
                    ra, rb = find((x, y)), find(n)
                    if ra != rb:
                        parent[ra] = rb

    groups = {}
    for c in cells:
        groups.setdefault(find(c), set()).add(c)

    # 이전 ID 승계: 큰 클러스터부터 겹침이 가장 큰 이전 ID 를 가져간다
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
        _drop_cluster(cid)                        # 병합돼 사라진 클러스터 정리
    cluster_cells = new_map


# =============================================================================
# =============================================================================
# 11. 형태 추정 -> 지도 반영                                        [정답 X]
# =============================================================================
# =============================================================================
#
# 파이프라인: 관측점 복원 -> 볼록껍질 -> 형태/크기/중심 추정 -> 셀 채우기
#
#   estimate_obstacle   : 관측점만으로 (형태, 중심, 크기) 를 뽑는다  [순수 계산]
#   cells_of_shape      : 그 도형이 덮는 셀 목록을 만든다             [순수 계산]
#   update_obstacle_shape: grid 에 실제로 칠하고 소유권을 관리한다   [상태 변경]
#
# 형태 판별에 쓰는 지표 세 개와 왜 그걸 골랐는지:
#   thickness (min축/max축) : 앞면만 봤는지 판단. 얇으면 판정 보류(unknown)
#   circ (원형도)           : 원 판별용. 삼각/사각은 값이 겹쳐서 구분 못 함
#   extent (면적비)         : 삼각형 vs 사각형. 이게 유일하게 완전 분리됨

def estimate_obstacle(observations):
    """관측점만으로 (형태, cx, cy, half, 회전각, half_w, half_h) 추정. 정답 미참조.
    - 형태: 원형도(circularity) + 면적비(extent)
    - 크기: 관측점 확산 폭 (셀 격자로 양자화해 과대추정 방지)
    - 중심: 양자화된 관측 bbox 중심 (push 보정은 실측상 해로워 제거함)
    껍질이 얇으면(앞면만 봄) 형태 판정은 보류(unknown)하고 위치/크기만 준다."""
    if len(observations) < SHAPE_MIN_OBS:
        return None
    pts = [(rx + d * math.cos(math.radians(a)), ry + d * math.sin(math.radians(a)))
           for a, d, rx, ry in observations]
    h = convex_hull(pts)
    if len(h) < 3:
        return None
    A, P = poly_area(h), poly_perimeter(h)
    if P == 0:
        return None
    circ = 4 * math.pi * A / (P * P)

    xs = [p[0] for p in pts]
    ys = [p[1] for p in pts]
    if max(max(xs) - min(xs), max(ys) - min(ys)) <= 0:
        return None

    # [수정1] 양자화를 thickness 계산 앞으로 당겼다.
    # real_scan 은 floor 로 셀을 판정하는데 복원점은 rx + d*cos(a) 실수 좌표라
    # 셀 경계 안에서 최대 1칸 가까이 부푼다. 예전엔 half 만 양자화하고
    # thickness 는 실수 좌표로 계산했다.
    # 이 부풀이는 두 축에 비슷한 '절대량'으로 붙으므로, 얇은 축이 비율로 훨씬
    # 크게 부푼다 -> thickness 과대평가.
    # 그 방향이 나쁘다 - THICKNESS_MIN 은 '앞면만 봤으면 판정 보류'라는
    # 안전장치인데, 통과되기 쉬워져 부분 관측으로 형태를 확정해버린다.
    # 실측(앞면만 본 사각형 396케이스): 양자화 시 thickness 감소 331 / 증가 65,
    # 평균 0.446 -> 0.372. 판정 뒤집힘은 한 방향뿐 - 통과->보류 45건, 그 반대 0건.
    # 개별 예: 7x7 앞면만 관측 -> 실수 3.69/6.67=0.553 (통과, 형태 확정)
    #                            양자화 3/6=0.500 (보류, unknown)
    qxs = [math.floor(x) for x in xs]
    qys = [math.floor(y) for y in ys]
    qw, qh = max(qxs) - min(qxs), max(qys) - min(qys)
    qmax = max(qw, qh)
    # 히트 셀이 한 칸인데 관측지점만 여러 개면(MAX_VIEWPOINTS_PER_CELL) qmax 가
    # 0 이 될 수 있다. SHAPE_MIN_OBS 는 관측 횟수 기준이라 통과되고 껍질도
    # 실수 좌표라 3점 이상 나와서 여기까지 도달한다. 그때는 thickness 0 -> unknown.
    thickness = (min(qw, qh) / qmax) if qmax > 0 else 0.0

    # 면적비(extent) = 껍질면적 / 껍질 bbox면적.
    #
    # ★수정8: bbox 를 '축정렬'로 잡으면 회전한 물체에서 완전히 무너진다.
    #   45도 돌아간 정사각형은 한 변 s 인데 축정렬 bbox 변이 s*root2 라
    #       extent = s^2 / 2s^2 = 0.5
    #   즉 회전만으로 사각형의 extent 가 삼각형 값(0.5)까지 떨어진다.
    #   circ 는 0.785 라 삼각형 문턱(0.78)을 간신히 넘으므로 '원'으로 직행한다.
    #
    #   [실측] 정답 도형을 임의 각도로 회전시킨 월드(시드 144~147):
    #     형태정확 1.00 -> 0.37.  사각형 23개 중 15개, 삼각형 16개 중 12개가 원.
    #     ENABLE_ROTATION 을 켜도 그대로였다 - 그건 '그리는 각도'만 바꿀 뿐
    #     extent 계산에는 관여하지 않기 때문이다.
    #
    #   최소외접사각형(min-area rect)을 분모로 쓰면 회전에 불변이 된다.
    #   정사각형은 각도와 무관하게 1.0, 삼각형은 0.5 가 나온다.
    hxs = [p[0] for p in h]
    hys = [p[1] for p in h]
    hbox = (max(hxs) - min(hxs)) * (max(hys) - min(hys))
    if EXTENT_MIN_AREA and len(h) >= 3:
        _, ratio = min_area_rect_angle(h)     # ratio = 최소면적 / 축정렬면적
        denom = hbox * ratio
    else:
        denom = hbox
    extent = (A / denom) if denom > 0 else 1.0

    half = qmax / 2.0 or 0.5
    cx = (max(qxs) + min(qxs)) / 2.0
    cy = (max(qys) + min(qys)) / 2.0

    # ---- 형태 분류 ----
    # cx, cy 가 있어야 viewpoint_spread 를 잴 수 있어 중심 계산이 위로 올라왔다.
    if thickness < THICKNESS_MIN:
        shape = "unknown"                      # 앞면만 봄 - 두께 자체가 안 나옴
    elif SHAPE_RULE == "v1":
        # ---- v1 (가장 예전 규칙, A/B 비교용) ----
        if circ >= CIRC_MIN:
            shape = "circle"
        else:
            shape = "triangle" if extent <= EXTENT_TRI_MAX else "rect"
    elif viewpoint_spread(observations, cx, cy) < SHAPE_MIN_VP_SPREAD:
        # 한쪽에서만 봤다. 사각형 모서리와 삼각형은 이 상태에서 구분 불가라
        # 틀린 형태를 자신 있게 그리느니 보류한다.
        shape = "unknown"
    elif SHAPE_RULE == "v2":
        # ---- v2: circ 로 삼각형을 먼저, extent 로 원/사각형 ----
        #   축정렬 월드에서만 유효하다. 회전이 들어가면 무너진다.
        if circ < CIRC_TRI_MAX:
            shape = "triangle"
        else:
            shape = "circle" if extent <= EXTENT_CIRC_MAX else "rect"
    else:
        # ---- v3: extent 로 삼각형을 먼저, circ 로 원/사각형 ----
        #   최소외접 extent 는 회전에 불변이라 회전 월드에서도 성립한다.
        if extent < EXTENT_TRI_MAX_V3:
            shape = "triangle"
        elif SHAPE_HOLD_SMALL_ROUND and half < SHAPE_MIN_HALF_FOR_ROUND:
            # 너무 작아 원/사각형을 원리적으로 구분할 수 없다 (위 주석 참고)
            shape = "unknown"
        else:
            shape = "circle" if circ >= CIRC_CIRCLE_MIN else "rect"

    rot_deg = 0.0
    if ENABLE_ROTATION:
        if shape == "rect":
            ang, fit_ratio = min_area_rect_angle(h)
            rot_deg = ang if fit_ratio <= ROT_FIT_MAX else 0.0
        elif shape == "triangle":
            rot_deg = triangle_rotation(h, half=half)

    return shape, cx, cy, half, rot_deg, qw / 2.0, qh / 2.0


def cells_of_shape(shape, cx, cy, half, rot_deg=0.0, half_w=None, half_h=None):
    """추정 도형이 덮는 셀 목록. 정답 미참조.

    [중요] 판정에 int 반올림을 쓰면 안 된다. 벡터 patch 는 실수 (cx, cy, half)로
    그려지므로, 여기서 반올림하면 '격자를 숨긴 칸'과 '도형이 덮는 칸'이 어긋나
    흰 구멍이 생긴다."""
    r = max(half, 0.5)
    rw = max(half_w, 0.5) if half_w is not None else r
    rh = max(half_h, 0.5) if half_h is not None else r
    scan_r = int(math.ceil(max(r, rw, rh) * math.sqrt(2))) + 1
    icx, icy = int(round(cx)), int(round(cy))
    rad = math.radians(-rot_deg)
    cos_r, sin_r = math.cos(rad), math.sin(rad)
    out = []
    for y in range(max(0, icy - scan_r), min(GRID_SIZE, icy + scan_r + 1)):
        for x in range(max(0, icx - scan_r), min(GRID_SIZE, icx + scan_r + 1)):
            dx, dy = x - cx, y - cy
            ldx = dx * cos_r - dy * sin_r
            ldy = dx * sin_r + dy * cos_r
            if shape == "circle":
                ok = dx * dx + dy * dy <= r * r
            elif shape == "triangle":
                ok = point_in_triangle(ldx, ldy, 0, r, -r, -r, r, -r)
            else:
                ok = abs(ldx) <= rw and abs(ldy) <= rh
            if ok:
                out.append((x, y))
    return out


def update_obstacle_shape(g, cid):
    """클러스터의 관측점으로 형태/중심/크기를 추정해 도형을 그린다(속까지 채움).

    [버그9] 예전 시그니처는 (grid, ox, oy, size, real_shape) 였는데 size 와
    real_shape 는 함수 안에서 한 번도 쓰이지 않았고 호출부도 0, None 을 넘겼다.
    이제는 클러스터 ID 하나만 받는다 - 애초에 ox, oy 는 정답 좌표였다."""
    obs = cluster_obs(cid)
    if len(obs) < SHAPE_MIN_OBS:
        return
    # 광선각 스프레드는 값싼 사전 필터로만 쓴다(추정 자체를 시도할지 여부).
    if circular_spread([o[0] for o in obs]) < SHAPE_MIN_SPREAD:
        return
    est = estimate_obstacle(obs)
    if est is None:
        return
    shape, cx, cy, half, rot_deg, half_w, half_h = est
    last_estimated_shape[cid] = shape

    # [수정2] 확정(검정) 판정은 추정 중심 기준 '관측지점 방위각' 커버리지로.
    # 광선각으로 재면 큰 장애물을 한쪽에서 세 번 본 것만으로 180도를 넘긴다.
    # cx, cy 가 필요하므로 estimate_obstacle 뒤로 순서가 내려왔다.
    vp_spread = viewpoint_spread(obs, cx, cy)
    value = 1 if vp_spread >= CONFIRM_SPREAD else 2

    # unknown 은 이웃에 가려져 얇은 띠만 본 상태다. max(폭,높이) 정사각형으로
    # 그리면 본 적 없는 영역까지 튀어나오므로 '본 만큼'의 직사각형으로만 그린다.
    draw_w, draw_h = (half_w, half_h) if shape == "unknown" else (half, half)
    obstacle_render_info[cid] = (shape, cx, cy, half, rot_deg, value, draw_w, draw_h)

    new_cells = cells_of_shape(shape, cx, cy, half, rot_deg, draw_w, draw_h)
    new_set = set(new_cells)

    # [버그3 수정] 되돌림은 '내가 소유한 칸'에 대해서만.
    # 예전엔 grid 값이 1/2 이기만 하면 되돌려서, 겹친 이웃 장애물이 칠한 칸까지
    # -1 로 지워버렸다.
    for cell in last_filled.get(cid, ()):
        if cell in new_set or cell_owner.get(cell) != cid:
            continue
        x, y = cell
        if g[y][x] in (1, 2):
            g[y][x] = -1          # 이 칸은 실제로 한 번도 직접 관측된 적이 없다
        cell_owner.pop(cell, None)

    for cell in new_cells:
        owner = cell_owner.get(cell)
        if owner is not None and owner != cid:
            continue              # 이웃 클러스터가 이미 점유 -> 건드리지 않음
        x, y = cell
        if g[y][x] in (-1, 2):
            g[y][x] = value
        cell_owner[cell] = cid

    last_filled[cid] = new_cells


# =============================================================================
# =============================================================================
# 12. 경로 계획 (도달성)                                            [정답 X]
# =============================================================================
# =============================================================================
#
# grid 만 보고 "어디까지 갈 수 있고, 비용이 얼마인가"를 계산한다.
# 16방향(8방향 + 나이트 이동) 이동이라 스텝 비용이 균일하지 않으므로
# 균일 BFS 가 아니라 다익스트라를 쓴다.

def _build_move_table():
    """16방향 이동마다 segment_safe 가 실제로 검사하게 되는 '경유 칸' 오프셋을
    미리 뽑아둔다.

    [성능] 예전 bfs_from 은 노드마다 16개 이웃 각각에 대해 segment_safe 를
    호출했다(내부에서 삼각함수 + 반올림을 7회씩). 3600칸 x 16 x 7 = 40만 회를
    매 스텝 파이썬으로 돌아 시뮬이 시드당 50초씩 걸렸다.
    경유 칸은 이동 오프셋에만 의존하므로 한 번만 계산하면 된다.

    [버그11] 덤으로 path_planner.segment_safe 의 홀짝 의존 버그도 여기서 막는다.
    segment_safe 는 int(round(절대좌표)) 를 쓰는데 파이썬 round 는 은행가 반올림이라
    round(10.5)=10, round(11.5)=12 로 결과가 갈린다. 그래서 똑같은 대각 이동이
    시작 x가 짝수면 코너 칸을 검사하지 않고, 홀수면 검사한다(재현 확인함).
    여기서는 항상 양쪽 코너를 검사하는 보수적 규칙으로 통일한다 - 몸통이 5x5라
    코너 자르기는 어차피 허용하면 안 된다."""
    table = []
    for (dx, dy), cost in get_neighbors_cost(0, 0):
        seg_len2 = dx * dx + dy * dy
        mids = []
        for a in range(min(0, dx), max(0, dx) + 1):
            for b in range(min(0, dy), max(0, dy) + 1):
                if (a, b) in ((0, 0), (dx, dy)):
                    continue
                # 셀 중심과 이동 선분 사이 수직거리. 0.72 는 대각 코너(0.707)는
                # 포함하고 나이트 이동의 바깥 칸(0.894)은 배제하는 값.
                if abs(dx * b - dy * a) / math.sqrt(seg_len2) <= 0.72:
                    mids.append((a, b))
        table.append(((dx, dy), cost, tuple(mids)))
    return tuple(table)


# 모듈 로드 시 한 번만 계산 (위 _build_move_table 정의 뒤에 와야 한다)
MOVE_TABLE = _build_move_table()


def planning_grid(g, strict=True):
    """경로계획용으로 값을 치환한 지도 사본.  ★실물 대응 수정 3

    path_planner.compute_safe_map / compute_clearance_map 은 값 3만 장애물로
    본다. 추정 채움(1,2)을 막고 싶으면 그 값을 3으로 바꿔서 넘기면 된다
    (path_planner 를 건드리지 않고 정책만 바꾸는 방법).

    미확정(2)까지 막을지는 SAFE_BLOCK_UNCONFIRMED 로 고른다. 추정 도형이
    과대하게 그려지는 경우가 있어 기본은 끈다."""
    if not (strict and SAFE_BLOCK_ESTIMATED):
        return g
    gg = g.copy()
    gg[gg == 1] = 3
    if SAFE_BLOCK_UNCONFIRMED:
        gg[gg == 2] = 3
    return gg


def turn_penalty(prev_dir, dx, dy):
    """진입 방향이 바뀔 때 무는 비용.  ★실물 대응 수정 4

    실물에서 회전은 가장 오래 걸리고, 오도메트리 오차의 주범이며,
    제자리 회전 전류 스파이크로 전원 래치를 유발한 이력이 있다.
    90도당 TURN_COST 만큼 물린다."""
    if prev_dir is None or TURN_COST <= 0:
        return 0.0
    a0 = math.degrees(math.atan2(prev_dir[1], prev_dir[0]))
    a1 = math.degrees(math.atan2(dy, dx))
    diff = abs((a1 - a0 + 180) % 360 - 180)
    return TURN_COST * (diff / 90.0)


def bfs_from(g, sx, sy, heading_deg=None):
    """로봇에서 도달 가능한 칸 전체 + 부모포인터/누적비용/진입방향.
    대각/나이트 이동은 실제 거리가 다르므로 균일 BFS 대신 다익스트라를 쓴다.

    reach[(x,y)] = (부모좌표, 누적비용, 진입방향)
      인덱스 0,1 은 예전과 같다(path_from / 목표선정 코드 호환).
      인덱스 2 는 회전 비용 계산용으로 추가됐다.

    [근사 주의] 회전 비용을 정확히 다루려면 상태가 (x, y, 방향)이어야 한다.
    여기서는 부모에 기록된 진입 방향만 보므로, 한번 확정된 칸에 나중에 더 좋은
    방향으로 도달해도 갱신되지 않는다. 최적은 아니지만 상태 수를 16배로
    늘리지 않고 회전을 크게 줄인다.

    [갇힘 방지] SAFE_BLOCK_ESTIMATED 로 추정 장애물까지 막으면, 로봇 주변이
    추정 도형에 둘러싸여 한 칸도 못 가는 상황이 생길 수 있다. 그때는 그 스텝만
    예전 기준(값 3만 차단)으로 다시 풀어 데드락을 피한다."""
    # 현재 로봇이 향한 방향을 시작 진입방향으로 심는다. 안 심으면 첫 이동의
    # 회전이 공짜가 되는데, 정작 우리가 줄이고 싶은 게 그 회전이다.
    start_dir = None
    if heading_deg is not None and TURN_COST > 0:
        ar = math.radians(heading_deg)
        start_dir = (math.cos(ar), math.sin(ar))

    for strict in (True, False):
        pg = planning_grid(g, strict)
        cmap = compute_clearance_map(pg)
        smap = compute_safe_map(pg)
        reach = {(sx, sy): (None, 0.0, start_dir)}
        pq = [(0.0, sx, sy)]
        done = set()
        while pq:
            d, cx, cy = heapq.heappop(pq)
            if (cx, cy) in done:
                continue
            done.add((cx, cy))
            prev_dir = reach[(cx, cy)][2]
            for (dx, dy), stepc, mids in MOVE_TABLE:
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
                nd = (d + stepc + turn_penalty(prev_dir, dx, dy)
                      + (CLEARANCE_COST * lack if lack > 0 else 0.0))
                if (nx, ny) not in reach or nd < reach[(nx, ny)][1]:
                    reach[(nx, ny)] = ((cx, cy), nd, (dx, dy))
                    heapq.heappush(pq, (nd, nx, ny))
        # 한 칸도 못 갔고 아직 완화 여지가 남았으면 느슨한 기준으로 재시도
        if len(reach) > 1 or not (strict and SAFE_BLOCK_ESTIMATED):
            return reach
    return reach


def path_from(reach, target):
    """bfs_from 결과의 부모포인터를 거꾸로 따라가 경로 복원."""
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
#
# "어디를 다음에 볼 것인가"를 정하는 파트. cleanup 단계의 핵심.
#
#   has_line_of_sight      : A 에서 B 가 보이는가
#   find_nearest_unknown   : 가장 가까운 미탐색 칸
#   can_observe             : 이 칸을 볼 수 있는 자리가 어디든 있는가 (있/없음만)
#   find_observation_point  : 그중 가장 싸게 갈 수 있는 자리 (실제 좌표)
#
# ★수정5 관련 한계: can_observe / find_observation_point 는 로봇 '방향'을
#   전혀 보지 않는다. 즉 "저기 서면 이 칸이 보인다"고 판단하는데, 실제로는
#   "저기 서서 그쪽을 향해야" 보인다. 회전이 가능하므로 도달성 판정 자체는
#   여전히 맞지만 비용은 과소평가된다. 조준 회전 비용은 scan_toward 시점에
#   turns 로 잡히므로 총량은 집계되지만, 목표 '선정'에는 반영되지 않는다.
#   더 정확히 하려면 reach 비용에 (그 지점에서 목표를 향하는 회전각)을
#   더해야 한다.

def has_line_of_sight(g, x0, y0, x1, y1, max_range=SCAN_RANGE):
    """(x0,y0) 에서 (x1,y1) 이 보이는지.

    [수정4] 무엇을 시야 차단으로 볼지가 LOS_BLOCK_CONFIRMED 로 갈린다.
    값 3(직접관측)은 광선이 실제로 맞힌 표면 칸뿐이라 드문드문하다.
    3만 막으면 광선이 그 사이를 빠져나가 장애물 뒤 미탐색 칸까지 '보인다'고
    오판하고, 로봇이 거기까지 걸어가 아무것도 못 본 뒤에야 핑크가 된다.
    반대로 확정(1)까지 막으면 추정 도형이 과대하게 그려진 경우 비어있는 칸을
    조기에 핑크로 확정해버린다. 미확정(2)은 어느 쪽이든 투과시킨다."""
    blockers = (1, 3) if LOS_BLOCK_CONFIRMED else (3,)
    dist = math.hypot(x1 - x0, y1 - y0)
    if dist > max_range:
        return False
    steps = int(dist) + 1
    for i in range(1, steps + 1):
        t = i / steps
        x = int(round(x0 + (x1 - x0) * t))
        y = int(round(y0 + (y1 - y0) * t))
        if (x, y) == (x1, y1):
            return True
        if not (0 <= x < GRID_SIZE and 0 <= y < GRID_SIZE):
            return False
        if g[y][x] in blockers:
            return False
    return True


def find_nearest_unknown(g, rx, ry):
    """가장 가까운 미탐색 칸(맨해튼 거리).

    [수정3] unreachable_unknowns 검사를 뺐다 - 죽은 코드였다.
    그 집합에 add 하는 모든 지점에서 grid 도 4 로 같이 세팅하고(3곳 전부),
    값 4 는 어디서도 -1 로 되돌아가지 않는다(되돌림은 전부 `in (1, 2)` 조건).
    따라서 g == -1 결과에 애초에 그 셀들이 들어오지 않아 검사가 발동하지 않았다.

    덤으로 numpy 벡터화. 예전엔 미탐색 수천 칸을 매 스텝 파이썬 루프로 돌았다.
    np.where 가 row-major 순서로 반환하고 argmin 이 첫 최소값을 고르므로
    기존 루프의 strict `<` 타이브레이킹과 결과가 동일하다."""
    ys, xs = np.where(g == -1)
    if xs.size == 0:
        return None
    d = np.abs(xs - rx) + np.abs(ys - ry)
    i = int(np.argmin(d))
    return int(xs[i]), int(ys[i])


def find_confirm_point(g, reach, cx, cy, bearing_deg, max_range=None):
    """(cx,cy) 를 bearing_deg 방향에서 볼 수 있는 도달 가능한 자리.  ★수정7

    find_observation_point 와 목적이 다르다. 저건 '미탐색 칸을 보기만 하면
    되는' 자리를 찾고, 이건 '아직 안 본 방위에서' 보는 자리를 찾는다.
    같은 물체를 이미 본 쪽에서 또 봐야 관측 수만 늘고 커버리지는 그대로다.

    비용 = 이동 비용 + 원하는 방위에서 벗어난 각도 x CONFIRM_BEARING_PENALTY.
    방위를 절대 조건으로 걸면 그 방향이 벽이거나 도달 불가일 때 아무것도
    못 찾으므로, 벌점으로 처리해 차선책을 허용한다.

    거리는 CONFIRM_STANDOFF 근처를 선호한다. 너무 붙으면 ToF 원뿔이 물체를
    벗어나지 못해 좌우 끝을 못 보고, 너무 멀면 표면 표본이 성겨진다."""
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
    """(tx,ty) 를 볼 수 있는 도달 가능한 자리가 하나라도 있는가.
    하나 찾으면 즉시 종료 - '포기해도 되는가' 판정용이라 좌표는 필요 없다."""
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
    """(tx,ty) 를 볼 수 있는 자리 중 이동 비용이 가장 싼 곳.
    can_observe 와 달리 전부 훑어서 최소를 골라야 하므로 조기 종료가 없다."""
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
#
# ★ 실기에서는 물리 법칙이 이 역할을 한다(= 진짜로 박는다).
#
# 왜 필요한가: 경로 안전판정(compute_safe_map)은 값 3(직접관측)만 막는다.
# 그래서 아직 못 본 장애물 속으로는 경로가 그대로 뚫린다. 그 상태로 미탐색이
# 0 이 되면 '커버리지 완료'가 떴는데, 그건 벽을 통과해서 얻은 숫자였다.
# 이 함수가 그걸 충돌로 잡아 collisions 카운터에 남긴다.

def bump_cells(px, py, nx, ny):
    """(px,py) -> (nx,ny) 로 이동할 때 로봇 몸통(5x5)이 '실제' 장애물과
    겹치는 칸 목록.

    16방향 이동에는 2칸짜리 나이트 이동이 있어 중간 칸도 표본을 떠야 한다."""
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
#
# ★ 지금은 빈 함수(stub). 실기에서 motor_commander.send_command 로 교체된다.
#
# 실물 이식 시 확인할 것:
#   - 나이트 이동은 실제 거리가 √5(약 2.24칸)인데 duration_ms 가 고정 200 이다.
#     거리에 비례해 계산하거나 거리 기반 프로토콜로 바꿔야 한다.
#   - 회전이 시간 기반(1도=10ms)인데 팀원 ESP32 펌웨어는 MPU6050 자이로
#     폐루프(coarse/fine 2단계)다. 각도 기반 명령으로 바꿔야 받을 수 있다.
#   - turn 과 forward 를 연속으로 쏘고 완료를 안 기다린다. ack 대기 필요.
#   - ★수정5 이후 scan_toward 도 회전 명령을 낸다. 조준 회전은 이동보다
#     정밀해야 하므로(서보 끝이 목표에 걸쳐야 함) 자이로 폐루프가 특히 중요하다.

def send_command(cmd):
    """실기에서는 motor_commander.send_command 로 교체."""
    pass


# =============================================================================
# =============================================================================
# 16. 채점                                         [정답 O / 시뮬 전용]
# =============================================================================
# =============================================================================
#
# 지표 읽는 법:
#   iou        : 점유 격자가 정답과 얼마나 겹치는가 (지도 품질)
#   shape_acc  : 형태를 맞힌 비율. unknown 보류는 분모에서 제외된다
#   merged     : 진짜 장애물 2개 이상을 하나로 뭉친 클러스터 수 (클러스터링 실패)
#   missed     : 아예 못 잡은 장애물 수
#   confirmed  : 확정(검정) 승격된 클러스터 수 - [수정2] 효과 확인용
#   collisions : 범퍼가 잡은 충돌 (0 이 아니면 경로가 안 본 장애물을 통과 중)
#   pink       : 포기한 칸 수 (LOS_BLOCK_CONFIRMED A/B 비교의 핵심 지표)
#   aim_turns  : 조준을 위한 회전 횟수 - [수정5] 효과 확인용
#   aim_blocked: 서보 범위 밖이라 포기한 스캔 (TURN_TO_AIM=False 일 때만 증가)

def evaluate():
    """정답과 비교한 지표. 시뮬 채점용이며 로봇 로직은 이걸 참조하지 않는다."""
    occ_true = (truth_id >= 0)
    occ_est = np.isin(grid, [1, 2, 3])
    inter = int((occ_true & occ_est).sum())
    union = int((occ_true | occ_est).sum())

    # 클러스터가 실제로 어느 장애물(들)의 히트를 담고 있는지로 채점한다.
    merged = 0          # 두 개 이상의 진짜 장애물을 하나로 뭉친 클러스터 수
    covered = set()     # 클러스터에 잡힌 진짜 장애물
    shape_ok = shape_tot = 0
    center_errs = []
    confirmed = 0       # 확정(검정)으로 승격된 클러스터 수
    for cid, members in cluster_cells.items():
        true_ids = {int(truth_id[y][x]) for (x, y) in members if truth_id[y][x] >= 0}
        covered |= true_ids
        if len(true_ids) > 1:
            merged += 1
        est_shape = last_estimated_shape.get(cid)
        ri = obstacle_render_info.get(cid)
        if ri is not None and ri[5] == 1:
            confirmed += 1
        if est_shape is None or est_shape == "unknown" or ri is None:
            continue
        # 히트를 가장 많이 기여한 장애물을 이 클러스터의 정답으로 본다
        counts = {}
        for (x, y) in members:
            t = int(truth_id[y][x])
            if t >= 0:
                counts[t] = counts.get(t, 0) + 1
        if not counts:
            continue
        best_t = max(counts, key=counts.get)
        ox, oy = obstacle_ids[best_t]
        shape_tot += 1
        if len(true_ids) == 1 and est_shape == hidden_obstacles[(ox, oy)]["shape"]:
            shape_ok += 1
        center_errs.append(math.hypot(ri[1] - ox, ri[2] - oy))

    # 유령 장애물: 지도는 장애물(3)이라는데 실제로는 빈 칸.
    phantom = int(((grid == 3) & (truth_id < 0)).sum())

    # ---- 실물 소요 시간 추정  ★수정6 ----
    # 연속 모드는 sweep_during 이 실제로 누적한 값이다.
    # 정지 모드는 "광선마다 서보를 겨냥했다"는 가정으로 환산한다
    # (aim 200~900ms 의 중앙값 = STATIONARY_RAY_S). 이 둘의 차이가
    # 전시회 가능 여부를 가른다.
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
        "shape_acc": (shape_ok / shape_tot) if shape_tot else 0.0,
        "shape_judged": shape_tot,
        "obstacles": len(hidden_obstacles),
        "clusters": len(cluster_cells),
        "confirmed": confirmed,
        "merged": merged,
        "missed": len(hidden_obstacles) - len(covered),
        "center_err": (sum(center_errs) / len(center_errs)) if center_errs else float('nan'),
        "max_obs": max((cluster_obs_count(c) for c in cluster_cells), default=0),
    }


# =============================================================================
# =============================================================================
# 17. 메인 루프
# =============================================================================
# =============================================================================
#
# 한 스텝의 순서:
#
#   1) 진행방향 FOV 스캔                 -- 섹션 8
#   2) 가장 가까운 미탐색 조준 스캔      -- 섹션 8 + 13  ★수정5: 서보 제약 적용
#   3) 클러스터 재구성 (CLUSTER_EVERY)   -- 섹션 10
#   4) 형태 추정 갱신                    -- 섹션 11
#   5) 신뢰도 갱신 (시각화용)            -- 섹션 9
#   6) 종료 판정 (미탐색 0?)
#   7) 도달성 계산 + 목표 선정           -- 섹션 12 + 13
#   8) 이동 + 범퍼 검사                  -- 섹션 14 + 15
#
# 두 단계(phase)의 차이:
#   탐사(exploration) : 프런티어가 남아 있는 동안. 미탐색 경계로 계속 전진
#   cleanup           : 프런티어 소진 후. 관측 불가능한 곳을 핑크로 정리하며
#                       진행이 멈추면(STALL_LIMIT) 잔여를 통째로 포기

def _next_phase_after_survey():
    """탐사/정리가 끝난 뒤 어디로 갈지.  ★수정7
    플래그를 꺼두면 단계를 건너뛴다(A/B 비교용)."""
    if CONFIRM_PHASE:
        return "confirm"
    if RETURN_HOME:
        return "return"
    return "done"


def run(seed, visualize=True, verbose=True, pause=0.05):
    global robot_x, robot_y, robot_angle, step, collisions, grid
    global turns, turn_deg, moves
    global phase, return_path, confirm_cid, confirm_done, confirm_gained
    global returned

    reset_world(seed)
    viz = _Viz(seed) if visualize else None

    committed = None
    commit_steps = 0
    cleanup_phase = False
    last_progress = 0
    prev_rem = 10 ** 9
    reason = "완료"
    survey_note = ""
    moved_last_step = True      # ★수정6: 첫 스텝은 시동 스윕으로 처리
    confirm_visits = {}         # ★수정7: cid -> 재방문 시도 횟수
    confirm_start = 0           # 확정 단계 시작 스텝(예산 계산용)

    while True:
        step += 1
        if step > MAX_STEPS:
            # [버그8] 추정 도형이 줄어들면 미탐색 칸이 다시 생길 수 있어
            # 미탐색 수가 단조감소하지 않는다. 라이브락 방지용 상한.
            reason = f"MAX_STEPS({MAX_STEPS}) 초과"
            break

        # 1) 스캔
        #
        # ★수정6: 두 모델이 갈리는 지점.
        #   정지 일괄(예전): 여기서 헤딩 중심 FOV 를 통째로 훑는다.
        #   연속 스윕(실물): 스캔은 이동/회전 '중에' 이미 일어났다.
        #     여기서 추가로 할 일은 두 경우뿐 -
        #       (a) 첫 스텝: 지도가 비어 프런티어가 없다. 한 바퀴 돌려 시동
        #       (b) 직전 스텝에 못 움직였다: 가만있으면 지도가 안 변해 라이브락
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

        # 2) 가장 가까운 미탐색이 보이면 그쪽 조준 (화각 밖 누락 방지)
        #    ★수정5: 서보 범위를 검사하고, 필요하면 몸통을 돌린다.
        #    ★수정6: 연속 모드에서는 광선마다 겨냥하지 않고 제자리 한 바퀴 스윕.
        nu0 = find_nearest_unknown(grid, robot_x, robot_y)
        if nu0 is not None and math.hypot(nu0[0] - robot_x, nu0[1] - robot_y) <= SCAN_RANGE \
                and has_line_of_sight(grid, robot_x, robot_y, nu0[0], nu0[1]):
            aim0 = math.degrees(math.atan2(nu0[1] - robot_y, nu0[0] - robot_x))
            if CONTINUOUS_SCAN:
                aim_sweep(grid, aim0)
            else:
                scan_toward(grid, robot_x, robot_y, aim0)

        # 3) 관측 -> 클러스터 재구성 (정답 미참조)
        if step % CLUSTER_EVERY == 0 or step == 1:
            rebuild_clusters()

        # 4) 형태 채움 갱신: 관측이 충분히 늘어난 클러스터만
        for cid in list(cluster_cells.keys()):
            c = cluster_obs_count(cid)
            if c >= SHAPE_MIN_OBS and c - last_obs_count.get(cid, 0) >= 6:
                last_obs_count[cid] = c
                update_obstacle_shape(grid, cid)

        # 5) 신뢰도 갱신 (시각화 표시용)
        for cid in list(cluster_cells.keys()):
            if evaluate_confidence(cid) < 0.5:
                low_confidence_obstacles.add(cid)
            else:
                low_confidence_obstacles.discard(cid)

        # 6) 단계 전이 판정  ★수정7
        #
        # 예전에는 미탐색이 0 이 되면 그대로 끝냈다. 그런데 지도를 다 그렸다고
        # 임무가 끝난 게 아니다 - 그 지도 속 물체의 절반 가까이가 아직
        # '미확정'이다. 그래서 탐사/정리 뒤에 확정 순회를, 그 뒤에 복귀를 붙였다.
        if phase in ("explore", "cleanup") and int((grid == -1).sum()) == 0:
            phase = _next_phase_after_survey()
            confirm_start = step

        if phase == "done":
            break

        # 7) 도달성 계산 + 목표 선정
        reach = bfs_from(grid, robot_x, robot_y, robot_angle)
        frontiers = (find_frontiers(grid)
                     if phase in ("explore", "cleanup") else [])
        reachable = [f for f in frontiers
                     if f in reach and reach[f][1] > 0 and f not in unreachable_unknowns]

        target = path = None

        if phase == "explore" and reachable:
            # ---- 탐사 단계: 가장 싼 프런티어로 ----
            committed = None
            target = min(reachable, key=lambda f: reach[f][1])
            path = path_from(reach, target)

        elif phase in ("explore", "cleanup"):
            # ---- 정리(cleanup) 단계 ----
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

            # [버그7] 배치 포기 스캔은 매우 비싸다(미탐색 전수 x 반경 x LoS).
            # 주기적으로, 예산 안에서만 돌린다.
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
                # 목표 하나를 붙잡고(committed) 처리될 때까지 유지
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
                        # 여기서 보인다 -> 조준하고 그래도 안 보이면 포기
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
            # ---- 확정 순회 단계  ★수정7 ----
            #
            # 아직 확신 없는 클러스터를 골라, '아직 안 본 방위'로 찾아가
            # 한 번 더 본다. 이미 본 쪽에서 또 보면 관측 수만 늘고
            # viewpoint_spread 는 그대로라 확정으로 안 올라간다.
            pending = [c for c in cluster_cells
                       if needs_confirm(c)
                       and confirm_visits.get(c, 0) < CONFIRM_MAX_VISITS]

            if not pending or step - confirm_start > CONFIRM_STEP_BUDGET:
                confirm_cid = None
                phase = "return" if RETURN_HOME else "done"

            else:
                # 대상이 없거나 이미 해결됐으면 가장 가까운 것으로 새로 고른다
                if confirm_cid not in pending:
                    confirm_cid = min(
                        pending,
                        key=lambda c: math.hypot(
                            obstacle_render_info[c][1] - robot_x,
                            obstacle_render_info[c][2] - robot_y))
                cid = confirm_cid
                ri = obstacle_render_info.get(cid)
                if ri is None:
                    confirm_visits[cid] = CONFIRM_MAX_VISITS
                    confirm_cid = None
                else:
                    ccx, ccy = ri[1], ri[2]
                    bearing = viewpoint_gap_bearing(cluster_obs(cid), ccx, ccy)
                    op = find_confirm_point(grid, reach, ccx, ccy, bearing)

                    if op is None:
                        # 그 방위로 갈 자리가 없다. 원리적으로 못 보는 위치
                        # (아레나 모서리 등)라 더 시도해도 소용없다.
                        confirm_visits[cid] = CONFIRM_MAX_VISITS
                        confirm_cid = None
                    elif math.hypot(op[0] - robot_x, op[1] - robot_y) < 1.5:
                        # 도착 -> 조준 재관측
                        was_confirmed = (ri[5] == 1 and ri[0] != "unknown")
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
                        ri2 = obstacle_render_info.get(cid)
                        if (ri2 is not None and ri2[5] == 1
                                and ri2[0] != "unknown" and not was_confirmed):
                            confirm_gained += 1
                        confirm_cid = None
                    else:
                        target = op
                        path = path_from(reach, op)
                        if not path:
                            confirm_visits[cid] = CONFIRM_MAX_VISITS
                            confirm_cid = None

        elif phase == "return":
            # ---- 복귀 단계  ★수정7 ----
            # 실물에서는 투입 지점이 곧 회수 지점이다.
            if (robot_x, robot_y) == home:
                returned = True
                return_path = []
                reason = "복귀 완료"
                phase = "done"
            else:
                path = path_from(reach, home)
                if path is None:
                    # 추정 장애물(값 1)이 길을 막았을 수 있다.
                    # 복귀는 임무의 마지막이므로, 직접 관측(3)만 막는 느슨한
                    # 기준으로 한 번 더 풀어본다. 갇혀서 못 돌아오느니 낫다.
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
                    # 그리기용. 지금 위치부터 시작점까지 전체 경로.
                    return_path = [(robot_x, robot_y)] + list(path)

        # 8) 이동 (범퍼 검사 포함)
        moved_last_step = False
        if path:
            prev_x, prev_y = robot_x, robot_y
            nxt_x, nxt_y = path[0]
            hits = bump_cells(prev_x, prev_y, nxt_x, nxt_y)
            if hits:
                # 충돌: 이동 취소, 접촉 칸을 직접관측(3)으로 기록 후 재계획.
                collisions += 1
                for bx, by in hits:
                    grid[by][bx] = 3
                    cell_owner.pop((bx, by), None)
            else:
                dx, dy = nxt_x - prev_x, nxt_y - prev_y
                new_angle = math.degrees(math.atan2(dy, dx))
                diff = (new_angle - robot_angle + 180) % 360 - 180

                # ---- 회전 ----
                # ★수정6: 회전하는 동안에도 서보는 계속 돈다.
                #   90도 회전 1.2초면 광선 60개. 이동 1칸(16개)보다 많다.
                #   회전을 순수 손실로 보던 예전 모델과 해석이 달라진다.
                if abs(diff) > 5:
                    turns += 1
                    turn_deg += abs(diff)
                    if CONTINUOUS_SCAN:
                        sweep_while_turning(prev_x, prev_y, robot_angle, diff)
                    send_command({"cmd": "turn_left" if diff > 0 else "turn_right",
                                  "speed": 100, "duration_ms": int(abs(diff) * 10)})
                robot_angle = new_angle

                # ---- 직진 ----
                if CONTINUOUS_SCAN:
                    sweep_while_moving(prev_x, prev_y, nxt_x, nxt_y, new_angle)
                send_command({"cmd": "forward", "speed": 100, "duration_ms": 200})

                robot_x, robot_y = nxt_x, nxt_y
                moves += 1
                moved_last_step = True

        if viz:
            viz.progress(target, frontiers, phase != "explore")

    if USE_CONE_TRIM:
        # 시뮬레이션 종료. 마지막 다리가 끝까지 안 갔어도(방향 반전 전)
        # 안에 남은 채점 안 된 히트를 여기서 비워준다. 서보 물리 한계에서
        # 끝난 게 아니므로 end_is_limit=False - 그 끝은 보수적으로 자른다.
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
              f"| 형태정확 {result['shape_acc']:.2f}({result['shape_judged']}) "
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
#
# 표시 전용. 시뮬 로직에 영향을 주지 않는다.
# sim_pygame.py 가 이 클래스를 pygame 버전으로 교체한다(S._Viz = PygameViz).

class _Viz:

    # ---- 초기화 / 이벤트 ----------------------------------------------------

    def __init__(self, seed):
        import matplotlib.pyplot as plt
        from matplotlib.colors import ListedColormap
        self.plt = plt
        # [버그10] Pi 에는 Malgun Gothic 이 없다. 있는 것부터 골라 쓴다.
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
        """클릭한 칸을 기억해 _info_box 가 상세 정보를 띄운다."""
        if event.inaxes is None or event.xdata is None or event.ydata is None:
            return
        self.selected = (int(round(event.xdata)), int(round(event.ydata)))

    # ---- 정답 지도 (창 1개, 1회) -------------------------------------------

    def _show_truth(self, seed):
        """정답 지도. TRUTH_SMOOTH 면 탐사 지도와 같은 벡터 도형으로 그린다."""
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

    # ---- 추정 지도 구성요소 (매 스텝) --------------------------------------

    def _display_grid(self):
        """벡터 도형으로 그릴 장애물의 칸만 배경(0)으로 감춰 그 위에 매끈한
        patch 를 겹칠 자리를 만든다. 실제 grid 는 건드리지 않는다."""
        disp = grid.copy()
        for oid in obstacle_render_info:
            for x, y in last_filled.get(oid, ()):
                if disp[y][x] in (1, 2):
                    disp[y][x] = 0
        return disp

    def _patches(self):
        """추정 도형을 매끈한 벡터 도형으로 겹쳐 그린다."""
        from matplotlib.patches import Circle, Rectangle, Polygon
        ax = self.plt.gca()
        for (shape, cx, cy, half, rot_deg, value,
             draw_w, draw_h) in obstacle_render_info.values():
            color = 'black' if value == 1 else 'yellow'
            uncertain = (shape == "unknown")
            style = dict(facecolor=color, zorder=3)
            if uncertain:
                style.update(alpha=0.45, edgecolor='gray', linestyle='--', linewidth=1.2)
            else:
                style.update(edgecolor='none')
            if shape == "circle":
                p = Circle((cx, cy), radius=half, **style)
            elif shape == "triangle":
                rad = math.radians(rot_deg)
                c, s = math.cos(rad), math.sin(rad)
                pts = [(px * c - py * s + cx, px * s + py * c + cy)
                       for px, py in [(0, half), (-half, -half), (half, -half)]]
                p = Polygon(pts, closed=True, **style)
            else:
                w, h = draw_w * 2, draw_h * 2
                p = Rectangle((cx - w / 2, cy - h / 2), w, h,
                              angle=(0.0 if uncertain else rot_deg),
                              rotation_point='center', **style)
            ax.add_patch(p)

    def _info_box(self):
        """클릭한 칸의 상세 정보."""
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
                txt = (f"위치 : ({sx}, {sy})\n상태: 장애물({status})\n"
                       f"클러스터 #{cid} (히트셀 {len(cluster_cells.get(cid, ()))}개)\n"
                       f"추정형태: {last_estimated_shape.get(cid, '미정')}\n"
                       f"관측횟수: {len(obs)} (관측지점 {nvp}곳)\n"
                       f"광선각 커버리지: {ray_sp:.0f}도\n"
                       f"관측지점 커버리지: {vp_sp:.0f}도 (확정 기준 {CONFIRM_SPREAD}도)\n"
                       f"신뢰도: {evaluate_confidence(cid):.1f}")
            else:
                txt = f"위치 : ({sx}, {sy})\n상태: 장애물({status})\n관측횟수: 기록없음"
        self.plt.gcf().text(0.02, 0.98, txt, fontsize=10, verticalalignment='top',
                            bbox=dict(boxstyle='round', facecolor='white',
                                      edgecolor='black'))

    # ---- 프레임 그리기 -----------------------------------------------------

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
# [수정5] 서보 제약 A/B - 이게 이번 변경의 핵심 실험이다:
#   py path_planner_sim2.py --seeds 6 --headless                     서보 제약 O
#   py path_planner_sim2.py --seeds 6 --headless --no-servo-limit    예전 동작
#   -> 회전(특히 조준 회전)과 스텝이 얼마나 늘어나는지 = 예전 수치가
#      얼마나 낙관적이었는지의 정량화
#
#   py path_planner_sim2.py --seeds 6 --headless --no-turn-to-aim
#   -> 회전 못 하는 로봇의 한계 (핑크가 급증할 것)
#
# [수정9] 원뿔 절단 A/B:
#   py path_planner_sim2.py --seeds 6 --headless                 절단 O (기본)
#   py path_planner_sim2.py --seeds 6 --headless --no-cone-trim  절단 X (예전)
#   -> 반폭 오차/IoU 개선폭 재현 확인용

def set_scan_range(v):
    """SCAN_RANGE 를 런타임에 바꾼다.

    ★ 그냥 globals()['SCAN_RANGE'] = v 로는 안 된다.
      real_scan / has_line_of_sight / can_observe / find_observation_point 는
      max_range=SCAN_RANGE 를 '기본 인자'로 받는데, 기본 인자는 함수 정의
      시점에 한 번만 평가되어 함수 객체에 박힌다. 전역을 바꿔도 안 따라온다.
      __defaults__ 를 직접 갈아끼워야 CLI 플래그가 실제로 먹는다.

      이걸 모르고 --scan-range 플래그만 만들면, 값이 바뀐 것처럼 보이는데
      실제 스캔은 예전 사거리로 도는 조용한 버그가 된다."""
    globals()['SCAN_RANGE'] = v
    for fn in (real_scan, has_line_of_sight, can_observe, find_observation_point):
        d = list(fn.__defaults__)
        d[-1] = v                      # max_range 가 마지막 기본 인자
        fn.__defaults__ = tuple(d)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--seed', type=int, default=14)
    ap.add_argument('--seeds', type=int, default=0,
                    help='N>0 이면 seed..seed+N-1 을 헤드리스로 배치 실행')
    ap.add_argument('--headless', action='store_true')
    ap.add_argument('--min-sep', type=int, default=None,
                    help='장애물 사이 최소 간격(칸). 0 이면 서로 겹치게 배치돼 '
                         '분리 불가능한 덩어리가 생긴다(난이도 상승).')
    ap.add_argument('--los-block-confirmed', dest='los', action='store_true',
                    default=None,
                    help='가시선이 확정 장애물(1)도 막게 한다')
    ap.add_argument('--no-los-block-confirmed', dest='los', action='store_false',
                    help='가시선이 직접관측(3)만 막게 한다(예전 동작)')
    ap.add_argument('--no-logodds', action='store_true',
                    help='점유격자를 예전 하드 값 방식으로 (수정1 끄기)')
    ap.add_argument('--no-safe-block', action='store_true',
                    help='추정 확정(1)을 통행 가능으로 (수정3 끄기)')
    ap.add_argument('--turn-cost', type=float, default=None,
                    help='90도 회전당 비용. 0 이면 예전 동작 (수정4 끄기)')
    ap.add_argument('--false-hit', type=float, default=None,
                    help='ToF 오측률(0~1). 예: 0.01 -> 판독 100회당 1회 헛것')
    ap.add_argument('--miss', type=float, default=None,
                    help='장애물 미검출률(0~1)')
    ap.add_argument('--shape-v1', action='store_true',
                    help='형태 분류를 v1 규칙으로 (원->삼각형 오판 재현용)')
    ap.add_argument('--shape-rule', choices=['v1', 'v2', 'v3'], default=None,
                    help='형태 분류 규칙. 기본 v3(회전 대응)')
    ap.add_argument('--no-hold-small', action='store_true',
                    help='작은 물체도 원/사각형 판정 강행 (수정8 일부 끄기)')
    ap.add_argument('--vp-spread', type=float, default=None,
                    help='형태 판정에 요구할 최소 관측지점 커버리지(도)')
    # ---- 수정5 ----
    ap.add_argument('--servo-off', type=float, default=None,
                    help='서보 가동범위(중심 기준 ±도). 기본 65. '
                         'CAMERA_FOV 도 자동으로 2배로 맞춰진다.')
    ap.add_argument('--no-servo-limit', action='store_true',
                    help='서보 제약 해제 = 예전 동작(어느 방향이든 공짜 스캔)')
    ap.add_argument('--no-turn-to-aim', action='store_true',
                    help='범위 밖은 몸통을 안 돌리고 포기 (회전 못 하는 로봇)')
    ap.add_argument('--scan-range', type=int, default=None,
                    help='ToF 사거리(칸). 기본 36 = DIST_MAX_MM 1800 / 셀 50mm')
    ap.add_argument('--stationary-scan', action='store_true',
                    help='예전 정지 일괄 스캔으로 (수정6 끄기). '
                         '실물 소요 시간이 15배로 늘어난다')
    ap.add_argument('--robot-speed', type=float, default=None,
                    help='로봇 속도(m/s). 기본 0.15 (추정값)')
    ap.add_argument('--no-confirm', action='store_true',
                    help='확정 순회 단계 끄기 (수정7 일부 끄기)')
    ap.add_argument('--no-return', action='store_true',
                    help='시작점 복귀 끄기')
    # ---- 수정9 ----
    ap.add_argument('--no-cone-trim', action='store_true',
                    help='ToF 원뿔 각도 절단 끄기 (실측 이전 동작, A/B용)')
    ap.add_argument('--cone-trim-deg', type=float, default=None,
                    help='원뿔 절단량(도). 기본 12.5(원뿔 반각). '
                         '0/10/12.5/20/25 로 실측 검증됨(12.5가 최적)')
    ap.add_argument('--no-cone-model', action='store_true',
                    help='ToF 원뿔 물리모델 끄기 = 폭 0도 레이저 (예전 동작). '
                         '이걸 끄고 절단만 켜면 존재하지 않는 문제를 보정하려다 '
                         '오히려 나빠진다 - 실측으로 확인됨(IoU 0.98->0.79)')
    ap.add_argument('--cone-subrays', type=int, default=None,
                    help='원뿔 안 서브 광선 표본 개수. 기본 9')
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
    if args.shape_v1:
        globals()['SHAPE_RULE'] = "v1"
    if args.shape_rule is not None:
        globals()['SHAPE_RULE'] = args.shape_rule
    if args.no_hold_small:
        globals()['SHAPE_HOLD_SMALL_ROUND'] = False
    if args.vp_spread is not None:
        globals()['SHAPE_MIN_VP_SPREAD'] = args.vp_spread
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

    # ---- 수정5: 서보 범위를 바꾸면 FOV 도 같이 바꿔야 앞뒤가 맞는다 ----
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
            print(f"{len(results)}개 시드 요약 (MIN_SEP={OBSTACLE_MIN_SEP}, "
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
            print(f"  IoU {avg('iou'):.3f} | 형태정확 {avg('shape_acc'):.3f} "
                  f"| 중심오차 {avg('center_err'):.2f}칸")
        return

    try:
        run(args.seed, visualize=not args.headless)
    except KeyboardInterrupt:
        print("\n중단됨")


if __name__ == '__main__':
    main()
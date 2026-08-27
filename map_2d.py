"""
map_2d.py -- 실물 로봇 지도작성/주행 오케스트레이터 (PySide6)

[존재 이유]
    tof_commander.py / vision_object.py 가 몇 년째 "map_2D 가 소비한다" 고
    적어두기만 하고 실제 파일이 없었다(route_message 의 sensor_queue 를
    받아갈 곳이 없어서 그 스트림이 갈 곳을 잃고 있었다). 이 파일이 그
    소비자다 - 이름도 그 문서화된 이름(map_2D)을 그대로 따랐다.

    이전에 PySide 로 만들었던 버전이 있었으나(사용자 확인) 지금 저장소엔
    없어서 이 구조에 맞춰 새로 짰다.

[핵심 설계 결정 - 왜 이런 모양인가]
    1. path_planner_sim2.py 를 고치지 않는다. 그 파일은 오늘 스무딩을
       A/B 검증했고 대시보드(시뮬레이션)도 그대로 쓴다 - 실물용으로
       고치면 둘 다 위험해진다. 대신 "계획" 관련 함수(bfs_from/
       path_from/smooth_next/observe_cell/record_hit/find_frontiers)만
       라이브러리로 가져다 쓴다. sim_bridge.py 가 이미 이 모듈의 전역
       (S.grid, S.robot_x 등)을 직접 읽고 쓰는 선례가 있어 - 같은 패턴.
       ★real_scan() 자체는 안 쓴다 - 그 함수는 가짜 진실(truth_id)에 대고
       레이캐스팅하는 시뮬레이션 전용이다. 그 자리를 아래 process_stream_
       sample() 이 대신한다(실측값으로 레이를 걷는 로직만 재사용).

    2. 로봇 위치(robot_x/y)는 열린 루프 추정이다 - ESP32 가 위치를
       안 보낸다(엔코더 미완성, route_message 의 코멘트 참고). 그런데
       시뮬레이션도 원래 "명령을 내린 쪽이 위치를 스스로 갱신"하는
       방식이었으므로 같은 방식을 그대로 쓴다 - 새 메커니즘이 아니다.
       ★헤딩(각도)는 다르다 - ESP32 자이로가 실측값을 주므로 열린 루프로
       추정하지 않고 스트림의 yaw 를 그대로 쓴다.
       ★엔코더가 생기면 고칠 곳은 Odometry.on_forward_command() 안
       '추정' 부분 하나뿐이다 - 그 함수 docstring 에 표시해 뒀다.

    3. ESP32(센서 스트림 + 모터 명령)와 라즈베리파이(카메라)를 둘 다
       받는다 - 통신 구조 절 CLAUDE.md 문서와 동일한 토폴로지.
       ESP32  : socket_receiver.Esp32Receiver(스트림 수신) +
                motor_commander(명령 송신)
       라즈베리파이 : FrameGrabber(dashboard.py 사본, PySide 앱 전체를
                끌어오지 않으려고 최소 형태로 독립 보유 - 이 저장소의
                기존 관례, camera_test.py/camera_test_opencv.py 도 각자
                Esp32Receiver 류를 독립적으로 들고 있다)

    4. 안전 기본값: 모터 명령 송신은 기본 꺼짐이다(센싱/지도작성만
       먼저 확인). GUI 의 "자율주행" 체크박스를 눌러야 실제로 나간다.
       처음 실물에 붙이는 단계라 검증 안 된 상태로 바로 움직이게 하고
       싶지 않았다.

[2026-08-27 우선순위 갱신 - 사용자 확정]
    "장애물 회피 주행"보다 "장애물 탐지(지도작성)" 를 먼저 하드웨어로
    검증하기로 했다 - 주행 로직은 나중에 손본다. 그래서:
      ★카메라 정밀화(obstacle_detector.detect_obstacle) 자동 트리거를
        이번에 실제로 걸었다(아래 5-1절, _maybe_refine_with_camera).
      ★ESP32 쪽은 안 고쳤다 - rescue_sensor_node.ino 가 이미 sweep/aim
        을 지원하므로 이 테스트엔 그걸로 충분하다. 이동제어 펌웨어
        (main_base.ino) 는 이번 테스트에 아예 필요 없다(모터를 안 씀).
      계획/주행(plan_next_target/drive_one_step, 자율주행 체크박스)은
      그대로 뒀다 - 기본 꺼짐이라 이 테스트에 지장 없다. 주행 로직
      자체를 다듬는 건 이후 별도 작업.

[v1 스코프 - 여전히 일부러 안 넣은 것]
    - 확정(confirm)/복귀(return) 단계, 요구조자 발견, 클러스터 형태추정
      (rebuild_clusters/update_obstacle_shape) - path_planner_sim2.run()
      의 전체 미션 상태기계를 그대로 옮기면 이 파일 하나가 그 정도
      규모(2200줄)가 된다.
    - 장애물 회피를 반영한 정교한 재계획 - 위 우선순위 갱신 참고, 이번
      스코프가 아니다.

[실행]
    python map_2d.py                       # robot_config 의 IP 사용
    python map_2d.py --esp32-ip 192.168.1.50 --pi-ip 192.168.1.60
    python map_2d.py --no-hardware         # GUI 만 띄우고 연결 시도 안 함(레이아웃 확인용)
"""

import argparse
import math
import sys
import threading
import time
from queue import Queue, Empty

import cv2
import numpy as np
from PySide6.QtCore import Qt, QTimer
from PySide6.QtGui import QImage, QPixmap
from PySide6.QtWidgets import (QApplication, QCheckBox, QHBoxLayout, QLabel,
                               QMainWindow, QPushButton, QVBoxLayout, QWidget)

import robot_config as C
import path_planner_sim2 as S          # ★라이브러리로만 쓴다 - 이 모듈은 안 고침
from path_planner import find_frontiers
from map_canvas import MapCanvas
from socket_receiver import Esp32Receiver
from tof_commander import TofCommander, route_message
import motor_commander as MC
import obstacle_detector


# =============================================================================
# 1. 카메라 프레임 수신 (dashboard.py 의 FrameGrabber 사본)
#
# ★dashboard.py 전체를 import 하지 않는다 - 그 파일은 자기 완결적인 PySide
#   앱이라 부작용(모듈 상단 상수, GRID_N 등)을 끌고 올 이유가 없다. 이
#   저장소는 이미 각 스크립트가 필요한 작은 클래스를 독립적으로 복사해
#   쓰는 관례가 있다(camera_test.py/camera_test_opencv.py 도 서로 Esp32
#   Receiver 류를 각자 보유 - 그쪽 docstring 참고).
# =============================================================================
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
        if self.t.is_alive():
            self.t.join(timeout=2.0)
        if self.cap.isOpened():
            self.cap.release()


# =============================================================================
# 2. 위치 추정 (Odometry)
#
# ★엔코더가 생기면 여기 한 곳만 고치면 된다 - on_forward_command() 의
#   "추정" 표시 부분. 나머지(헤딩 갱신, 좌표계, 소비하는 쪽)는 안 바뀐다.
# =============================================================================
class Odometry:
    """열린 루프 위치 + 실측 헤딩.

    시뮬레이션(path_planner_sim2.run())도 원래 로봇 위치를 '측정'한 적이
    없다 - "이 명령을 내렸으니 여기 있겠지" 로 자기 자신을 갱신해 온
    것뿐이다(robot_x, robot_y = nxt_x, nxt_y). 실물도 같은 방식이다:
    ESP32 는 위치를 안 보내므로(엔코더 미완성 - route_message 참고),
    명령을 내린 이 프로세스가 계속 스스로 추정한다.

    헤딩만은 다르다 - ESP32 자이로가 실측값을 스트림으로 주므로 그걸
    그대로 쓴다(추정 안 함)."""

    def __init__(self, x=0.0, y=0.0, angle_deg=0.0):
        self.x = x
        self.y = y
        self.angle_deg = angle_deg     # 갱신은 update_heading() 으로만
        self._lock = threading.Lock()

    def update_heading(self, yaw_raw):
        """스트림의 raw yaw(자이로) -> 지도 기준 헤딩. 실측값, 추정 아님."""
        with self._lock:
            self.angle_deg = C.gyro_to_heading(yaw_raw)

    def on_forward_command(self, distance_mm):
        """★엔코더 붙이면 여기를 실측값(틱 적분)으로 바꿀 것. 지금은
        '이만큼 가라고 시켰으니 갔겠지' 추정이다 - 실제로 밀리거나
        슬립하면 이 추정은 조용히 틀어진다(odometry_plan.md §3 참고)."""
        with self._lock:
            rad = math.radians(self.angle_deg)
            dist_cells = C.mm_to_cells(distance_mm)
            self.x += dist_cells * math.cos(rad)
            self.y += dist_cells * math.sin(rad)

    def snapshot(self):
        with self._lock:
            return self.x, self.y, self.angle_deg


# =============================================================================
# 3. 지도 상태 초기화
#
# ★reset_world() 를 안 쓴다 - 그 함수는 가짜 장애물(truth_id)까지 만든다.
#   여기서는 관측(observe_cell/record_hit)이 실제로 필요로 하는 전역만
#   골라서 만든다. 이 함수들이 어떤 전역을 읽는지는 path_planner_sim2.py
#   본문에서 확인했다(observe_cell -> logodds, record_hit -> hit_obs).
# =============================================================================
def init_grid_state():
    S.grid = np.full((S.GRID_SIZE, S.GRID_SIZE), -1, dtype=np.int64)
    S.logodds = np.zeros((S.GRID_SIZE, S.GRID_SIZE), dtype=np.float64)
    S.hit_obs = {}
    S.cluster_cells = {}
    S.cell_owner = {}
    S._next_cid = 0
    # rescuee_truths 는 시뮬 전용(정답 위치) - real_scan()/_check_rescuee_
    # detection() 을 안 쓰므로 빈 리스트로 둬도 안전하다. 요구조자 발견은
    # v1 스코프 밖(위 docstring 참고).
    S.rescuee_truths = []


# =============================================================================
# 4. 센서 스트림 -> 지도 반영
#
# real_scan() 의 레이워크 부분(free 칸 관측 -> observe_cell(False), 히트칸
# -> observe_cell(True))만 재사용한다. real_scan() 자체는 가짜 진실에
# 대고 거리를 '만들어내는' 함수라 못 쓴다 - 여기선 거리가 이미 ESP32가
# 잰 실측값(또는 카메라로 정밀화된 값, 아래 5-1절)으로 주어진다.
# =============================================================================
def apply_hit(odom: Odometry, servo_off_deg, dist_mm):
    """'로봇기준 각도 + 거리(mm)' 하나를 grid 에 반영하는 공용 경로.

    두 군데가 이 함수로 모인다:
      - process_stream_sample() : ToF 원시 스트림 그대로
      - _maybe_refine_with_camera() (worker) : 카메라로 정밀화된 각도/거리
        (vision_object 문서 표현으로 "ToF 보다 80배 정확한" 각도)
    ★단위 주의: record_hit() 은 sim 관례상 거리를 '칸' 단위로 받는다
      (_cone_min_distance 등이 칸 단위 max_range 로 동작하므로) - mm 을
      여기서 칸으로 바꿔서 넘긴다."""
    rx, ry, heading = odom.snapshot()
    world_angle = heading + servo_off_deg

    dist_cells = C.mm_to_cells(dist_mm)
    ar = math.radians(world_angle)
    ca, sa = math.cos(ar), math.sin(ar)

    walk_n = max(1, int(dist_cells))
    for i in range(walk_n):
        x = math.floor(rx + i * ca)
        y = math.floor(ry + i * sa)
        if not (0 <= x < S.GRID_SIZE and 0 <= y < S.GRID_SIZE):
            return
        S.observe_cell(S.grid, x, y, False)

    hx = math.floor(rx + dist_cells * ca)
    hy = math.floor(ry + dist_cells * sa)
    if not (0 <= hx < S.GRID_SIZE and 0 <= hy < S.GRID_SIZE):
        return
    S.observe_cell(S.grid, hx, hy, True)
    S.record_hit((hx, hy), world_angle, dist_cells, rx, ry)


def process_stream_sample(msg, odom: Odometry):
    """route_message() 가 만든 dict 하나 -> grid 갱신(apply_hit 위임).

    msg 필드(tof_commander.route_message 참고):
      distance_mm, angle_deg(서보 오프셋), valid, gas, robot_angle(=yaw 원본)
    """
    if msg.get('robot_angle') is not None:
        odom.update_heading(msg['robot_angle'])

    if not msg.get('valid'):
        # ToF 무효값(범위 밖/에러) - 자유공간으로도 단정하지 않는다. 잘못된
        # '자유' 판정이 실제로 있는 장애물을 지워버리는 쪽이 그 반대보다
        # 더 위험하다(로봇이 충돌한다) - 보수적으로 그냥 버린다.
        return

    dist_mm = msg.get('distance_mm', 0.0)
    if not (C.DIST_MIN_MM <= dist_mm <= C.DIST_MAX_MM):
        return

    apply_hit(odom, msg.get('angle_deg', 0.0), dist_mm)


# =============================================================================
# 4-1. 카메라 정밀화 (★2026-08-27 부터 실제로 자동 트리거함)
#
# obstacle_detector.detect_obstacle() 가 하는 일(각도는 카메라로, 거리는
# 재겨냥한 ToF 로) 을 ToF 스윕 히트마다 걸면 카메라/ToF/Canny 를 20ms 마다
# 두드리게 된다 - 너무 잦다. 그래서 "이 각도 구간, 최근에 이미 확인했나"
# 를 버킷+쿨다운으로 걸러서 새 방향에서 히트가 잡힐 때만 정밀화한다.
# =============================================================================
REFINE_BUCKET_DEG = 5.0     # ESP32 스윕 스텝(5도)과 맞춤 - 그보다 세밀해도 의미 없음
REFINE_COOLDOWN_S = 3.0     # 같은 방향을 이 시간 안엔 다시 정밀화 안 함


def maybe_refine_with_camera(worker, msg):
    """새로운 방향에서 유효한 ToF 히트가 잡히면 카메라로 정밀화해서
    (더 정확한 각도로) apply_hit() 을 다시 부른다. 성공하면 worker.detections
    에 결과를 쌓는다(GUI 표시용) - 실패해도 조용히 넘어간다(원시 ToF
    관측은 process_stream_sample() 이 이미 반영했으므로 지도 자체는
    안전하다, 이건 '정밀화 보너스'일 뿐)."""
    if worker.cam is None or worker.tof is None:
        return
    if not msg.get('valid'):
        return
    dist_mm = msg.get('distance_mm', 0.0)
    if not (C.DIST_MIN_MM <= dist_mm <= C.DIST_MAX_MM):
        return

    servo_off = msg.get('angle_deg', 0.0)
    bucket = round(servo_off / REFINE_BUCKET_DEG) * REFINE_BUCKET_DEG
    now = time.monotonic()
    if now - worker.refined_buckets.get(bucket, 0.0) < REFINE_COOLDOWN_S:
        return

    frame = worker.cam.read()
    if frame is None:
        return
    worker.refined_buckets[bucket] = now   # 실패해도 쿨다운은 소모(카메라 없는 방향 재시도 방지)

    try:
        res = obstacle_detector.detect_obstacle(
            frame, tof_angle_deg=servo_off, servo_angle_deg=servo_off,
            tof=worker.tof)
    except Exception as e:                                # noqa: BLE001
        worker.set_status("pi", f"정밀화 오류: {e}")
        return

    if not res.get("ok"):
        return

    apply_hit(worker.odom, res["angle_deg"], res["distance_mm"])
    with worker._status_lock:
        worker.detections.append(res)
        worker.detections = worker.detections[-20:]   # 최근 20개만 유지


# =============================================================================
# 5. 계획 + 주행 한 스텝
#
# bfs_from/path_from/smooth_next 는 전부 명시적 인자를 받는 순수함수라
# (모듈 전역 robot_x/y 를 안 읽는다 - path_planner_sim2.py 본문에서 확인),
# 여기서 만든 Odometry 값을 그대로 넘기면 된다. S.run() 을 통째로 옮겨오지
# 않고 이 부분만 재사용하는 이유가 이것 - 결합이 이 세 함수 선에서 끊긴다.
# =============================================================================
def plan_next_target(odom: Odometry):
    """탐사할 다음 목표(프런티어)를 고른다. path_planner_sim2 의 전체
    커밋/미확정 로직(v1 스코프 밖)보다 훨씬 단순하다 - 도달 가능한
    프런티어 중 이동비용이 가장 싼 곳."""
    rx, ry, heading = odom.snapshot()
    isx, isy = int(rx), int(ry)     # bfs_from 은 배열 인덱스라 int 필요
    reach = S.bfs_from(S.grid, isx, isy, heading)
    frontiers = find_frontiers(S.grid)
    candidates = [f for f in frontiers if f in reach]
    if not candidates:
        return None, reach
    target = min(candidates, key=lambda f: reach[f][1])
    return target, reach


def drive_one_step(odom: Odometry, target, reach, autonomy_enabled):
    """target 까지 경로를 잡고 스무딩된 다음 칸으로 실제 이동 명령을
    보낸다. autonomy_enabled=False 면 계획만 하고 명령은 안 보낸다
    (센싱/지도작성만 확인하는 안전 모드)."""
    path = S.path_from(reach, target)
    if not path:
        return
    rx, ry, heading = odom.snapshot()
    nxt = S.smooth_next(S.grid, rx, ry, path)
    nx, ny = nxt
    dx, dy = nx - rx, ny - ry
    dist_cells = math.hypot(dx, dy)
    if dist_cells < 0.05:
        return
    dist_mm = C.cells_to_mm(dist_cells)
    new_heading = math.degrees(math.atan2(dy, dx))
    turn_diff = (new_heading - heading + 180) % 360 - 180

    if not autonomy_enabled:
        return

    # ★회전은 고정 90도만 지원한다(main_base.ino) - CLAUDE.md "★모터 명령
    #   경로" 참고. 20도 넘게 틀어야 하면 90도 회전을 보내고, 그보다
    #   작으면 회전 없이 그냥 전진한다(약간의 각도 오차는 다음 스텝
    #   자이로 헤딩 갱신으로 자연히 보정됨 - 열린루프 위치만 그 오차를
    #   그대로 먹는다. 임의각 회전이 되기 전까진 이게 최선이다).
    if abs(turn_diff) > 45:
        MC.turn_left() if turn_diff > 0 else MC.turn_right()
        return   # 이번 스텝은 회전만. 다음 tick 에 다시 계획해서 전진
    MC.move_forward(dist_mm)
    odom.on_forward_command(dist_mm)


# =============================================================================
# 6. 백그라운드 워커 - 센서 소비 + (선택) 계획/주행
# =============================================================================
class RealWorldWorker(threading.Thread):
    def __init__(self, esp32_ip, pi_ip, use_hardware=True):
        super().__init__(daemon=True)
        self.esp32_ip = esp32_ip
        self.pi_ip = pi_ip
        self.use_hardware = use_hardware
        self.odom = Odometry()
        self.running = True
        self.autonomy_enabled = False
        self.status = {"esp32": "미연결", "pi": "미연결", "steps": 0}
        self._status_lock = threading.Lock()

        self.esp32 = None
        self.tof = None
        self.cam = None

        # ★TofCommander 는 응답을 소켓에서 직접 안 읽고 큐에서 꺼낸다
        #   (tof_commander.py 설계) - Esp32Receiver 가 스트림 소켓에서 읽은
        #   각 줄을 route_message() 로 이 두 큐에 나눠 담는다. sensor_queue
        #   는 지도 갱신(process_stream_sample), response_queue 는
        #   TofCommander.aim()/scan_range() 가 소비한다.
        self.sensor_queue = Queue()
        self.response_queue = Queue()

        # ★2026-08-27 우선순위: 장애물 탐지(카메라 정밀화)를 먼저 검증.
        self.refined_buckets = {}    # 각도버킷 -> 마지막 정밀화 시각(monotonic)
        self.detections = []         # obstacle_detector 성공 결과 최근 20개(GUI 표시용)

        self.plan_interval_s = 1.0
        self._last_plan = 0.0

    def set_status(self, key, val):
        with self._status_lock:
            self.status[key] = val

    def get_status(self):
        with self._status_lock:
            return dict(self.status)

    def run(self):
        init_grid_state()

        if self.use_hardware:
            self._connect_esp32()
            self._connect_camera()

        while self.running:
            self._drain_sensor_queue()

            now = time.monotonic()
            if now - self._last_plan >= self.plan_interval_s:
                self._last_plan = now
                self._plan_tick()

            time.sleep(0.02)

    def _connect_esp32(self):
        def _on_msg(msg):
            route_message(msg, self.sensor_queue, self.response_queue)

        try:
            self.esp32 = Esp32Receiver(verbose=False, on_message=_on_msg)
            self.esp32.start()
            self.set_status("esp32", f"스트림 대기중 (:{C.STREAM_PORT})")
        except Exception as e:                        # noqa: BLE001
            self.set_status("esp32", f"스트림 실패: {e}")

        try:
            self.tof = TofCommander(self.esp32_ip, cmd_port=C.CMD_PORT,
                                    stream_queue=self.response_queue)
            self.tof.connect(timeout=3.0)
            self.set_status("esp32", "연결됨")
        except Exception as e:                        # noqa: BLE001
            self.set_status("esp32", f"명령 채널 실패: {e}")
            self.tof = None

    def _connect_camera(self):
        url = f"tcp://{self.pi_ip}:{C.CAMERA_PORT}"
        try:
            self.cam = FrameGrabber(url)
            if self.cam.start():
                self.set_status("pi", "연결됨")
            else:
                self.set_status("pi", "연결 실패")
        except Exception as e:                        # noqa: BLE001
            self.set_status("pi", f"실패: {e}")

    def _drain_sensor_queue(self):
        # ★한 tick 에 너무 많이 처리하면 계획/렌더링이 밀린다 - 상한을 둔다.
        for _ in range(50):
            try:
                msg = self.sensor_queue.get_nowait()
            except Empty:
                break
            # 원시 ToF 반영을 항상 먼저 - 지도는 정밀화 성공 여부와
            # 무관하게 채워진다. 정밀화(아래)는 '보너스' 이자 성공하면
            # apply_hit() 을 한 번 더 불러 같은 칸을 더 정확한 값으로 덮는다.
            process_stream_sample(msg, self.odom)
            # ★tof.aim() 왕복(최대 수백ms~2초) 이 여기서 블로킹된다 - 버킷+
            #   쿨다운으로 드물게만 걸리므로(REFINE_COOLDOWN_S) 감수한다.
            #   장애물 탐지 검증이 이번 우선순위라 정밀도를 택함(2026-08-27).
            maybe_refine_with_camera(self, msg)

    def _plan_tick(self):
        target, reach = plan_next_target(self.odom)
        if target is None:
            return
        drive_one_step(self.odom, target, reach, self.autonomy_enabled)
        with self._status_lock:
            self.status["steps"] += 1

    def stop(self):
        self.running = False
        if self.cam is not None:
            self.cam.stop()
        if self.tof is not None:
            self.tof.close()
        if self.esp32 is not None:
            self.esp32.stop()


# =============================================================================
# 7. GUI
# =============================================================================
class MapWindow(QMainWindow):
    def __init__(self, worker: RealWorldWorker):
        super().__init__()
        self.worker = worker
        self.setWindowTitle("map_2d - 실물 지도작성")
        self.resize(900, 760)

        root = QWidget()
        layout = QVBoxLayout(root)

        top = QHBoxLayout()
        self.lbl_esp32 = QLabel("ESP32: -")
        self.lbl_pi = QLabel("Pi: -")
        self.lbl_pos = QLabel("위치: -")
        self.chk_autonomy = QCheckBox("자율주행 (모터 명령 전송)")
        self.chk_autonomy.stateChanged.connect(self._on_autonomy_toggled)
        top.addWidget(self.lbl_esp32)
        top.addWidget(self.lbl_pi)
        top.addWidget(self.lbl_pos)
        top.addStretch(1)
        top.addWidget(self.chk_autonomy)
        layout.addLayout(top)

        self.canvas = MapCanvas(cell_m=C.CELL_SIZE)
        layout.addWidget(self.canvas, 1)

        cam_row = QHBoxLayout()
        self.cam_label = QLabel("카메라: 미연결")
        self.cam_label.setFixedHeight(160)
        self.cam_label.setAlignment(Qt.AlignCenter)
        cam_row.addWidget(self.cam_label, 2)

        # ★2026-08-27 - 장애물 탐지(카메라 정밀화) 검증용 패널. 이번
        #   우선순위(회피 주행보다 탐지 먼저)에 맞춰 추가했다 - 정밀화가
        #   실제로 잡히고 있는지 눈으로 바로 확인하려는 용도.
        self.detect_label = QLabel("감지된 장애물: -")
        self.detect_label.setFixedHeight(160)
        self.detect_label.setAlignment(Qt.AlignTop | Qt.AlignLeft)
        self.detect_label.setWordWrap(True)
        cam_row.addWidget(self.detect_label, 1)
        layout.addLayout(cam_row)

        self.setCentralWidget(root)

        self.timer = QTimer(self)
        self.timer.timeout.connect(self._refresh)
        self.timer.start(150)

    def _on_autonomy_toggled(self, state):
        self.worker.autonomy_enabled = bool(state)

    def _refresh(self):
        st = self.worker.get_status()
        self.lbl_esp32.setText(f"ESP32: {st['esp32']}")
        self.lbl_pi.setText(f"Pi: {st['pi']}  (스텝 {st['steps']})")

        rx, ry, heading = self.worker.odom.snapshot()
        self.lbl_pos.setText(f"위치: ({rx:.1f}, {ry:.1f})  헤딩 {heading:.1f}도")

        if S.grid is not None:
            self.canvas.set_data(S.grid, robot=(rx, ry), heading=heading)

        if self.worker.cam is not None:
            frame = self.worker.cam.read()
            if frame is not None:
                h, w = frame.shape[:2]
                img = QImage(frame.data, w, h, frame.strides[0],
                             QImage.Format_BGR888)
                self.cam_label.setPixmap(
                    QPixmap.fromImage(img).scaledToHeight(
                        self.cam_label.height(), Qt.SmoothTransformation))

        with self.worker._status_lock:
            recent = list(self.worker.detections[-6:])
        if recent:
            lines = [
                f"각도{d['angle_deg']:+.1f}° 거리{d['distance_mm']}mm "
                f"폭{d['width_mm']:.0f}mm {d['shape']}"
                for d in reversed(recent)
            ]
            self.detect_label.setText("감지된 장애물:\n" + "\n".join(lines))
        else:
            self.detect_label.setText("감지된 장애물: 아직 없음")

    def closeEvent(self, ev):
        self.worker.stop()
        super().closeEvent(ev)


# =============================================================================
def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--esp32-ip', default=C.ESP32_IP)
    ap.add_argument('--pi-ip', default=C.PI_IP)
    ap.add_argument('--no-hardware', action='store_true',
                    help='연결 시도 없이 GUI 레이아웃만 확인')
    args = ap.parse_args()

    if not S.grid is None:
        pass  # (grid 는 worker.run() 시작 시 init_grid_state() 로 채워짐)

    worker = RealWorldWorker(args.esp32_ip, args.pi_ip,
                             use_hardware=not args.no_hardware)
    worker.start()

    app = QApplication(sys.argv)
    win = MapWindow(worker)
    win.show()
    ret = app.exec()
    worker.stop()
    sys.exit(ret)


if __name__ == '__main__':
    main()

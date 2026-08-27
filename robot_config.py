#캘리브레이션 상수 중앙화 — FOCAL_PX=281, DIST_MAX_MM, 서보 범위, HFOV 통일
"""
robot_config.py -- 보정값 한 곳 모음

같은 상수가 파일마다 흩어져 있으면, 실측해서 값을 바꿀 때 한 군데를 빠뜨려
파일별로 각도가 달라지는 사고가 난다. 여기 한 곳만 고치면 전부 반영되게 한다.

[사용법]
    import robot_config as C
    ang = C.pixel_to_angle(px)          # 직접 atan 쓰지 말고 함수 경유

[ESP32 값은 여기 없다]
    SERVO_CENTER / SERVO_SIGN 등은 펌웨어에 있어야 하므로 .ino 에서 고친다.
    아래에 현재값 사본을 두어 양쪽이 어긋나지 않게 관리한다.
"""

import math

# ===============================
# 카메라  (2026-08 실측 완료)
# ===============================
# [실측 절차]
#   A4 세로배치(가로 210mm)를 1000mm 거리 벽에 붙이고 촬영 → 픽셀폭 측정
#       f = 픽셀폭 * 거리 / 실제폭
#
# [측정 결과]
#   640x480 (4:3, 기본)               119px → f 567
#   640x360 (16:9, --mode 2304:1296)   59px → f 281   ← 채택
#
# ★★ 라파 스트리밍 명령을 반드시 아래로 맞출 것. 4:3으로 찍으면
#    libcamera가 1536x864 센서모드(중앙 3072x1728만 쓰는 크롭모드)를 고르고
#    거기서 4:3으로 좌우를 더 잘라내 시야가 정확히 절반(2.02배)이 된다.
#    imx708_wide 를 표준렌즈처럼 쓰게 된다.
#
#      rpicam-vid -t 0 --mode 2304:1296 --width 640 --height 360 \
#                 --inline --listen -o tcp://0.0.0.0:8888
#
IMG_W = 640
IMG_H = 360             # ★ 16:9. 480 아님.
FOCAL_PX = 281.0        # 실측. 해상도/센서모드 바뀌면 재측정 필수.
CX = IMG_W / 2.0        # 320.0
CY = IMG_H / 2.0        # 180.0

# ── 위험 임계값 (§8-3 단일 출처화) ──────────────────────────────────
# 맨몸 인간 내성 기준. 습도 낮은 공기는 짧게면 고온도 버티지만(사우나
# 80~100도/10~20분, 탈출+휴식 전제), 산업 열스트레스 기준(WBGT)은 습도
# 반영 시 32~34도부터 위험 취급한다. 요구조자는 스스로 못 움직일 수
# 있어 그보다 보수적으로 잡았다.
FIRE_TEMP_C = 50.0      # 이 온도(°C) 이상 = 화재 위험 (지도표시+경로회피+카드위험)
GAS_PPM = 40.0          # 이 농도(ppm) 이상 = 가스 위험 (지도표시+경로회피+카드위험)

TEMP_WARN_C = 40.0      # 카드 "경계(주의)" 조기경고선
GAS_WARN_PPM = 30.0

# 참고용 화각. 계산에 쓰지 말 것.
# 배럴 왜곡 때문에 실제 시야는 이보다 넓다(사양 대각 120도 → 16:9 가로 ~113도).
# "픽셀↔각도 변환"에는 FOCAL_PX 를, "로봇이 볼 수 있는 범위"에는 CAMERA_VIEW_DEG 를
# 쓴다. 둘이 다른 물리량이므로 값이 다른 게 정상이다.
CAMERA_VIEW_DEG = 113.0   # TODO: 벽 눈금으로 실측 확인

# ===============================
# 카메라 - ToF 광축 오프셋  (조립 후 실측)
# ===============================
# 카메라와 ToF 가 같은 서보에 달려 있어도 광축이 완전히 평행하진 않다.
# "화면 정중앙(0도)에 물체를 놓았을 때 ToF 가 실제로 몇 도를 보는가" 의 차이.
#
# [실측 방법]
#  1. 가는 막대(ToF 원뿔보다 얇은 것)를 로봇 앞에 세운다
#  2. 화면 정중앙에 오도록 서보를 맞춘다
#  3. ToF 거리를 본다
#       - 막대 거리가 나오면 -> 오프셋 ~ 0
#       - 배경 거리가 나오면 -> 어긋남
#  4. 서보를 조금씩 돌려 막대 거리가 안정적으로 나오는 각도를 찾는다
#  5. 그 각도가 아래 값
#
# 2도면 1m 에서 35mm, 5도면 87mm 빗나간다.
CAM_TOF_OFFSET_DEG = 0.0    # TODO: 미측정

# 겨냥 목표각. 오프셋이 있으면 화면 '정중앙(0)'이 아니라 이 값에 물체를 놓아야
# ToF 가 정확히 겨냥된다.
AIM_TARGET_DEG = CAM_TOF_OFFSET_DEG

# 폐루프 겨냥에서 이 정도 안으로 들어오면 성공으로 본다.
# ToF 원뿔이 25도라 2도면 충분히 안쪽이다.
AIM_TOLERANCE_DEG = 2.0
AIM_MAX_ITER = 3
AIM_TIMEOUT_S = 3.0

# ===============================
# 지도 / 격자
# ===============================
GRID_SIZE = 60
CELL_SIZE = 0.05        # m per cell (5cm)
# 5cm 격자 + 사거리 1800mm → 36칸. path_planner_sim2 의 SCAN_RANGE 와 맞출 것.
SCAN_RANGE_CELLS = int((1800 / 1000.0) / CELL_SIZE)   # 36

# ===============================
# 거리센서 유효범위  (실측으로 확정)
# ===============================
# VL53L0X 는 미검출 시 최대거리나 에러값을 반환한다. 그대로 쓰면 허공에
# 가짜 장애물 벽이 생기므로 반드시 걸러야 한다.
#
# ★ ESP32 의 TOF_MAX_MM 과 반드시 같은 값으로 유지할 것.
#   양쪽이 다르면 ESP32 가 valid=1 로 보낸 값을 파이썬이 버리는 일이 생긴다.
DIST_MIN_MM = 30.0
DIST_MAX_MM = 1800.0        # .ino 의 TOF_MAX_MM 도 1800 으로 맞출 것
TOF_CONE_DEG = 25.0         # VL53L0X FoV. 테두리 직접 측정이 불가한 이유

# ===============================
# 자이로  (ESP32 규약과 맞출 것)
# ===============================
# 자이로는 왼쪽 회전이 음수인데 수학 관례(atan2, cos/sin)는 왼쪽이 양수다.
# 그대로 쓰면 지도가 좌우로 뒤집히므로 입구에서 한 번만 부호를 바꾼다.
# 실물에서 왼쪽으로 90도 돌렸을 때 지도가 반시계로 도는지 확인해 확정.
GYRO_SIGN = -1.0

# ===============================
# 네트워크
# ===============================
PC_IP = "192.168.137.1"
PI_IP = "192.168.137.52"
ESP32_IP = "192.168.137.51"     # DHCP 라 재부팅 시 바뀔 수 있음
STREAM_PORT = 9999              # ESP32 -> PC (PC가 서버)
CMD_PORT = 8890                 # PC -> ESP32 (ESP32가 서버)
CAMERA_PORT = 8888              # 라파 -> PC

# ===============================
# ESP32 펌웨어 값 (참고용 사본 - 실제 값은 .ino 에서 고칠 것)
# ===============================
ESP32_SERVO_CENTER = 90         # 정면을 볼 때의 서보 명령각
ESP32_SERVO_SIGN = 1.0          # 오프셋각 -> 서보각 부호
ESP32_SERVO_MIN_OFF = -80.0     # 오프셋 하한 (물리 서보각 10)
ESP32_SERVO_MAX_OFF = 80.0      # 오프셋 상한 (물리 서보각 170)


# ===============================
# 유도 값
# ===============================
def focal_px(img_w=None):
    """
    초점거리(px). 실측값이 기준이며, 해상도만 다를 때 비례 스케일한다.
    ※ 센서 모드가 바뀌면(크롭 발생) 비례가 성립하지 않으므로 재측정할 것.
    """
    if img_w is None or img_w == IMG_W:
        return FOCAL_PX
    return FOCAL_PX * img_w / IMG_W


def pixel_to_angle(px, img_w=None):
    """이미지 x픽셀 -> 광축 기준 각도(도). 선형 근사가 아닌 핀홀(atan) 모델.
    광각일수록 가장자리에서 선형 근사 오차가 커진다."""
    w = IMG_W if img_w is None else img_w
    return math.degrees(math.atan((px - w / 2.0) / focal_px(w)))


def angle_to_pixel(deg, img_w=None):
    """각도 -> 이미지 x픽셀. 겨냥 목표 지점을 화면에 표시할 때 쓴다."""
    w = IMG_W if img_w is None else img_w
    return w / 2.0 + focal_px(w) * math.tan(math.radians(deg))


def width_from_box(dist_mm, x_l, x_r, img_w=None):
    """
    YOLO 박스 좌우 픽셀 + ToF 중심거리 -> 물체 실제 폭(mm).

    시선거리를 광축 방향 깊이로 바꾼 뒤 픽셀폭에 곱한다.
        Z = dist * cos(중심각);  width = 픽셀폭 * Z / f

    ※ 물체가 평면(상자 앞면)이라는 가정. 원통이면 2*d*tan(각폭/2) 가 맞지만
      그 식은 중심각이 안 들어가서 화면 가장자리에서 크게 틀린다.
    """
    w = IMG_W if img_w is None else img_w
    f = focal_px(w)
    center_ang = math.atan(((x_l + x_r) / 2.0 - w / 2.0) / f)
    depth = dist_mm * math.cos(center_ang)
    return abs(x_r - x_l) * depth / f


def mm_to_cells(mm):
    return (mm / 1000.0) / CELL_SIZE


def cells_to_mm(cells):
    return cells * CELL_SIZE * 1000.0


def normalize_angle(a):
    """임의 각도를 [-180, 180) 로 접는다. 같은 방향이 항상 같은 값이 된다."""
    return (a + 180.0) % 360.0 - 180.0


def gyro_to_heading(gyro_deg):
    """자이로 원본값 -> 지도 기준 헤딩(수학 관례, 반시계 +)."""
    return normalize_angle(GYRO_SIGN * gyro_deg)


def tof_spot_mm(distance_mm):
    """해당 거리에서 ToF 측정 스팟의 지름(mm).
    테두리를 직접 겨냥하면 안 되는 이유를 수치로 확인할 때 쓴다."""
    return 2.0 * distance_mm * math.tan(math.radians(TOF_CONE_DEG / 2.0))


def half_fov(img_w=None):
    """핀홀 모델 기준 수평 반화각(도). ★ FOCAL_PX 에서 유도된다.

    (img_w/2) / tan(half_fov) == focal_px(img_w) 가 항상 성립하므로,
    이 값을 써서 초점거리를 역산하는 옛 코드도 정확한 값을 얻는다.

    ※ CAMERA_VIEW_DEG(113도, 실제 시야)와 다른 게 정상이다.
      전자는 핀홀 근사, 후자는 배럴 왜곡을 포함한 실제 시야각이다.
      "픽셀↔각도"에는 이 함수를, "볼 수 있는 범위"에는 CAMERA_VIEW_DEG 를 쓴다.
    """
    w = IMG_W if img_w is None else img_w
    return math.degrees(math.atan((w / 2.0) / focal_px(w)))


# ★ CAMERA_HFOV 는 일부러 정의하지 않는다. 독립 상수로 두면 FOCAL_PX 와
#   어긋나고(예: HFOV=113 을 쓰면 폭이 +33% 과대), 그 오차가 조용히 폭 계산에
#   섞여 들어간다. 전체 화각이 필요하면 2 * half_fov() 를 쓸 것.


def cam_angle_to_tof(cam_deg):
    """카메라 각도 -> ToF 겨냥 각도. 부호 규약을 여기 한 곳에서만 정한다.

    CAM_TOF_OFFSET_DEG 는 "화면 정중앙(카메라 0도)에 물체를 놓았을 때 ToF 가
    실제로 보는 각도"다. 따라서 ToF 를 물체에 맞추려면 카메라 각도에서
    그만큼 빼야 한다. (과거 vision_object.py 는 반대로 더하고 있었음 — 오프셋이
    0일 땐 안 드러나다가 실측값을 넣는 순간 두 경로가 2*offset 만큼 벌어진다.)
    """
    return cam_deg - CAM_TOF_OFFSET_DEG


def tof_angle_to_cam(tof_deg):
    """cam_angle_to_tof() 의 역함수 -- ToF 기준 각도차 -> 카메라 기준 각도차.

    [왜 필요한가]
      cam_angle_to_tof() 는 "카메라가 본 각도로 ToF를 겨냥하기" 방향만 정의돼
      있었다. 반대로 "ToF가 이미 어떤 각도에서 뭔가 감지했는데, 그게 지금
      카메라 프레임에서는 몇 도쯤일까"를 구하려면(ToF 히트 위치를 카메라로
      크롭해서 윤곽선만 따는 파이프라인에 필요) 역방향 변환이 있어야 한다.

    ★ 부호는 cam_angle_to_tof() 와 정확히 반대(+ offset)다. 양쪽을 여기
      한 파일에서만 정의해 부호가 서로 어긋날 여지를 없앤다. 둘 중 하나만
      고치고 다른 쪽을 안 고치는 사고를 막기 위해, 이 둘은 항상 서로의
      역함수 관계(tof_angle_to_cam(cam_angle_to_tof(x)) == x)로 유지할 것.
    """
    return tof_deg + CAM_TOF_OFFSET_DEG


if __name__ == "__main__":
    print("=== 현재 보정값 ===")
    print(f"해상도           : {IMG_W} x {IMG_H}  (16:9)")
    print(f"초점거리(실측)   : {FOCAL_PX:.1f}px")
    print(f"1픽셀 각도(중앙) : {pixel_to_angle(CX + 1):.3f}도")
    print(f"핀홀 환산 HFOV   : {2 * pixel_to_angle(IMG_W):.1f}도 "
          f"(참고용. 실제 시야 {CAMERA_VIEW_DEG}도)")
    print(f"카메라-ToF 오프셋: {CAM_TOF_OFFSET_DEG}도  [미측정]")
    print(f"겨냥 목표각      : {AIM_TARGET_DEG}도 "
          f"(화면 픽셀 {angle_to_pixel(AIM_TARGET_DEG):.0f})")
    _f_check = (IMG_W / 2.0) / math.tan(math.radians(half_fov()))
    print(f"half_fov() 역산  : f={_f_check:.1f}px "
          f"(focal_px()={focal_px():.1f}px 와 같아야 함, 차이 {abs(_f_check - focal_px()):.4f})")
    _round_trip = tof_angle_to_cam(cam_angle_to_tof(37.5))
    print(f"역함수 검증      : cam->tof->cam(37.5) = {_round_trip:.4f} "
          f"(37.5 와 같아야 함)")
    print()
    print("=== 실측 역검증 (A4 210mm @ 1000mm -> 59px) ===")
    print(f"  width_from_box = {width_from_box(1000, 290.5, 349.5):.1f}mm "
          f"(정답 210mm)")
    print()
    print("=== 픽셀 -> 각도 ===")
    for px in (0, 160, 320, 480, 640):
        print(f"  {px:4d}px -> {pixel_to_angle(px):7.2f}도")
    print()
    print("=== ToF 스팟 크기 (테두리 직접측정이 불가한 이유) ===")
    for d in (300, 500, 1000, 1500):
        print(f"  {d}mm 에서 스팟 지름 {tof_spot_mm(d):.0f}mm")
    print()
    print("=== 자이로 변환 확인 ===")
    for g in (0, -90, -180, -270, 90, 270):
        print(f"  자이로 {g:>5}도 -> 헤딩 {gyro_to_heading(g):>7.1f}도")
#	장애물 테두리 각도/형태/크기 처리. 입력은 마스크(YOLO-seg) 또는 컨투어(Canny) 둘 다 가능 — 각도/부호 계산은 전부 robot_config(C) 로 위임, 로컬 상수 사본 없음
"""
vision_object.py -- 물체 테두리 각도 / 형태 / 크기 계산

[핵심 변경 — YOLO 없이 장애물 인식]
  전시회 장애물을 직접 제작하고 실제 재난 잔해 데이터셋을 확보할 방법이
  없어서, 장애물 쪽은 "클래스가 뭔지" 분류할 필요도 방법도 없다. 다행히
  경로계획에 필요한 건 "여기부터 여기까지"(윤곽선 좌/우 극점 각도)뿐이라
  class-agnostic 윤곽 추출(Canny+findContours)로 충분하다.
  YOLO(분류 필요)는 화재 탐지에만 남는다 — 불은 도메인 미스매치 없이
  기존 공개 데이터셋(D-Fire 등)으로 학습 가능하기 때문.

  그래서 이 파일은 입력 형태와 무관하게 동작하도록 두 갈래로 갈린다:
    - 마스크 경로 (mask_to_contour -> analyze_mask)   : YOLO-seg 등, 레거시
    - 컨투어 경로 (contour_from_canny_roi -> analyze_contour) : ToF 트리거,
      class-agnostic, 지금 장애물 인식에 실제로 쓰는 경로
  둘 다 결국 analyze_contour() 로 합류하므로 이후 처리(형태 분류/각도
  변환/크기 계산)는 완전히 동일하다.

[역할]
  픽셀 마스크 또는 윤곽선을 받아서
    1) 윤곽선 추출 (마스크 경로일 때만; 컨투어 경로는 이미 윤곽선)
    2) 좌/우 테두리의 '각도' 계산   (ToF보다 80배 정확한 부분)
    3) 형태 분류 (원/사각/삼각)      (지도쪽 estimate_obstacle 과 같은 기준)
    4) 회전각 추출                   (지도쪽에는 없는, 카메라만 할 수 있는 것)
  까지 처리한다. 거리는 ToF 가 담당하므로 여기서 구하지 않는다.

[왜 bbox 가 아니라 윤곽선인가]
  bbox 는 축정렬 사각형이라 기울어진 물체의 좌우 끝을 과대평가한다.
  윤곽선의 실제 극점을 쓰면 각폭이 정확해지고, 그만큼 크기 오차가 준다.

[지도쪽과의 관계]
  map_2D 의 estimate_obstacle 은 ToF 관측점으로 형태를 추정한다.
  여기서 나온 카메라 형태와 교차검증하면 신뢰도를 올릴 수 있다.
  특히 회전각은 ToF 로는 얻기 어려우므로 카메라 값을 쓰는 게 좋다.
"""

import math
import cv2
import numpy as np

# ===============================
# 카메라 파라미터 -- robot_config 한 곳에서 관리한다.
# 값이 파일마다 흩어져 있으면 실측 후 한 군데를 빠뜨려 각도가 어긋난다.
# ===============================
import robot_config as C

IMG_W = C.IMG_W
IMG_H = C.IMG_H
FOCAL_PX = C.focal_px()


def pixel_to_angle(cx_px, img_w=IMG_W):
    """이미지 x픽셀 -> 광축 기준 각도(도). 핀홀(atan) 모델."""
    return C.pixel_to_angle(cx_px, img_w)


def mask_to_contour(mask):
    """이진 마스크 -> 가장 큰 외곽 윤곽선. 없으면 None."""
    m = (mask > 0).astype(np.uint8) * 255
    cnts, _ = cv2.findContours(m, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not cnts:
        return None
    return max(cnts, key=cv2.contourArea)


def classify_shape(contour):
    """윤곽선 -> (형태, 지표들).
      - 면적비(extent, 최소외접사각형 기준)로 삼각형을 먼저 거르고
      - 원형도(circularity)로 원/사각형을 나눈다

    ★ 순서를 extent 우선으로 바꿨다(path_planner_sim2.py 의 v1→v3 교훈과
      동일한 이유). 원래는 근사 다각형 꼭짓점 수(verts)로 먼저 분기했는데,
      Canny+dilate 를 거친 작은 도형은 꼭짓점 개수가 노이즈에 약해서
      (실측: 60px 삼각형이 verts=4 로 잘못 잡혀 곧장 rect 로 확정됨 -
      회전/크기 조합 21케이스 중 6케이스가 이 경로로 오분류) extent 는
      멀쩡한데도 아예 보지도 못하고 틀렸다.
      extent 는 훨씬 안정적으로 갈린다(실측):
        삼각형        0.53~0.59
        원            0.80~0.81
        사각형        0.97~1.00
      ※ circularity 임계값 0.87 은 path_planner_sim2.py 의 CIRC_CIRCLE_MIN
      (0.94)과 다르다 - 의도적이다. 그쪽은 격자 래스터화된 도형(원이
      마름모로 그려짐)이고 여기는 실제 곡선을 Canny 로 딴 도형이라 물리적
      실체 자체가 달라서, 같은 값을 쓰면 오히려 틀린다(실측: 여기 원의
      circ 는 0.90 근방이라 0.94 를 쓰면 전부 사각형으로 오판됨).
    """
    area = cv2.contourArea(contour)
    perim = cv2.arcLength(contour, True)
    if perim == 0 or area == 0:
        return "unknown", {}

    circularity = 4 * math.pi * area / (perim * perim)

    # Douglas-Peucker 근사. eps 는 둘레에 비례시켜 크기 무관하게 만든다.
    # verts 는 더 이상 분류에 안 쓰지만(위 이유), 참고 지표로는 남겨둔다.
    eps = 0.02 * perim
    approx = cv2.approxPolyDP(contour, eps, True)
    verts = len(approx)

    # 최소외접사각형 -> 회전각과 채움률
    (cx, cy), (w, h), angle = cv2.minAreaRect(contour)
    rect_area = w * h
    extent = area / rect_area if rect_area > 0 else 0     # 사각형에 가까울수록 1
    aspect = max(w, h) / min(w, h) if min(w, h) > 0 else 0

    EXTENT_TRI_MAX = 0.70    # 이 미만이면 삼각형 (실측 0.53~0.59, 여유 있게 설정)
    CIRC_CIRCLE_MIN = 0.87   # 삼각형이 아닐 때, 이 이상이면 원 (실측 원 0.90 / 사각형 회전45도 최대 0.84)

    if extent < EXTENT_TRI_MAX:
        shape = "triangle"
    elif circularity >= CIRC_CIRCLE_MIN:
        shape = "circle"
    else:
        shape = "rect"

    return shape, {
        "circularity": round(circularity, 3),
        "vertices": verts,
        "extent": round(extent, 3),
        "aspect": round(aspect, 2),
        "rot_deg": round(angle, 1),
        "center_px": (round(cx, 1), round(cy, 1)),
        "rect_wh_px": (round(w, 1), round(h, 1)),
    }


def analyze_contour(cnt, img_w=IMG_W):
    """윤곽선(전체 프레임 좌표계) -> 각도/형태 정보.

    mask_to_contour() 를 거치지 않고 Canny+findContours 등에서 바로 얻은
    컨투어도 여기로 바로 넣을 수 있다 (analyze_mask 의 핵심 로직 분리판).
    ※ 반환에 거리는 없다. ToF 로 거리를 받은 뒤 size_from_distance() 로
    크기를 낸다.
    """
    if cnt is None:
        return None

    pts = cnt.reshape(-1, 2)
    x_min = float(pts[:, 0].min())
    x_max = float(pts[:, 0].max())
    y_min = float(pts[:, 1].min())
    y_max = float(pts[:, 1].max())

    ang_l = pixel_to_angle(x_min, img_w)
    ang_r = pixel_to_angle(x_max, img_w)
    # 중심은 bbox 중앙이 아니라 윤곽선 무게중심을 쓴다(기울어진 물체에서 더 정확)
    M = cv2.moments(cnt)
    cx = M["m10"] / M["m00"] if M["m00"] else (x_min + x_max) / 2
    ang_c = pixel_to_angle(cx, img_w)

    shape, metrics = classify_shape(cnt)

    return {
        "shape": shape,
        "angle_left": round(ang_l, 2),
        "angle_right": round(ang_r, 2),
        "angle_center": round(ang_c, 2),
        "angular_width": round(ang_r - ang_l, 2),
        "bbox_px": (round(x_min, 1), round(y_min, 1), round(x_max, 1), round(y_max, 1)),
        "metrics": metrics,
        "contour": cnt,
    }


def analyze_mask(mask, img_w=IMG_W):
    """마스크 하나 -> 각도/형태 정보. (YOLO-seg 등 마스크 기반 입력 전용,
    레거시 경로. 실제 컨투어 처리는 analyze_contour() 에 위임한다.)"""
    return analyze_contour(mask_to_contour(mask), img_w)


def size_from_distance(info, distance_mm, img_w=IMG_W, img_h=IMG_H):
    """analyze_mask 결과 + ToF 사거리 -> 실제 크기(mm).

    [주의] ToF 가 주는 건 '사거리'(광선 방향 직선거리)인데,
    크기 계산에 필요한 건 '깊이'(광축 방향 성분)다.
    물체가 화면 가운데서 벗어날수록 둘의 차이가 커져서,
    2*d*tan(각폭/2) 식을 그대로 쓰면 크기가 최대 20% 넘게 과소평가된다.

        깊이 Z = 사거리 r x cos(중심각)
        폭     = 픽셀폭 x Z / f
    """
    f = C.focal_px(img_w)
    x0, y0, x1, y1 = info["bbox_px"]
    ang_c = math.radians(info["angle_center"])

    depth = distance_mm * math.cos(ang_c)      # 사거리 -> 깊이
    width_mm = abs(x1 - x0) * depth / f
    height_mm = abs(y1 - y0) * depth / f

    return {
        "width_mm": round(width_mm, 1),
        "height_mm": round(height_mm, 1),
        "distance_mm": distance_mm,
        "depth_mm": round(depth, 1),
        "shape": info["shape"],
        "rot_deg": info["metrics"].get("rot_deg"),
    }


# ===============================
# ToF 트리거 Canny 윤곽 추출 (class-agnostic, 장애물용 — 실제 진입점)
# ===============================
# ToF 스윕이 어떤 각도에서 물체에 걸리면, 그 각도 근방만 카메라에서 잘라
# Canny+findContours 로 윤곽선을 딴다. "이게 뭔지" 몰라도 되므로 학습이나
# 색상 통제가 전혀 필요 없다. ToF 원뿔이 넓어 정확한 위치가 불확실하므로
# 크롭에 여유(margin)를 준다.

def cam_pixel_angle_for_robot_target(robot_angle_deg, current_servo_angle_deg):
    """ToF가 robot_angle_deg 에서 감지한 물체가, current_servo_angle_deg
    위치에서 찍은 '지금 이 카메라 프레임' 안에서는 몇 도(광축 기준)에
    있을지 역산한다. object_angle_robot() 의 역함수.

    로봇기준각 = 서보각 + cam_angle_to_tof(카메라각)  (object_angle_robot)
    -> 카메라각 = tof_angle_to_cam(로봇기준각 - 서보각)
    """
    delta = robot_angle_deg - current_servo_angle_deg
    return C.tof_angle_to_cam(delta)


def roi_from_tof_hit(robot_angle_deg, current_servo_angle_deg,
                      margin_deg=None, img_w=IMG_W, img_h=IMG_H):
    """ToF 히트 각도 -> 카메라 크롭 박스 (x0, y0, x1, y1), 전체 프레임 좌표.

    margin_deg 기본값은 ToF 원뿔의 반각(TOF_CONE_DEG/2 = 12.5도) — 원뿔
    실측으로 이미 확정된 트림값과 같다. 스윕 중이라 robot_angle_deg 와
    current_servo_angle_deg 사이에 시차가 있어도 이 여유가 흡수해준다.
    수직 방향은 서보가 팬(pan)만 하므로 정보가 없어 전체 높이를 그대로 쓴다.
    """
    if margin_deg is None:
        margin_deg = C.TOF_CONE_DEG / 2.0   # 12.5도

    cam_deg = cam_pixel_angle_for_robot_target(robot_angle_deg, current_servo_angle_deg)
    px_a = C.angle_to_pixel(cam_deg - margin_deg, img_w)
    px_b = C.angle_to_pixel(cam_deg + margin_deg, img_w)
    x0, x1 = sorted((px_a, px_b))
    x0 = int(max(0, round(x0)))
    x1 = int(min(img_w, round(x1)))
    return (x0, 0, x1, img_h)


def contours_from_canny_roi(frame, roi_box, canny_low=60, canny_high=150,
                             min_area_px=200, close_kernel=5, max_contours=10):
    """ROI 안에서 Canny+findContours 로 윤곽선 '여러 개'를 딴다 (클래스 무관).

    frame        : 전체 프레임 (BGR 또는 그레이스케일 numpy 배열)
    roi_box      : (x0, y0, x1, y1), 전체 프레임 픽셀 좌표
    min_area_px  : 이보다 작은 컨투어는 노이즈로 버린다.
    close_kernel : Canny 엣지는 끊어진 선이 많아 findContours 전에 살짝
                   팽창(dilate)시켜 닫아준다. 크기 조정용.
    max_contours : 넓이 내림차순으로 최대 이 개수까지만 돌려준다.

    반환: [(contour, area_px), ...] 넓이 내림차순, 전체 프레임 좌표계로 복원됨.
          없으면 빈 리스트.

    ★ contour_from_canny_roi() (단수, 최대 컨투어 하나만) 는 이 함수의 결과
      중 첫 번째만 꺼내는 얇은 래퍼다. YOLO 는 한 프레임에서 물체 여러 개를
      동시에 감지하는데, 이 함수가 있어야 Canny 쪽도 같은 조건으로 비교
      (camera_test_opencv.py 등에서 여러 물체 동시 인식) 할 수 있다.

    ★ ROI 크롭 좌표에서 딴 컨투어를 (x0, y0) 만큼 다시 더해 전체 프레임
      좌표로 되돌린다 — 이걸 빼먹으면 이후 pixel_to_angle 계산이 전부
      틀어진다(크롭 폭만큼 각도가 밀림).
    """
    x0, y0, x1, y1 = roi_box
    if x1 - x0 < 2 or y1 - y0 < 2:
        return []

    crop = frame[y0:y1, x0:x1]
    gray = cv2.cvtColor(crop, cv2.COLOR_BGR2GRAY) if crop.ndim == 3 else crop
    gray = cv2.GaussianBlur(gray, (5, 5), 0)
    edges = cv2.Canny(gray, canny_low, canny_high)

    kernel = np.ones((close_kernel, close_kernel), np.uint8)
    edges = cv2.dilate(edges, kernel, iterations=1)

    cnts, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    scored = [(c, cv2.contourArea(c)) for c in cnts]
    scored = [(c, a) for c, a in scored if a >= min_area_px]
    scored.sort(key=lambda t: t[1], reverse=True)
    scored = scored[:max_contours]

    offset = np.array([[x0, y0]])
    return [(c + offset, a) for c, a in scored]


def contour_from_canny_roi(frame, roi_box, canny_low=60, canny_high=150,
                            min_area_px=200, close_kernel=5):
    """ROI 안에서 Canny+findContours 로 가장 큰 윤곽선 하나만 딴다 (클래스 무관).
    여러 개가 필요하면 contours_from_canny_roi() 를 쓸 것.

    반환: 전체 프레임 좌표계로 복원된 컨투어(np.ndarray), 없으면 None.
    """
    found = contours_from_canny_roi(frame, roi_box, canny_low, canny_high,
                                     min_area_px, close_kernel, max_contours=1)
    return found[0][0] if found else None


def bbox_vs_contour(mask, img_w=IMG_W):
    """bbox 로 잰 각폭 vs 윤곽선으로 잰 각폭 비교(기울어진 물체에서 차이 확인용)."""
    cnt = mask_to_contour(mask)
    if cnt is None:
        return None
    x, y, w, h = cv2.boundingRect(cnt)
    bb = pixel_to_angle(x + w, img_w) - pixel_to_angle(x, img_w)
    pts = cnt.reshape(-1, 2)
    ct = pixel_to_angle(float(pts[:, 0].max()), img_w) - pixel_to_angle(float(pts[:, 0].min()), img_w)
    return {"bbox_angular_width": round(bb, 2), "contour_angular_width": round(ct, 2)}


# ===============================
# YOLO-seg 연결부 (노트북에서 실행) — 화재 탐지 전용, 장애물 인식엔 이제 안 씀
# ===============================
# 장애물 윤곽선은 위 contour_from_canny_roi() 가 담당한다(class-agnostic,
# 학습/색상 통제 불필요). YOLO 는 "화재냐 아니냐" 분류가 실제로 필요한
# 화재 탐지에만 남는다 — 불은 재난현장이든 어디든 시각적으로 똑같아서
# 도메인 미스매치가 없어 공개 데이터셋(D-Fire 등)으로 학습 가능하다.
def masks_from_yolo(result, conf_min=0.6):
    """ultralytics YOLO-seg 결과 -> [(클래스명, 마스크, 신뢰도), ...]

    사용 예:
        from ultralytics import YOLO
        model = YOLO("yolov8n-seg.pt")          # 반드시 -seg 모델
        r = model(frame)[0]
        for cls, mask, conf in masks_from_yolo(r):
            info = analyze_mask(mask)
            ...
    """
    out = []
    if getattr(result, "masks", None) is None:
        return out
    names = result.names
    for i, m in enumerate(result.masks.data):
        conf = float(result.boxes.conf[i])
        if conf < conf_min:
            continue
        cls_id = int(result.boxes.cls[i])
        mask = m.cpu().numpy().astype(np.uint8)
        out.append((names.get(cls_id, str(cls_id)), mask, conf))
    return out


# ===============================
# 일체형(카메라 + ToF가 서보에 함께 장착) 각도 계산
# ===============================
# 카메라가 서보와 같이 돌기 때문에, 픽셀 각도는 '로봇 정면 기준'이 아니라
# '지금 서보가 보고 있는 방향 기준'이다. 로봇 기준으로 바꾸려면 서보각을 더한다.
#
#     로봇기준 절대각 = 서보각 + 픽셀각 + 장착오차
#
# 이점: ToF를 겨냥하려면 서보를 '픽셀각만큼' 더 돌리면 끝이고,
#       돌린 뒤 물체가 화면 정중앙에 오므로 겨냥 성공을 눈으로 확인할 수 있다.
#       (카메라 고정형에서는 불가능한 검증)

def object_angle_robot(pixel_x, servo_angle_deg, img_w=IMG_W):
    """물체의 '로봇 정면 기준' 절대 각도(도).
    지도(map_2D)에 넘길 때는 여기에 로봇 헤딩까지 더해야 한다.
    부호 규약은 robot_config.cam_angle_to_tof() 한 곳에서만 정한다
    (camera_test.py 와 반대 부호였던 버그 수정)."""
    return servo_angle_deg + C.cam_angle_to_tof(pixel_to_angle(pixel_x, img_w))


def servo_target_to_center(pixel_x, servo_angle_deg, img_w=IMG_W):
    """물체를 화면 정중앙에 놓기 위해 서보를 돌릴 목표각(도).
    이 각도로 돌리면 ToF 광축이 물체를 향하게 된다."""
    return object_angle_robot(pixel_x, servo_angle_deg, img_w)


def add_robot_frame_angles(info, servo_angle_deg):
    """analyze_mask/analyze_contour 결과에 로봇 기준 절대각 3개를 추가한다.
    (analyze_with_servo() 와 obstacle_detector.py 가 공유하는 핵심 변환)"""
    if info is None:
        return None
    info["servo_angle"] = servo_angle_deg
    info["robot_angle_left"] = round(servo_angle_deg + C.cam_angle_to_tof(info["angle_left"]), 2)
    info["robot_angle_right"] = round(servo_angle_deg + C.cam_angle_to_tof(info["angle_right"]), 2)
    info["robot_angle_center"] = round(servo_angle_deg + C.cam_angle_to_tof(info["angle_center"]), 2)
    return info


def analyze_with_servo(mask, servo_angle_deg, img_w=IMG_W):
    """analyze_mask 결과의 각도들을 '로봇 기준'으로 변환해서 함께 담아준다."""
    return add_robot_frame_angles(analyze_mask(mask, img_w), servo_angle_deg)
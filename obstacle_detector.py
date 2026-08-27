"""
obstacle_detector.py -- ToF 스윕이 잡은 히트를 카메라로 정밀화하는 오케스트레이터

[파이프라인]
    ToF 연속 스윕 중 히트 발생 (angle_deg, dist)
      -> 그 각도 근방만 카메라 프레임에서 크롭      (vision_object.roi_from_tof_hit)
      -> Canny+findContours 로 class-agnostic 윤곽선 추출
                                                    (vision_object.contour_from_canny_roi)
      -> 윤곽선 좌/우 극값 각도를 로봇(ToF) 기준으로 변환
                                                    (vision_object.add_robot_frame_angles)
      -> 그 중심각으로 ToF 를 정밀 재겨냥            (TofCommander.aim)
      -> 재겨냥한 깨끗한 거리로 폭/형태 계산          (vision_object.size_from_distance)

[왜 YOLO 를 안 쓰는가]
    전시회 장애물을 직접 제작하고, 실제 재난 잔해를 촬영해 학습 데이터셋을
    만들 방법이 없다. 다행히 여기서 필요한 건 "이게 뭔지"가 아니라
    "여기부터 여기까지"뿐이라 클래스 분류(=학습) 자체가 필요 없다.
    YOLO 는 화재 탐지에만 남는다 (화재는 도메인 미스매치 없이 기존 공개
    데이터셋으로 학습 가능 — vision_object.py 상단 docstring 참고).

[tof_commander.measure_object() 와 다른 점]
    measure_object() 는 "카메라가 이미 어디를 보고 있는지 aim 없이 안다"는
    전제(YOLO bbox, 카메라가 곧 겨냥 기준)로 짜여 있다. 여기서는 반대로
    "ToF 가 먼저 어딘가에서 히트를 보고 -> 카메라가 그 근처만 뒤늦게 확인"
    하는 흐름이라 서보각(servo_angle_deg)을 명시적으로 다뤄야 한다.

[사용 예]
    tof = TofCommander(C.ESP32_IP)
    tof.connect()
    # 스윕 스트림에서 히트 하나: angle_deg=-12.3, dist=612
    # 그 순간 카메라도 한 프레임 grab 했다고 가정 (frame, servo_angle_deg 는
    # 그 캡처 시점의 실제 서보각 — 스트림의 angle_deg 와 시차가 있을 수 있음)
    res = detect_obstacle(frame, tof_angle_deg=-12.3, servo_angle_deg=-12.3, tof=tof)
    print(res)   # {'ok': True, 'distance_mm':608, 'width_mm':341, ...}
"""

import robot_config as C
import vision_object as vo


def detect_obstacle(frame, tof_angle_deg, servo_angle_deg, tof,
                     margin_deg=None, aim_samples=5, aim_timeout=2.0,
                     canny_low=60, canny_high=150, min_area_px=200):
    """ToF 히트 하나를 카메라로 정밀화해 폭/형태까지 낸다.

    frame            : 그 순간 카메라가 찍은 전체 프레임 (BGR numpy 배열)
    tof_angle_deg    : ToF 스윕이 히트를 보고한 로봇기준 각도
    servo_angle_deg  : frame 을 찍은 순간의 실제 서보 각도.
                        스윕 중이라 tof_angle_deg 와 다를 수 있다 — 그 시차가
                        크면 margin_deg 를 넉넉히 줘야 ROI 안에 물체가 들어온다.
    tof              : 이미 connect() 된 TofCommander 인스턴스
    margin_deg       : None 이면 TOF_CONE_DEG/2(12.5도) 사용

    반환: dict. 성공 시
        {'ok': True, 'distance_mm', 'width_mm', 'height_mm', 'angle_deg',
         'angle_left', 'angle_right', 'angular_width_deg', 'shape',
         'rot_deg', 'yaw', 'roi'}
    실패 시 {'ok': False, 'reason': ..., ...(가능한 만큼의 부분 정보)}
    reason 종류: 'no_contour'(Canny가 윤곽선을 못 찾음),
                 'no_response'(ToF 재겨냥 응답 없음),
                 'no_valid_range'(ToF 재겨냥은 됐는데 유효 거리가 아님)
    """
    roi = vo.roi_from_tof_hit(tof_angle_deg, servo_angle_deg, margin_deg=margin_deg)

    cnt = vo.contour_from_canny_roi(frame, roi, canny_low=canny_low,
                                     canny_high=canny_high, min_area_px=min_area_px)
    if cnt is None:
        return {"ok": False, "reason": "no_contour", "roi": roi}

    info = vo.analyze_contour(cnt)
    info = vo.add_robot_frame_angles(info, servo_angle_deg)

    ang_c_robot = info["robot_angle_center"]

    res = tof.aim(ang_c_robot, samples=aim_samples, timeout=aim_timeout)
    if res is None or not res.get("ok"):
        return {
            "ok": False,
            "reason": "no_response" if res is None else "no_valid_range",
            "angle_deg": ang_c_robot,
            "shape": info["shape"],
            "roi": roi,
        }

    dist = int(res["dist"])
    size = vo.size_from_distance(info, dist)

    return {
        "ok": True,
        "distance_mm": dist,
        "width_mm": size["width_mm"],
        "height_mm": size["height_mm"],
        "angle_deg": ang_c_robot,
        "angle_left": info["robot_angle_left"],
        "angle_right": info["robot_angle_right"],
        "angular_width_deg": round(abs(info["robot_angle_right"] - info["robot_angle_left"]), 2),
        "shape": info["shape"],
        "rot_deg": info["metrics"].get("rot_deg"),
        "yaw": res.get("yaw"),
        "roi": roi,
    }


if __name__ == "__main__":
    # 하드웨어 없이 파이프라인 자체가 도는지 확인 (TofCommander 를 가짜로 대체)
    import numpy as np
    import cv2

    class FakeTof:
        """실제 ESP32 없이 aim() 응답만 흉내낸다."""
        def aim(self, angle_deg, samples=5, timeout=2.0):
            return {"ok": True, "dist": 600, "yaw": 0.0, "n": samples}

    frame = np.full((C.IMG_H, C.IMG_W, 3), 60, dtype=np.uint8)
    cv2.rectangle(frame, (280, 120), (360, 260), (220, 220, 220), -1)

    result = detect_obstacle(frame, tof_angle_deg=0.0, servo_angle_deg=0.0,
                              tof=FakeTof())
    print("=== FakeTof 로 파이프라인 자체 검증 ===")
    for k, v in result.items():
        print(f"  {k}: {v}")
#	모터 명령 송신 — 포트 8890 소켓 (send_command 자리)
#
# ★ 2026-08-27 프로토콜 갱신: 실제 이동제어 펌웨어(main_base.ino)를 대조한
#   결과가 아래 두 함수에 반영돼 있다.
#     - move_forward(mm): 여전히 duration_ms 방식(펌웨어에 거리 개념이
#       없음 - CLAUDE.md "⑤ 거리 개념이 아예 없다" 참고). 계수는 이제
#       robot_config 에서만 가져온다(단일출처) - 여기 숫자를 또 안 고쳐도
#       실측 후 robot_config 한 곳만 고치면 끝난다.
#     - turn_left/turn_right(deg): ★펌웨어가 duration_ms 를 안 본다.
#       자이로(MPU6050 DMP) 폐루프로 고정 90도만 돈다 - deg 인자는 지금
#       무시된다(호출부 호환을 위해 시그니처만 유지). 회전이 이제
#       "시간 명령"이 아니라 "완료할 때까지 알아서" 인 상태 명령이라
#       duration_ms 계산 자체가 의미가 없어졌다.
#   ★부작용: 경로 스무딩(SMOOTH_MAX_MM)이 만드는 임의 각도(18.43도 등)
#     회전은 실물에서 전부 90도로 잘린다 - 이건 이 파일이 아니라
#     main_base.ino 의 한계다(파이썬을 어떻게 고쳐도 안 없어짐). 임의
#     각도가 필요해지면 .ino 의 LEFT_TURN_ANGLE/RIGHT_TURN_ANGLE 을
#     함수 인자로 바꿔야 한다 - 지금은 "원본 변수 안 건드림" 방침으로
#     보류 중.
import socket
import json

import robot_config as C

# ★ IP/포트는 robot_config 를 유일한 출처로 쓴다. 예전엔 IP가 파일마다
#   .126 / .51 / .55 / .79 로 제각각이었고(수정지침.md §4), 포트도 8888
#   (=라파 카메라 스트림 포트)로 잘못 잡혀 있어서 모터 명령이 카메라
#   스트림 포트로 나가고 있었다. ESP32 는 DHCP 라 IP가 바뀔 수 있으니
#   여기 값이 아니라 robot_config.ESP32_IP 를 실측 후 갱신할 것.
ESP32_IP = C.ESP32_IP
ESP32_MOTOR_PORT = C.CMD_PORT       # 8890

_client_socket = None

def connect_to_esp32():
    global _client_socket
    if _client_socket is not None:
        return True
    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.settimeout(3.0)
        sock.connect((ESP32_IP, ESP32_MOTOR_PORT))
        _client_socket = sock
        print("ESP32 모터 서버 연결 성공")
        return True
    except Exception as e:
        print(f"ESP32 연결 실패: {e}")
        _client_socket = None
        return False

def send_command(command):
    global _client_socket
    if _client_socket is None:
        if not connect_to_esp32():
            return

    try:
        message = json.dumps(command) + "\n"
        _client_socket.send(message.encode())
        print(f"명령 전송: {command}")
    except Exception as e:
        print(f"명령 전송 실패: {e}")
        _client_socket = None

def move_forward(distance_mm=250):
    # K, C 는 robot_config 가 단일 출처 - 여기 숫자를 직접 안 고친다.
    # 둘 다 아직 TODO(미실측) 이라 지금은 이전과 같은 값(K=4, C=0)이지만,
    # 실측 후엔 robot_config 한 곳만 고치면 이 함수도 자동으로 맞다.
    duration_ms = int(distance_mm * C.MS_PER_MM + C.MOVE_ACCEL_MS)
    send_command({"cmd": "forward", "speed": 100, "duration_ms": duration_ms})

def turn_left(angle_deg=10):
    # ★angle_deg 는 지금 무시된다 - main_base.ino 가 duration_ms 를 안 보고
    # 자이로 폐루프로 고정 90도만 돈다(위 파일 상단 주석 참고). 시그니처는
    # 기존 호출부 호환을 위해 유지.
    send_command({"cmd": "turn_left"})

def turn_right(angle_deg=10):
    # ★turn_left 와 동일 - angle_deg 무시, 고정 90도.
    send_command({"cmd": "turn_right"})

def stop():
    send_command({"cmd": "stop"})

def ping():
    """연결 확인용. ESP32 가 살아있으면 {"type":"pong"} 이 온다(응답은
    이 소켓에서 직접 안 읽는다 - 필요하면 호출부에서 recv)."""
    send_command({"cmd": "ping"})

def close_connection():
    global _client_socket
    if _client_socket is not None:
        _client_socket.close()
        _client_socket = None
        print("ESP32 연결 종료")

# motor_commander.py 맨 아래에 테스트 코드 추가
if __name__ == "__main__":
    send_command({"cmd": "연결 완료", "msg": "연결 확인"})
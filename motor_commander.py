#	모터 명령 송신 — 포트 8890 소켓 (send_command 자리)
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
    duration_ms = int(distance_mm * 4)  # 임시 계산식, 나중에 실측값으로 조정
    send_command({"cmd": "forward", "speed": 100, "duration_ms": duration_ms})

def turn_left(angle_deg=10):
    duration_ms = int(abs(angle_deg) * 10)
    send_command({"cmd": "turn_left", "speed": 100, "duration_ms": duration_ms})

def turn_right(angle_deg=10):
    duration_ms = int(abs(angle_deg) * 10)
    send_command({"cmd": "turn_right", "speed": 100, "duration_ms": duration_ms})

def stop():
    send_command({"cmd": "stop"})

def close_connection():
    global _client_socket
    if _client_socket is not None:
        _client_socket.close()
        _client_socket = None
        print("ESP32 연결 종료")

# motor_commander.py 맨 아래에 테스트 코드 추가
if __name__ == "__main__":
    send_command({"cmd": "연결 완료", "msg": "연결 확인"})
/*
  main_base.ino
  =====================================================
  베이스: 사용자가 검증한 RC카 수동조종(BT) 스케치. 아래 원본 섹션의
  변수/함수/로직은 단 한 줄도 안 바꿨다 - 전부 그대로다.

  이번에 추가한 것: TCP(WiFi) 로 노트북(motor_commander.py)이 같은
  차를 명령할 수 있는 두 번째 채널. BT 는 그대로 살아있다(수동조종
  시연용으로 유지하기로 확정 - CLAUDE.md "★설계 결정 2026-08-27" 참고).
  새 블록은 전부 "★신규" 로 표시해뒀다 - 원본과 신규를 한눈에 구분하려고.

  [파티션 - 필수]
    BT + WiFi 를 한 스케치에 넣으면 기본 파티션(1.3MB)은 플래시 초과로
    빌드가 실패한다(bt_tcp_coexist_test 로 실측 확인됨). 반드시 huge_app
    으로 컴파일할 것:
      arduino-cli compile --fqbn esp32:esp32:esp32:PartitionScheme=huge_app main_base
    IDE: 도구 -> Partition Scheme -> Huge APP (3MB No OTA/1MB SPIFFS)
    ★대가: OTA 불가, USB 로만 재플래시.

  [센서 관련 - 지금은 없음, 나중에]
    ToF/서보/온습도(rescue_sensor_node.ino 쪽 담당)는 아직 안 넣었다.
    지금은 "노트북 <-> ESP32 이동 명령" 통신 구조만 검증하는 단계다.
    두 펌웨어를 실제로 합칠 때 그쪽 로직을 이 구조 위에 옮기면 된다
    (그때 MPU6050_light 는 버리고 이 파일의 DMP 로 통일 - CLAUDE.md
    "★모터 명령 경로" 항목 참고).

  [motor_commander.py 와의 계약]
    ESP32 가 TCP 서버(CMD_PORT), 노트북이 클라이언트 - 기존 그대로.
    지원 명령(JSON, 줄바꿈으로 구분):
      {"cmd":"forward",  "speed":100, "duration_ms":500}
      {"cmd":"backward", "speed":100, "duration_ms":500}
      {"cmd":"turn_left"}                                 // 고정 90도, 폐루프
      {"cmd":"turn_right"}                                // 고정 90도, 폐루프
      {"cmd":"stop"}
      {"cmd":"ping"}
    ★turn_left/turn_right 는 duration_ms 를 안 본다 - 이 회전은 이미
    자이로 PID 로 목표각까지 자동 도달한다(원본 updateTurnControl 그대로,
    안 건드림). 임의 각도(예: 18.43도)는 원본의 LEFT_TURN_ANGLE/
    RIGHT_TURN_ANGLE 를 함수 인자로 바꿔야 나온다 - "원본 변수 안 건드림"
    원칙 때문에 이번엔 안 했다. 필요해지면 그때 별도로.
    ★forward/backward 는 duration_ms 만큼 진행 후 자동 정지한다(신규
    타이머, 아래 참고) - 원본의 'F'/'B' 는 'X' 받을 때까지 무한 전진이었는데,
    그 상태 진입/PID 자체는 그대로 두고 "정지 타이밍만" 밖에서 잰다.
  =====================================================
*/

#include <Wire.h>
#include "BluetoothSerial.h"
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include <WiFi.h>   // ★신규 - TCP 용

// =====================================================
// 함수 선언
// =====================================================
void finishTurn();
void stopMotorOutput();
void driveTurn(int pwm, float errSign);
void startTurn90(bool turnLeft);
void startHeadingCapture(bool forwardDirection);
void updateTurnControl();
void updateStraightControl(float dt);
void checkBluetoothConnection();
void startYawCalibration();
void processYawCalibrationSample(float rawYaw);
void applyCurrentMotorOutput();
void stopVehicle();
void handleCommand(char command);
void readCommands();
bool updateYaw();
bool initializeMPU6050();

// ★신규 - TCP 쪽 함수 선언
void maintainWifi();
void pollTcpCommand();
void handleTcpCommand(const String& line);
void checkTcpAutoStop();
bool jsonStr(const String& s, const char* key, String& out);
bool jsonNum(const String& s, const char* key, float& out);

// =====================================================
// Bluetooth
// =====================================================
BluetoothSerial SerialBT;

const char* BLUETOOTH_NAME = "ESP32_RC";


// =====================================================
// ★신규 - TCP (WiFi)
//
// ESP32 가 서버, 노트북(motor_commander.py)이 클라이언트 - robot_config.py
// 의 CMD_PORT(8890)와 값을 맞췄다. WIFI_SSID/PASS 는 rescue_sensor_node.ino
// 사본과 동일(라즈베리파이 핫스팟) - 그쪽이 바뀌면 여기도 맞출 것.
// =====================================================
const char*    WIFI_SSID_TCP = "pjh";
const char*    WIFI_PASS_TCP = "12345678";
const uint16_t CMD_PORT_TCP  = 8890;

WiFiServer  tcpCmdServer(CMD_PORT_TCP);
WiFiClient  tcpCmdClient;
String      tcpLineBuf;

unsigned long lastWifiRetryMs = 0;
const unsigned long WIFI_RETRY_MS = 5000;

// forward/backward 자동 정지 타이머. duration_ms 가 0 이면(생략 시) 안
// 걸고 - 예전 'F'/'B' 처럼 stop 받을 때까지 무한 전진.
//
// ★2단계로 나눴다 - 명령을 받자마자 카운트를 시작하면 안 된다.
//   startHeadingCapture() 는 최소 YAW_SETTLE_TIME_MS(300ms) 동안 헤딩만
//   재고 실제로는 안 움직인다(원본 로직, 안 건드림). duration_ms=100/200
//   같은 짧은 값을 명령 즉시부터 세면, 차가 출발하기도 전에 타이머가
//   울려 "안 감"으로 측정된다 - 실측(직진 K/C) 자체가 왜곡된다.
//   그래서 "요청됨"(캡처 중이라 아직 대기) 과 "가동함"(실제로
//   STATE_FORWARD/BACKWARD 로 들어간 순간부터 카운트) 을 분리한다.
bool          tcpAutoStopRequested   = false;   // 명령은 받았고 시작 대기 중
bool          tcpAutoStopArmed       = false;   // 실제로 움직이기 시작해 마감시각 확정됨
unsigned long tcpAutoStopDurationMs  = 0;
unsigned long tcpAutoStopAtMs        = 0;


// =====================================================
// BTS7960 핀
// =====================================================

// 왼쪽
#define L_REN   25
#define L_LEN   26
#define L_RPWM  27
#define L_LPWM  33

// 오른쪽
#define R_REN   16
#define R_LEN   17
#define R_RPWM  32
#define R_LPWM  19


// =====================================================
// MPU6050
// =====================================================
MPU6050 mpu;

#define MPU_INTERRUPT_PIN 4

volatile bool mpuInterrupt = false;

bool dmpReady = false;

uint8_t devStatus = 0;
uint8_t fifoBuffer[64];

Quaternion q;
VectorFloat gravity;

float ypr[3];

float rawYawDegree = 0.0f;
float manualYawOffset = 0.0f;
float currentYaw = 0.0f;

bool firstYawRead = true;


// =====================================================
// Bluetooth 연결 시 Yaw 0도 보정
// =====================================================
bool previousBluetoothConnected = false;
bool yawCalibrationActive = false;

const unsigned long YAW_CALIBRATION_TIME_MS = 2000;
const int YAW_CALIBRATION_MIN_SAMPLES = 50;

unsigned long yawCalibrationStartTime = 0;

double yawCalibrationSinSum = 0.0;
double yawCalibrationCosSum = 0.0;

int yawCalibrationSampleCount = 0;


// =====================================================
// PWM
// =====================================================
int leftBaseSpeed = 130;
int rightBaseSpeed = 130;

int leftAppliedPWM = 0;
int rightAppliedPWM = 0;

const int PWM_FREQUENCY = 1000;
const int PWM_RESOLUTION = 8;


// =====================================================
// 제어 주기
// =====================================================
const unsigned long CONTROL_INTERVAL_MS = 30;

const unsigned long PRINT_INTERVAL_MS = 400;

const unsigned long YAW_SETTLE_TIME_MS = 300;

const int MIN_YAW_SAMPLES = 5;


// =====================================================
// 직진 Yaw PID
// =====================================================
const float YAW_CORRECTION_SIGN = -1.0f;

float KP_YAW = 2.5f;
float KI_YAW = 0.0f;
float KD_YAW = 0.2f;

const int MAX_YAW_CORRECTION = 20;

const float YAW_INTEGRAL_LIMIT = 40.0f;

float yawErrorIntegral = 0.0f;
float lastYawError = 0.0f;


// =====================================================
// 90도 회전 설정
// =====================================================

// 목표각
const float LEFT_TURN_ANGLE = 90.0f;
const float RIGHT_TURN_ANGLE = -90.0f;


// -----------------------------------------------------
// 1단계 : 거친 접근
// -----------------------------------------------------
const float TURN_KP = 1.5f;

const int TURN_MIN_PWM = 35;
const int TURN_MAX_PWM = 100;

// 목표 20도 이내 진입하면 미세조정으로 전환
const float TURN_FINE_HANDOFF = 20.0f;


// -----------------------------------------------------
// 2단계 : 미세 조정
// -----------------------------------------------------
const int TURN_FINE_PWM_MIN = 45;
const int TURN_FINE_PWM_MAX = 75;

const float TURN_FINE_PWM_K = 1.5f;


// 짧게 회전시키는 시간
const int TURN_FINE_PULSE_MIN = 30;
const int TURN_FINE_PULSE_MAX = 110;

const float TURN_FINE_PULSE_K = 4.0f;


// 움직인 후 차체가 완전히 멈출 때까지 대기
const int TURN_FINE_SETTLE_MS = 300;


// ±1도 이내면 완료
const float TURN_FINE_TOLERANCE = 1.0f;


// 안전 타임아웃
const unsigned long TURN_TIMEOUT_MS = 8000;


// =====================================================
// 회전 상태
// =====================================================
enum TurnPhase {

  TURN_COARSE,
  TURN_FINE_WAIT,
  TURN_FINE_MOVE
};

TurnPhase turnPhase = TURN_COARSE;

unsigned long turnStartTime = 0;
unsigned long turnPulseEndTime = 0;
unsigned long turnSettleEndTime = 0;


// +1 = +Yaw 방향
// -1 = -Yaw 방향
int turnDir = 0;


// =====================================================
// 주행 상태
// =====================================================
float targetYaw = 0.0f;

float yawError = 0.0f;

int yawCorrection = 0;


bool headingCaptureActive = false;

unsigned long headingCaptureStartTime = 0;

double headingSinSum = 0.0;
double headingCosSum = 0.0;

int headingSampleCount = 0;


enum MotionState {

  STATE_STOPPED,

  STATE_CAPTURE_FORWARD,
  STATE_CAPTURE_BACKWARD,

  STATE_FORWARD,
  STATE_BACKWARD,

  STATE_TURN_LEFT,
  STATE_TURN_RIGHT
};

MotionState motionState = STATE_STOPPED;


unsigned long lastControlTime = 0;
unsigned long lastPrintTime = 0;


// =====================================================
// 각도 정규화
// =====================================================
float normalizeAngle(float angle) {

  while (angle > 180.0f)
    angle -= 360.0f;

  while (angle < -180.0f)
    angle += 360.0f;

  return angle;
}


// =====================================================
// 상태 이름
// =====================================================
const char* getStateName() {

  switch (motionState) {

    case STATE_STOPPED:
      return "STOP";

    case STATE_CAPTURE_FORWARD:
      return "CAP_F";

    case STATE_CAPTURE_BACKWARD:
      return "CAP_B";

    case STATE_FORWARD:
      return "FWD";

    case STATE_BACKWARD:
      return "BACK";

    case STATE_TURN_LEFT:
      return "LEFT90";

    case STATE_TURN_RIGHT:
      return "RIGHT90";

    default:
      return "?";
  }
}


// =====================================================
// Serial + Bluetooth 출력
// =====================================================
void sendBoth(const char* message) {

  Serial.println(message);

  SerialBT.println(message);
}


// =====================================================
// MPU 인터럽트
// =====================================================
void IRAM_ATTR dmpDataReady() {

  mpuInterrupt = true;
}


// =====================================================
// 왼쪽 모터
//
// + = 전진
// - = 후진
// =====================================================
void setLeftMotor(int pwm) {

  pwm = constrain(pwm, -255, 255);


  if (pwm > 0) {

    // 반대 방향 PWM 먼저 OFF
    ledcWrite(L_LPWM, 0);

    ledcWrite(L_RPWM, pwm);
  }

  else if (pwm < 0) {

    ledcWrite(L_RPWM, 0);

    ledcWrite(L_LPWM, -pwm);
  }

  else {

    ledcWrite(L_RPWM, 0);

    ledcWrite(L_LPWM, 0);
  }
}


// =====================================================
// 오른쪽 모터
//
// 기존 실제 배선 기준
// + = 전진 = LPWM
// - = 후진 = RPWM
// =====================================================
void setRightMotor(int pwm) {

  pwm = constrain(pwm, -255, 255);


  if (pwm > 0) {

    ledcWrite(R_RPWM, 0);

    ledcWrite(R_LPWM, pwm);
  }

  else if (pwm < 0) {

    ledcWrite(R_LPWM, 0);

    ledcWrite(R_RPWM, -pwm);
  }

  else {

    ledcWrite(R_RPWM, 0);

    ledcWrite(R_LPWM, 0);
  }
}


// =====================================================
// 모든 PWM 정지
// =====================================================
void stopMotorOutput() {

  leftAppliedPWM = 0;
  rightAppliedPWM = 0;

  setLeftMotor(0);

  setRightMotor(0);
}


// =====================================================
// BTS 강제 비활성화
// =====================================================
void disableBTS() {

  stopMotorOutput();

  delay(10);


  digitalWrite(L_REN, LOW);
  digitalWrite(L_LEN, LOW);

  digitalWrite(R_REN, LOW);
  digitalWrite(R_LEN, LOW);
}


// =====================================================
// BTS 활성화
// =====================================================
void enableBTS() {

  // 활성화 전 PWM 0 재확인
  stopMotorOutput();

  delay(100);


  digitalWrite(L_REN, HIGH);
  digitalWrite(L_LEN, HIGH);

  digitalWrite(R_REN, HIGH);
  digitalWrite(R_LEN, HIGH);


  delay(100);


  // 활성화 직후에도 0 재확인
  stopMotorOutput();
}


// =====================================================
// 현재 상태 모터 출력
// =====================================================
void applyCurrentMotorOutput() {

  switch (motionState) {


    // ---------------------------------
    // 전진
    // ---------------------------------
    case STATE_FORWARD:

      setLeftMotor(
        leftAppliedPWM
      );

      setRightMotor(
        rightAppliedPWM
      );

      break;


    // ---------------------------------
    // 후진
    // ---------------------------------
    case STATE_BACKWARD:

      setLeftMotor(
        -leftAppliedPWM
      );

      setRightMotor(
        -rightAppliedPWM
      );

      break;


    default:

      break;
  }
}


// =====================================================
// 제자리 회전 실제 모터 출력
//
// ★ 기존 코드 방식 그대로 유지
// =====================================================
void driveTurn(int pwm, float errSign) {

  // +Yaw 방향
  if (errSign > 0.0f) {

    setLeftMotor(pwm);

    setRightMotor(-pwm);
  }

  // -Yaw 방향
  else {

    setLeftMotor(-pwm);

    setRightMotor(pwm);
  }
}


// =====================================================
// MPU6050 초기화
// =====================================================
bool initializeMPU6050() {

  sendBoth(
    "=== MPU6050 CALIBRATION START ==="
  );

  sendBoth(
    "DO NOT MOVE THE VEHICLE"
  );


  mpu.initialize();


  pinMode(
    MPU_INTERRUPT_PIN,
    INPUT
  );


  if (!mpu.testConnection()) {

    sendBoth(
      "MPU6050 CONNECTION FAILED"
    );

    return false;
  }


  devStatus =
    mpu.dmpInitialize();


  mpu.setXGyroOffset(0);
  mpu.setYGyroOffset(0);
  mpu.setZGyroOffset(0);

  mpu.setXAccelOffset(0);
  mpu.setYAccelOffset(0);
  mpu.setZAccelOffset(0);


  if (devStatus != 0) {

    Serial.print(
      "DMP INIT FAILED. CODE: "
    );

    Serial.println(
      devStatus
    );


    SerialBT.print(
      "DMP INIT FAILED. CODE: "
    );

    SerialBT.println(
      devStatus
    );


    return false;
  }


  mpu.CalibrateAccel(6);

  mpu.CalibrateGyro(6);


  mpu.PrintActiveOffsets();


  mpu.setDMPEnabled(true);


  attachInterrupt(

    digitalPinToInterrupt(
      MPU_INTERRUPT_PIN
    ),

    dmpDataReady,

    RISING
  );


  dmpReady = true;

  firstYawRead = true;


  sendBoth(
    "=== CALIBRATION COMPLETE ==="
  );


  return true;
}


// =====================================================
// Bluetooth 연결 시 Yaw 0도 보정 시작
// =====================================================
void startYawCalibration() {

  stopMotorOutput();


  motionState =
    STATE_STOPPED;


  headingCaptureActive =
    false;


  yawCalibrationActive =
    true;


  yawCalibrationStartTime =
    millis();


  yawCalibrationSinSum = 0.0;

  yawCalibrationCosSum = 0.0;

  yawCalibrationSampleCount = 0;


  sendBoth(
    "=== YAW ZERO CALIBRATION START ==="
  );

  sendBoth(
    "DO NOT MOVE THE VEHICLE"
  );
}


// =====================================================
// Yaw 캘리브레이션 샘플
// =====================================================
void processYawCalibrationSample(float rawYaw) {

  if (!yawCalibrationActive)
    return;


  double radian =

    rawYaw *
    M_PI /
    180.0;


  yawCalibrationSinSum +=
    sin(radian);


  yawCalibrationCosSum +=
    cos(radian);


  yawCalibrationSampleCount++;


  unsigned long elapsed =

    millis() -
    yawCalibrationStartTime;


  if (
    elapsed <
    YAW_CALIBRATION_TIME_MS
  )
    return;


  if (
    yawCalibrationSampleCount <
    YAW_CALIBRATION_MIN_SAMPLES
  )
    return;


  double averageRadian =

    atan2(

      yawCalibrationSinSum,

      yawCalibrationCosSum
    );


  manualYawOffset =

    averageRadian *
    180.0 /
    M_PI;


  currentYaw = 0.0f;

  targetYaw = 0.0f;

  yawError = 0.0f;

  yawCorrection = 0;

  yawErrorIntegral = 0.0f;

  lastYawError = 0.0f;


  firstYawRead = false;

  yawCalibrationActive = false;


  sendBoth(
    "=== YAW ZERO CALIBRATION COMPLETE ==="
  );

  sendBoth(
    "CURRENT HEADING = 0.0 DEG"
  );

  sendBoth(
    "PAD CONTROL READY"
  );
}


// =====================================================
// Bluetooth 연결 확인
// =====================================================
void checkBluetoothConnection() {

  bool connected =
    SerialBT.hasClient();


  if (
    connected &&
    !previousBluetoothConnected
  ) {

    startYawCalibration();
  }


  previousBluetoothConnected =
    connected;
}


// =====================================================
// Yaw 업데이트
// =====================================================
bool updateYaw() {

  if (!dmpReady)
    return false;


  if (
    !mpu.dmpGetCurrentFIFOPacket(
      fifoBuffer
    )
  )
    return false;


  mpu.dmpGetQuaternion(
    &q,
    fifoBuffer
  );


  mpu.dmpGetGravity(
    &gravity,
    &q
  );


  mpu.dmpGetYawPitchRoll(

    ypr,

    &q,

    &gravity
  );


  rawYawDegree =

    ypr[0] *
    180.0f /
    M_PI;


  // Bluetooth 연결 직후 0도 보정 중
  if (yawCalibrationActive) {

    processYawCalibrationSample(
      rawYawDegree
    );


    currentYaw = 0.0f;


    return true;
  }


  if (firstYawRead) {

    manualYawOffset =
      rawYawDegree;


    currentYaw =
      0.0f;


    firstYawRead =
      false;
  }

  else {

    currentYaw =

      normalizeAngle(

        rawYawDegree -
        manualYawOffset
      );
  }


  return true;
}


// =====================================================
// F/B 시작할 때 현재 방향 캡처
// =====================================================
void startHeadingCapture(bool forwardDirection) {

  stopMotorOutput();


  headingCaptureActive =
    true;


  headingCaptureStartTime =
    millis();


  headingSinSum = 0.0;

  headingCosSum = 0.0;

  headingSampleCount = 0;


  yawErrorIntegral = 0.0f;

  lastYawError = 0.0f;

  yawError = 0.0f;

  yawCorrection = 0;


  if (forwardDirection) {

    motionState =
      STATE_CAPTURE_FORWARD;


    sendBoth(
      "FORWARD: CAPTURING HEADING..."
    );
  }

  else {

    motionState =
      STATE_CAPTURE_BACKWARD;


    sendBoth(
      "BACKWARD: CAPTURING HEADING..."
    );
  }
}


// =====================================================
// Heading 샘플 수집
// =====================================================
void collectHeadingSample() {

  if (!headingCaptureActive)
    return;


  double radian =

    currentYaw *
    M_PI /
    180.0;


  headingSinSum +=
    sin(radian);


  headingCosSum +=
    cos(radian);


  headingSampleCount++;


  unsigned long elapsed =

    millis() -
    headingCaptureStartTime;


  if (
    elapsed <
    YAW_SETTLE_TIME_MS
  )
    return;


  if (
    headingSampleCount <
    MIN_YAW_SAMPLES
  )
    return;


  double averageRadian =

    atan2(

      headingSinSum,

      headingCosSum
    );


  targetYaw =

    normalizeAngle(

      averageRadian *
      180.0 /
      M_PI
    );


  headingCaptureActive =
    false;


  yawErrorIntegral = 0.0f;

  lastYawError = 0.0f;


  leftAppliedPWM =
    leftBaseSpeed;


  rightAppliedPWM =
    rightBaseSpeed;


  if (
    motionState ==
    STATE_CAPTURE_FORWARD
  ) {

    motionState =
      STATE_FORWARD;


    sendBoth(
      "HEADING SET -> FORWARD GO"
    );
  }

  else {

    motionState =
      STATE_BACKWARD;


    sendBoth(
      "HEADING SET -> BACKWARD GO"
    );
  }


  lastControlTime =
    millis();


  applyCurrentMotorOutput();
}


// =====================================================
// 직진 / 후진 YAW PID
// =====================================================
void updateStraightControl(float dt) {

  if (
    motionState != STATE_FORWARD &&
    motionState != STATE_BACKWARD
  )
    return;


  yawError =

    normalizeAngle(

      targetYaw -
      currentYaw
    );


  yawErrorIntegral +=

    yawError *
    dt;


  yawErrorIntegral =

    constrain(

      yawErrorIntegral,

      -YAW_INTEGRAL_LIMIT,

      YAW_INTEGRAL_LIMIT
    );


  float yawDerivative =

    (dt > 0.0f)

    ?

    (yawError -
     lastYawError)
    /
    dt

    :

    0.0f;


  lastYawError =
    yawError;


  float pidOutput =

    (KP_YAW * yawError)

    +

    (KI_YAW *
     yawErrorIntegral)

    +

    (KD_YAW *
     yawDerivative);


  int correction =

    constrain(

      (int)round(

        pidOutput *
        YAW_CORRECTION_SIGN
      ),

      -MAX_YAW_CORRECTION,

      MAX_YAW_CORRECTION
    );


  int directional =

    (motionState ==
     STATE_FORWARD)

    ?

    correction

    :

    -correction;


  yawCorrection =
    directional;


  leftAppliedPWM =

    constrain(

      leftBaseSpeed -
      directional,

      0,

      255
    );


  rightAppliedPWM =

    constrain(

      rightBaseSpeed +
      directional,

      0,

      255
    );


  applyCurrentMotorOutput();
}


// =====================================================
// 90도 회전 완료
// =====================================================
void finishTurn() {

  bool wasLeft =

    (
      motionState ==
      STATE_TURN_LEFT
    );


  stopMotorOutput();


  motionState =
    STATE_STOPPED;


  turnPhase =
    TURN_COARSE;


  yawCorrection = 0;


  yawError =

    normalizeAngle(

      targetYaw -
      currentYaw
    );


  if (wasLeft) {

    sendBoth(
      "LEFT 90 COMPLETE -> STOP"
    );
  }

  else {

    sendBoth(
      "RIGHT 90 COMPLETE -> STOP"
    );
  }
}


// =====================================================
// 90도 회전 시작
//
// ★ 기존 누적 오차 방지 로직 유지
// =====================================================
void startTurn90(bool turnLeft) {

  if (yawCalibrationActive) {

    sendBoth(
      "YAW CALIBRATION IN PROGRESS - WAIT"
    );

    return;
  }


  if (
    firstYawRead ||
    !dmpReady
  ) {

    sendBoth(
      "YAW NOT READY - TRY AGAIN"
    );

    return;
  }


  headingCaptureActive =
    false;


  yawErrorIntegral = 0.0f;

  lastYawError = 0.0f;

  yawCorrection = 0;


  turnPhase =
    TURN_COARSE;


  turnStartTime =
    millis();


  // -----------------------------------------------
  // 이전 목표각과 현재각이 5도 이내라면
  // 현재의 약간 삐뚤어진 각도가 아니라
  // 이전 목표각 기준으로 다시 +90 / -90
  // -----------------------------------------------

  float baseYaw =
    currentYaw;


  if (

    fabs(

      normalizeAngle(

        targetYaw -
        currentYaw
      )

    )

    <= 5.0f
  ) {

    baseYaw =
      targetYaw;
  }


  if (turnLeft) {

    motionState =
      STATE_TURN_LEFT;


    targetYaw =

      normalizeAngle(

        baseYaw +
        LEFT_TURN_ANGLE
      );


    sendBoth(
      "LEFT 90 START"
    );
  }

  else {

    motionState =
      STATE_TURN_RIGHT;


    targetYaw =

      normalizeAngle(

        baseYaw +
        RIGHT_TURN_ANGLE
      );


    sendBoth(
      "RIGHT 90 START"
    );
  }


  float initErr =

    normalizeAngle(

      targetYaw -
      currentYaw
    );


  turnDir =

    (initErr >= 0.0f)

    ?

    1

    :

    -1;


  yawError =
    initErr;
}


// =====================================================
// ★ 90도 회전 제어
//
// 1. 빠르게 접근
// 2. 목표 근처에서 정지
// 3. 0.3초 기다림
// 4. 실제 각도 확인
// 5. ±1도 밖이면 짧은 펄스
// 6. 다시 정지 후 확인
// =====================================================
void updateTurnControl() {

  if (
    motionState != STATE_TURN_LEFT &&
    motionState != STATE_TURN_RIGHT
  )
    return;


  // -----------------------------------------------
  // 안전 타임아웃
  // -----------------------------------------------

  if (

    millis() -
    turnStartTime

    >=

    TURN_TIMEOUT_MS
  ) {

    stopMotorOutput();


    motionState =
      STATE_STOPPED;


    turnPhase =
      TURN_COARSE;


    sendBoth(
      "TURN TIMEOUT -> STOP"
    );


    return;
  }


  yawError =

    normalizeAngle(

      targetYaw -
      currentYaw
    );


  float absErr =
    fabs(yawError);


  bool crossed =

    (
      turnDir > 0 &&
      yawError <= 0.0f
    )

    ||

    (
      turnDir < 0 &&
      yawError >= 0.0f
    );


  switch (turnPhase) {


    // =================================================
    // 1단계 : 거친 접근
    // =================================================

    case TURN_COARSE:
    {

      // 목표까지 20도 이하
      if (
        absErr <=
        TURN_FINE_HANDOFF
      ) {

        // 반대 방향으로 아주 짧게 브레이크
        float brakeSign =

          (turnDir > 0)

          ?

          -1.0f

          :

          1.0f;


        driveTurn(
          40,
          brakeSign
        );


        delay(30);


        stopMotorOutput();


        // 차가 실제로 멈출 시간
        turnPhase =
          TURN_FINE_WAIT;


        turnSettleEndTime =

          millis() +
          TURN_FINE_SETTLE_MS;


        return;
      }


      int pwm =

        constrain(

          (int)round(

            TURN_KP *
            absErr
          ),

          TURN_MIN_PWM,

          TURN_MAX_PWM
        );


      int leftPWM =

        min(
          pwm,
          leftBaseSpeed
        );


      int rightPWM =

        min(
          pwm,
          rightBaseSpeed
        );


      int usePWM =

        min(
          leftPWM,
          rightPWM
        );


      leftAppliedPWM =
        usePWM;


      rightAppliedPWM =
        usePWM;


      driveTurn(
        usePWM,
        yawError
      );


      break;
    }


    // =================================================
    // 2단계 : 정지 후 실제 각도 확인
    // =================================================

    case TURN_FINE_WAIT:
    {

      stopMotorOutput();


      // 아직 0.3초 안 지났으면 기다림
      if (
        millis() <
        turnSettleEndTime
      )
        return;


      // -----------------------------------------------
      // ±1도 이내면 완료
      // -----------------------------------------------

      if (
        absErr <=
        TURN_FINE_TOLERANCE
      ) {

        finishTurn();

        return;
      }


      // -----------------------------------------------
      // 아직 오차가 있으면
      // 짧은 펄스 시간 계산
      // -----------------------------------------------

      int pulseMs =

        constrain(

          (int)(

            absErr *
            TURN_FINE_PULSE_K
          ),

          TURN_FINE_PULSE_MIN,

          TURN_FINE_PULSE_MAX
        );


      // 오차가 클수록 조금 더 강하게
      int finePWM =

        constrain(

          TURN_FINE_PWM_MIN

          +

          (int)(

            absErr *
            TURN_FINE_PWM_K
          ),

          TURN_FINE_PWM_MIN,

          TURN_FINE_PWM_MAX
        );


      turnPhase =
        TURN_FINE_MOVE;


      turnPulseEndTime =

        millis() +
        pulseMs;


      leftAppliedPWM =
        finePWM;


      rightAppliedPWM =
        finePWM;


      driveTurn(
        finePWM,
        yawError
      );


      break;
    }


    // =================================================
    // 2단계 : 미세 펄스 회전
    // =================================================

    case TURN_FINE_MOVE:
    {

      // 펄스 시간이 끝났거나
      // 목표를 넘어갔거나
      // ±1도 안으로 들어왔으면 즉시 정지

      if (

        millis() >=
        turnPulseEndTime

        ||

        crossed

        ||

        absErr <=
        TURN_FINE_TOLERANCE
      ) {

        stopMotorOutput();


        turnPhase =
          TURN_FINE_WAIT;


        turnSettleEndTime =

          millis() +
          TURN_FINE_SETTLE_MS;
      }


      break;
    }
  }
}


// =====================================================
// 차량 정지
// =====================================================
void stopVehicle() {

  headingCaptureActive =
    false;


  motionState =
    STATE_STOPPED;


  turnPhase =
    TURN_COARSE;


  yawErrorIntegral =
    0.0f;


  lastYawError =
    0.0f;


  yawError =
    0.0f;


  yawCorrection =
    0;


  stopMotorOutput();


  sendBoth("STOP");
}


// =====================================================
// Bluetooth 명령
//
// ★ L/R 매핑도 네 원래 코드 그대로 유지
// =====================================================
void handleCommand(char command) {

  // Yaw 0도 보정 중에는 이동 금지
  if (yawCalibrationActive) {

    if (
      command == 'X' ||
      command == 'x'
    ) {

      stopVehicle();
    }

    else {

      sendBoth(
        "YAW CALIBRATION IN PROGRESS - WAIT"
      );
    }


    return;
  }


  switch (command) {


    // ---------------------------------
    // 전진
    // ---------------------------------

    case 'F':
    case 'f':

      if (

        motionState !=
        STATE_FORWARD

        &&

        motionState !=
        STATE_CAPTURE_FORWARD
      ) {

        startHeadingCapture(true);
      }

      break;


    // ---------------------------------
    // 후진
    // ---------------------------------

    case 'B':
    case 'b':

      if (

        motionState !=
        STATE_BACKWARD

        &&

        motionState !=
        STATE_CAPTURE_BACKWARD
      ) {

        startHeadingCapture(false);
      }

      break;


    // ---------------------------------
    // 패드 L
    //
    // ★ 기존 코드 매핑 그대로
    // ---------------------------------

    case 'L':
    case 'l':

      if (

        motionState !=
        STATE_TURN_LEFT

        &&

        motionState !=
        STATE_TURN_RIGHT
      ) {

        stopMotorOutput();

        delay(20);

        startTurn90(false);
      }

      break;


    // ---------------------------------
    // 패드 R
    //
    // ★ 기존 코드 매핑 그대로
    // ---------------------------------

    case 'R':
    case 'r':

      if (

        motionState !=
        STATE_TURN_LEFT

        &&

        motionState !=
        STATE_TURN_RIGHT
      ) {

        stopMotorOutput();

        delay(20);

        startTurn90(true);
      }

      break;


    // ---------------------------------
    // 정지
    // ---------------------------------

    case 'X':
    case 'x':

      stopVehicle();

      break;


    // ---------------------------------
    // 왼쪽 PWM +10
    // ---------------------------------

    case 'I':
    case 'i':

      leftBaseSpeed =

        constrain(

          leftBaseSpeed + 10,

          0,

          255
        );


      SerialBT.print(
        "LEFT BASE PWM: "
      );

      SerialBT.println(
        leftBaseSpeed
      );

      break;


    // ---------------------------------
    // 왼쪽 PWM -10
    // ---------------------------------

    case 'K':
    case 'k':

      leftBaseSpeed =

        constrain(

          leftBaseSpeed - 10,

          0,

          255
        );


      SerialBT.print(
        "LEFT BASE PWM: "
      );

      SerialBT.println(
        leftBaseSpeed
      );

      break;


    // ---------------------------------
    // 오른쪽 PWM +10
    // ---------------------------------

    case 'O':
    case 'o':

      rightBaseSpeed =

        constrain(

          rightBaseSpeed + 10,

          0,

          255
        );


      SerialBT.print(
        "RIGHT BASE PWM: "
      );

      SerialBT.println(
        rightBaseSpeed
      );

      break;


    // ---------------------------------
    // 오른쪽 PWM -10
    // ---------------------------------

    case 'P':
    case 'p':

      rightBaseSpeed =

        constrain(

          rightBaseSpeed - 10,

          0,

          255
        );


      SerialBT.print(
        "RIGHT BASE PWM: "
      );

      SerialBT.println(
        rightBaseSpeed
      );

      break;


    default:

      return;
  }
}


// =====================================================
// 명령 읽기
// =====================================================
void readCommands() {

  while (
    SerialBT.available()
  ) {

    char command =
      SerialBT.read();


    if (
      command != '\n' &&
      command != '\r'
    ) {

      handleCommand(
        command
      );
    }
  }


  while (
    Serial.available()
  ) {

    char command =
      Serial.read();


    if (
      command != '\n' &&
      command != '\r'
    ) {

      handleCommand(
        command
      );
    }
  }
}


// =====================================================
// ★신규 - TCP 명령 읽기/처리
//
// BT 의 handleCommand(char) 와 별개 채널이다. 여기서도 원본 함수
// (startHeadingCapture/startTurn90/stopVehicle/leftBaseSpeed 등)를
// 그대로 재사용한다 - 새 로직을 만들지 않고 기존 진입점만 하나 늘렸다.
// =====================================================

// 아주 단순한 한 단계 JSON 파서 - rescue_sensor_node.ino 의 jsonStr/
// jsonNum 과 같은 패턴(그쪽도 각 파일이 자기 사본을 갖고 있어 이 파일도
// 독립적으로 하나 둔다 - 스케치 간 공유 헤더 없이 각자 완결되게 하는
// 이 저장소의 기존 관례를 따름).
bool jsonStr(const String& s, const char* key, String& out) {

  String pat = String("\"") + key + "\"";

  int ki = s.indexOf(pat);

  if (ki < 0) return false;

  int qi = s.indexOf('"', ki + pat.length() + 1);

  if (qi < 0) return false;

  int q2 = s.indexOf('"', qi + 1);

  if (q2 < 0) return false;

  out = s.substring(qi + 1, q2);

  return true;
}

bool jsonNum(const String& s, const char* key, float& out) {

  String pat = String("\"") + key + "\"";

  int ki = s.indexOf(pat);

  if (ki < 0) return false;

  int ci = s.indexOf(':', ki + pat.length());

  if (ci < 0) return false;

  out = s.substring(ci + 1).toFloat();

  return true;
}

void maintainWifi() {

  if (WiFi.status() == WL_CONNECTED) return;

  unsigned long now = millis();

  if (now - lastWifiRetryMs < WIFI_RETRY_MS) return;

  lastWifiRetryMs = now;

  WiFi.begin(WIFI_SSID_TCP, WIFI_PASS_TCP);
}

void handleTcpCommand(const String& line) {

  String cmd;

  if (!jsonStr(line, "cmd", cmd)) return;

  if (cmd == "forward" || cmd == "backward") {

    float speed = -1, durationMs = 0;

    if (jsonNum(line, "speed", speed)) {

      int sp = constrain((int)speed, 0, 255);

      leftBaseSpeed = sp;

      rightBaseSpeed = sp;
    }

    jsonNum(line, "duration_ms", durationMs);


    if (

      motionState == STATE_STOPPED

      ||

      (cmd == "forward" && motionState != STATE_FORWARD
                         && motionState != STATE_CAPTURE_FORWARD)

      ||

      (cmd == "backward" && motionState != STATE_BACKWARD
                          && motionState != STATE_CAPTURE_BACKWARD)
    ) {

      startHeadingCapture(cmd == "forward");
    }


    if (durationMs > 0) {

      // ★마감시각을 여기서 바로 확정하지 않는다 - 헤딩 캡처(최소 300ms)
      //   가 끝나고 실제로 STATE_FORWARD/BACKWARD 로 들어간 순간부터
      //   세야 한다. checkTcpAutoStop() 이 그 전환을 감지해서 확정한다.
      tcpAutoStopRequested  = true;

      tcpAutoStopArmed      = false;

      tcpAutoStopDurationMs = (unsigned long)durationMs;
    } else {

      tcpAutoStopRequested = false;

      tcpAutoStopArmed     = false;
    }

    if (tcpCmdClient) tcpCmdClient.println("{\"type\":\"ack\",\"cmd\":\"" + cmd + "\"}");
  }

  else if (cmd == "turn_left" || cmd == "turn_right") {

    // ★duration_ms 는 안 본다 - updateTurnControl() 이 자이로로 스스로
    //   끝을 판단한다(원본 그대로, 8초 타임아웃 포함).
    tcpAutoStopRequested = false;

    tcpAutoStopArmed = false;

    if (

      motionState != STATE_TURN_LEFT &&
      motionState != STATE_TURN_RIGHT
    ) {

      stopMotorOutput();

      delay(20);

      startTurn90(cmd == "turn_left");
    }

    if (tcpCmdClient) tcpCmdClient.println("{\"type\":\"ack\",\"cmd\":\"" + cmd + "\"}");
  }

  else if (cmd == "stop") {

    tcpAutoStopRequested = false;

    tcpAutoStopArmed = false;

    stopVehicle();

    if (tcpCmdClient) tcpCmdClient.println("{\"type\":\"ack\",\"cmd\":\"stop\"}");
  }

  else if (cmd == "ping") {

    if (tcpCmdClient) tcpCmdClient.println("{\"type\":\"pong\"}");
  }
}

void pollTcpCommand() {

  if (!tcpCmdClient || !tcpCmdClient.connected()) {

    WiFiClient nc = tcpCmdServer.available();

    if (nc) {

      tcpCmdClient = nc;

      sendBoth("[TCP] 노트북 연결됨");
    }
  }

  while (tcpCmdClient && tcpCmdClient.connected() && tcpCmdClient.available()) {

    char c = tcpCmdClient.read();

    if (c == '\n') {

      tcpLineBuf.trim();

      if (tcpLineBuf.length() > 0) handleTcpCommand(tcpLineBuf);

      tcpLineBuf = "";
    } else if (c != '\r') {

      tcpLineBuf += c;

      if (tcpLineBuf.length() > 220) tcpLineBuf = "";   // 폭주 방지
    }
  }
}

// forward/backward 의 duration_ms 경과를 감시해서 자동으로 stopVehicle()
// 한다. BT 의 'F'/'B'/'X' 흐름과는 독립 - BT 로 조종 중일 땐 duration_ms
// 를 안 보내니 이 타이머 자체가 안 걸린다.
//
// ★2단계: 명령 직후엔 헤딩 캡처(최소 300ms, 정지 상태)만 진행되고 실제
//   주행은 아직 시작 안 했다(원본 startHeadingCapture/collectHeadingSample
//   그대로, 안 건드림). 여기서 그 전환(STATE_CAPTURE_* -> STATE_FORWARD/
//   BACKWARD)을 감지해서 그 순간부터 duration_ms 를 센다 - 명령 시각부터
//   세면 짧은 duration_ms(100~200ms 등)가 캡처 중에 끝나버려 "출발도
//   못 하고 정지"로 왜곡된다(직진 K/C 실측을 여기서 하므로 정확도가
//   중요하다).
void checkTcpAutoStop() {

  if (tcpAutoStopRequested && !tcpAutoStopArmed) {

    if (motionState == STATE_FORWARD || motionState == STATE_BACKWARD) {

      tcpAutoStopArmed = true;

      tcpAutoStopAtMs = millis() + tcpAutoStopDurationMs;
    }

    // 캡처가 취소되거나(예: 다른 명령이 끼어듦) 넘어가 있으면 그냥 대기.
    // stop/turn 핸들러가 이미 tcpAutoStopRequested 를 false 로 내린다.
  }

  if (!tcpAutoStopArmed) return;

  if (millis() < tcpAutoStopAtMs) return;

  tcpAutoStopArmed = false;

  tcpAutoStopRequested = false;

  stopVehicle();
}


// =====================================================
// 상태 출력
// =====================================================
void printStatus() {

  char buf[130];


  snprintf(

    buf,

    sizeof(buf),

    "[%s] Y:%.1f T:%.1f E:%.1f | PWM L:%d R:%d",

    getStateName(),

    currentYaw,

    targetYaw,

    yawError,

    leftAppliedPWM,

    rightAppliedPWM
  );


  Serial.println(buf);

  SerialBT.println(buf);
}


// =====================================================
// SETUP
// =====================================================
void setup() {

  Serial.begin(115200);


  // ===================================================
  // ★ 1. 가장 먼저 BTS ENABLE LOW
  // ===================================================

  pinMode(
    L_REN,
    OUTPUT
  );

  pinMode(
    L_LEN,
    OUTPUT
  );

  pinMode(
    R_REN,
    OUTPUT
  );

  pinMode(
    R_LEN,
    OUTPUT
  );


  digitalWrite(
    L_REN,
    LOW
  );

  digitalWrite(
    L_LEN,
    LOW
  );


  digitalWrite(
    R_REN,
    LOW
  );

  digitalWrite(
    R_LEN,
    LOW
  );


  // ===================================================
  // ★ 2. PWM 초기화
  // ===================================================

  ledcAttach(
    L_RPWM,
    PWM_FREQUENCY,
    PWM_RESOLUTION
  );


  ledcAttach(
    L_LPWM,
    PWM_FREQUENCY,
    PWM_RESOLUTION
  );


  ledcAttach(
    R_RPWM,
    PWM_FREQUENCY,
    PWM_RESOLUTION
  );


  ledcAttach(
    R_LPWM,
    PWM_FREQUENCY,
    PWM_RESOLUTION
  );


  // ===================================================
  // ★ 3. 모든 PWM 0
  // ===================================================

  stopMotorOutput();


  delay(500);


  // ===================================================
  // Bluetooth
  // ===================================================

  SerialBT.begin(
    BLUETOOTH_NAME
  );


  // ===================================================
  // ★신규 - TCP(WiFi) 시작
  //
  // 논블로킹 - WiFi.begin() 은 연결 완료를 기다리지 않는다. 기다리면
  // 그동안 BT 도 아래 BTS 활성화 시퀀스도 전부 멎는다. 연결 여부는
  // loop() 의 maintainWifi() 가 5초 간격으로 폴링/재시도한다.
  // BTS 활성화(맨 아래 enableBTS())보다 먼저 있어도 무방 - 모터 핀은
  // 이미 위에서 LOW 로 잠가뒀다.
  // ===================================================

  WiFi.mode(WIFI_STA);

  WiFi.begin(WIFI_SSID_TCP, WIFI_PASS_TCP);

  tcpCmdServer.begin();


  // ===================================================
  // I2C
  // ===================================================

  Wire.begin(
    21,
    22
  );


  Wire.setClock(
    400000
  );


  // ===================================================
  // MPU6050
  //
  // 이 과정 동안 BTS는 계속 비활성
  // ===================================================

  if (
    !initializeMPU6050()
  ) {

    disableBTS();


    sendBoth(
      "INITIALIZATION STOPPED"
    );


    while (true) {

      delay(1000);
    }
  }


  // ===================================================
  // ★ 4. MPU 초기화 후에도 PWM 0 재확인
  // ===================================================

  stopMotorOutput();


  delay(500);


  // ===================================================
  // ★ 5. 마지막에 BTS 활성화
  // ===================================================

  enableBTS();


  motionState =
    STATE_STOPPED;


  lastControlTime =
    millis();


  lastPrintTime =
    millis();


  sendBoth(
    "=============================="
  );


  sendBoth(
    "SYSTEM READY"
  );


  sendBoth(
    "MOTOR STOPPED"
  );


  sendBoth(
    "F/B/L/R/X READY (BT) | forward/backward/turn_left/turn_right/stop (TCP)"
  );


  sendBoth(
    "=============================="
  );
}


// =====================================================
// LOOP
// =====================================================
void loop() {

  // ===================================================
  // ★신규 - TCP 채널. 전부 논블로킹(bt_tcp_coexist_test 로 검증된 패턴).
  // ===================================================

  maintainWifi();

  pollTcpCommand();

  checkTcpAutoStop();


  // Bluetooth 연결 감지
  checkBluetoothConnection();


  // MPU6050 Yaw 업데이트
  bool yawUpdated =
    updateYaw();


  // 패드 / Serial 명령
  readCommands();


  // 전진 / 후진 시작 방향 캡처
  if (

    headingCaptureActive

    &&

    yawUpdated
  ) {

    collectHeadingSample();
  }


  unsigned long now =
    millis();


  // ===================================================
  // 제어 주기
  // ===================================================

  if (

    now -
    lastControlTime

    >=

    CONTROL_INTERVAL_MS
  ) {

    float dt =

      (now -
       lastControlTime)

      /

      1000.0f;


    lastControlTime =
      now;


    // 전진 / 후진 PID
    if (

      motionState ==
      STATE_FORWARD

      ||

      motionState ==
      STATE_BACKWARD
    ) {

      updateStraightControl(
        dt
      );
    }


    // ★ 90도 자동 회전 + 미세조정
    if (

      motionState ==
      STATE_TURN_LEFT

      ||

      motionState ==
      STATE_TURN_RIGHT
    ) {

      updateTurnControl();
    }
  }


  // ===================================================
  // 상태 출력
  // ===================================================

  if (

    now -
    lastPrintTime

    >=

    PRINT_INTERVAL_MS
  ) {

    lastPrintTime =
      now;


    printStatus();
  }
}

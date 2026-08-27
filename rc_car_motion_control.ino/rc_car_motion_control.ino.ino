#include <Wire.h>
#include "BluetoothSerial.h"
#include "I2Cdev.h"

#include "MPU6050_6Axis_MotionApps20.h"

// =====================================================
// 함수 선언
// =====================================================
void finishTurn();
void resetAll();
void IRAM_ATTR leftEncoderISR();
void IRAM_ATTR rightEncoderISR();
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

// =====================================================
// Bluetooth
// =====================================================
BluetoothSerial SerialBT;

const char* BLUETOOTH_NAME = "ESP32_RC";


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
// ENCODER (전/후진 보정용)
// =====================================================
#define L_ENCODER_A 34
#define L_ENCODER_B 35
#define R_ENCODER_A 36
#define R_ENCODER_B 39

volatile long leftEncoderCount = 0;
volatile long rightEncoderCount = 0;

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

// 전/후진 전용 기본 PWM (잘 됐던 코드 값)
const int STRAIGHT_BASE_PWM = 95;

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
// ENCODER CONTROL - 첫 번째 전/후진 코드 값 그대로
// =====================================================
float KP_ENCODER = 0.08f;
const int MAX_ENCODER_CORRECTION = 8;

long previousLeftEncoder = 0;
long previousRightEncoder = 0;
long deltaLeft = 0;
long deltaRight = 0;
float encoderError = 0.0f;
int encoderCorrection = 0;

const unsigned long STRAIGHT_CONTROL_INTERVAL_MS = 50;
unsigned long lastStraightControlTime = 0;

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
// ENCODER ISR
// =====================================================
void IRAM_ATTR leftEncoderISR() {
  if (digitalRead(L_ENCODER_B)) leftEncoderCount++;
  else leftEncoderCount--;
}

void IRAM_ATTR rightEncoderISR() {
  if (digitalRead(R_ENCODER_B)) rightEncoderCount++;
  else rightEncoderCount--;
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
// RESET (C)
// =====================================================
void resetAll() {

  stopMotorOutput();
  motionState = STATE_STOPPED;
  headingCaptureActive = false;
  turnPhase = TURN_COARSE;

  noInterrupts();
  leftEncoderCount = 0;
  rightEncoderCount = 0;
  interrupts();

  previousLeftEncoder = 0;
  previousRightEncoder = 0;
  deltaLeft = 0;
  deltaRight = 0;
  encoderError = 0;
  encoderCorrection = 0;

  yawError = 0;
  lastYawError = 0;
  yawErrorIntegral = 0;
  yawCorrection = 0;

  // 회전/캡처 관련 상태도 초기화
  turnDir = 0;
  turnStartTime = 0;
  turnPulseEndTime = 0;
  turnSettleEndTime = 0;
  headingCaptureStartTime = 0;
  headingSinSum = 0.0;
  headingCosSum = 0.0;
  headingSampleCount = 0;

  // 현재 차체 방향을 새로운 0도로 사용
  manualYawOffset = rawYawDegree;
  currentYaw = 0;
  targetYaw = 0;
  firstYawRead = false;

  lastStraightControlTime = millis();

  sendBoth("====================");
  sendBoth("ALL VALUES RESET");
  sendBoth("YAW = 0");
  sendBoth("ENCODER L = 0");
  sendBoth("ENCODER R = 0");
  sendBoth("====================");
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
    STRAIGHT_BASE_PWM;


  rightAppliedPWM =
    STRAIGHT_BASE_PWM;


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

  // 첫 번째 전/후진 코드처럼 출발 순간 엔코더 기준점을 현재값으로 맞춤
  noInterrupts();
  previousLeftEncoder = leftEncoderCount;
  previousRightEncoder = rightEncoderCount;
  interrupts();
  deltaLeft = 0;
  deltaRight = 0;
  encoderError = 0;
  encoderCorrection = 0;
  lastStraightControlTime = millis();


  applyCurrentMotorOutput();
}


// =====================================================
// 직진 / 후진 YAW PID
// =====================================================
void updateStraightControl(float dtUnused) {

  if (
    motionState != STATE_FORWARD &&
    motionState != STATE_BACKWARD
  ) return;

  unsigned long now = millis();
  if (now - lastStraightControlTime < STRAIGHT_CONTROL_INTERVAL_MS) return;

  float dt = (now - lastStraightControlTime) / 1000.0f;
  lastStraightControlTime = now;

  // ---------------------------------------------------
  // Encoder - 첫 번째 전/후진 코드 방식
  // ---------------------------------------------------
  noInterrupts();
  long currentLeft = leftEncoderCount;
  long currentRight = rightEncoderCount;
  interrupts();

  deltaLeft = currentLeft - previousLeftEncoder;
  deltaRight = currentRight - previousRightEncoder;
  previousLeftEncoder = currentLeft;
  previousRightEncoder = currentRight;

  long speedLeft = abs(deltaLeft);
  long speedRight = abs(deltaRight);
  encoderError = speedLeft - speedRight;

  encoderCorrection = constrain(
    (int)round(KP_ENCODER * encoderError),
    -MAX_ENCODER_CORRECTION,
    MAX_ENCODER_CORRECTION
  );

  // ---------------------------------------------------
  // MPU Yaw PID - 첫 번째 전/후진 코드 방식
  // ---------------------------------------------------
  yawError = normalizeAngle(targetYaw - currentYaw);
  yawErrorIntegral += yawError * dt;
  yawErrorIntegral = constrain(yawErrorIntegral, -40.0f, 40.0f);

  float derivative = (dt > 0.0f) ? (yawError - lastYawError) / dt : 0.0f;
  lastYawError = yawError;

  float yawPID =
    KP_YAW * yawError +
    KI_YAW * yawErrorIntegral +
    KD_YAW * derivative;

  int rawYawCorrection = constrain(
    (int)round(yawPID * YAW_CORRECTION_SIGN),
    -MAX_YAW_CORRECTION,
    MAX_YAW_CORRECTION
  );

  // 후진은 Yaw 보정 방향 반전
  int directionalYaw =
    (motionState == STATE_FORWARD)
    ? rawYawCorrection
    : -rawYawCorrection;

  yawCorrection = directionalYaw;

  // ---------------------------------------------------
  // 최종 PWM - BASE 95, 범위 70~130
  // ---------------------------------------------------
  leftAppliedPWM = constrain(
    STRAIGHT_BASE_PWM - encoderCorrection - directionalYaw,
    70, 130
  );

  rightAppliedPWM = constrain(
    STRAIGHT_BASE_PWM + encoderCorrection + directionalYaw,
    70, 130
  );

  if (motionState == STATE_FORWARD) {
    setLeftMotor(leftAppliedPWM);
    setRightMotor(rightAppliedPWM);
  } else {
    setLeftMotor(-leftAppliedPWM);
    setRightMotor(-rightAppliedPWM);
  }
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
  headingCaptureActive = false;
  motionState = STATE_STOPPED;
  turnPhase = TURN_COARSE;

  yawErrorIntegral = 0.0f;
  lastYawError = 0.0f;
  yawError = 0.0f;
  yawCorrection = 0;

  deltaLeft = 0;
  deltaRight = 0;
  encoderError = 0;
  encoderCorrection = 0;

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
    // 전체 값 리셋
    // ---------------------------------
    case 'C':
    case 'c':
      resetAll();
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
// 상태 출력
// =====================================================
void printStatus() {

  long L, R;
  noInterrupts();
  L = leftEncoderCount;
  R = rightEncoderCount;
  interrupts();

  char buf[190];
  snprintf(
    buf, sizeof(buf),
    "[%s] Y:%.1f T:%.1f E:%.1f | ENC L:%ld R:%ld | dL:%ld dR:%ld EC:%d | PWM L:%d R:%d",
    getStateName(),
    currentYaw, targetYaw, yawError,
    L, R,
    abs(deltaLeft), abs(deltaRight), encoderCorrection,
    leftAppliedPWM, rightAppliedPWM
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
  // Encoder
  // ===================================================
  pinMode(L_ENCODER_A, INPUT);
  pinMode(L_ENCODER_B, INPUT);
  pinMode(R_ENCODER_A, INPUT);
  pinMode(R_ENCODER_B, INPUT);

  attachInterrupt(digitalPinToInterrupt(L_ENCODER_A), leftEncoderISR, RISING);
  attachInterrupt(digitalPinToInterrupt(R_ENCODER_A), rightEncoderISR, RISING);


  // ===================================================
  // Bluetooth
  // ===================================================

  SerialBT.begin(
    BLUETOOTH_NAME
  );


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
    "F/B/L/R/X/C READY"
  );


  sendBoth(
    "=============================="
  );
}


// =====================================================
// LOOP
// =====================================================
void loop() {

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
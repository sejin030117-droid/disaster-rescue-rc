/*
  rescue_sensor_node.ino -- ESP32 센서 노드 (이동 제어 제외)

  [이 파일이 하는 일]
    1. 통신구조 확립  : 노트북과 TCP 2채널 (아래 [통신] 참고)
    2. 서보 스윕      : ToF/카메라 마운트를 좌우로 훑는다
    3. 센서 -> 노트북 : ToF 거리 + 서보각 + yaw + 온도를 JSON 한 줄씩 전송
    4. 노트북 -> ESP32: "그 각도를 겨냥해라"(aim) 명령 수신
    5. 구간 스윕      : from~to 를 step 간격으로 훑으며 찍기(scan)
                        -> 스윕 파라미터는 아래 [사용자 조정 구역] 전역변수

  [이 파일에 없는 것 — 의도적]
    모터 구동(이동). BTS7960 핀은 정의만 하고 부팅 시 EN 을 LOW 로 내려
    "확실히 안 도는" 상태로만 둔다. 이동 로직은 별도 파일에서 다룬다.

  [통신] robot_config.py 와 반드시 일치해야 함
    - CMD_PORT   8890 : ESP32 가 "서버". 노트북이 붙어서 명령을 보낸다.
    - STREAM_PORT 9999 : 노트북이 "서버". ESP32 가 붙어서 데이터를 보낸다.
    ★ aim/scan 의 "응답"도 명령 소켓(8890)이 아니라 스트림 소켓(9999)으로
      돌아간다. 파이썬 tof_commander.py 가 그렇게 짜여 있다.

  [각도 규약]
    오프셋각(offset) = 로봇 정면이 0, 이 값을 파이썬과 주고받는다.
    물리 서보각 = SERVO_CENTER + SERVO_SIGN * offset   (90 + 1*offset)
    -> offset -80~+80 이 물리 서보 10~170 에 대응.
    robot_config.py 의 ESP32_SERVO_* 사본과 항상 같게 유지할 것.

  [필요 라이브러리 — 라이브러리 매니저에서 설치]
    ESP32Servo          (Kevin Harrington)
    Adafruit_VL53L0X    (Adafruit)
    MPU6050_light       (rfetick)
    DHT sensor library  (Adafruit) + Adafruit Unified Sensor

  [업로드 설정]
    보드: ESP32 Dev Module / 업로드 속도 921600 / 시리얼 115200
*/

#include <WiFi.h>
#include <Wire.h>
#include <ESP32Servo.h>
#include <Adafruit_VL53L0X.h>
#include <MPU6050_light.h>
#include <DHT.h>

// ==========================================================================
// [사용자 조정 구역] — 스윕 동작을 바꾸고 싶으면 여기만 고치면 된다
// ==========================================================================
float    SWEEP_MIN_OFF    = -80.0;   // 연속 스윕 좌측 한계 (오프셋각)
float    SWEEP_MAX_OFF    =  80.0;   // 연속 스윕 우측 한계
float    SWEEP_STEP_DEG   =   5.0;   // ★ 한 스텝 각도. 이 값마다 한 번 찍는다.
uint16_t SWEEP_SETTLE_MS  =    60;   // 서보 이동 후 안정화 대기(ms)
                                     //   너무 짧으면 흔들리는 중에 측정된다.
uint8_t  AIM_SAMPLES      =     5;   // aim 1회당 측정 횟수(중앙값을 씀)
uint16_t AIM_SETTLE_MS    =   180;   // aim 은 크게 움직일 수 있어 더 넉넉히

// ToF 유효범위 — ★ robot_config.py 의 DIST_MIN_MM/DIST_MAX_MM 과 같게 유지
const uint16_t TOF_MIN_MM = 30;
const uint16_t TOF_MAX_MM = 1800;

// 서보 각도 규약 — ★ robot_config.py 의 ESP32_SERVO_* 사본과 같게 유지
const float SERVO_CENTER   = 90.0;
const float SERVO_SIGN     = 1.0;
const float SERVO_MIN_OFF  = -80.0;
const float SERVO_MAX_OFF  =  80.0;

// ==========================================================================
// 네트워크
// ==========================================================================
const char*    WIFI_SSID   = "pjh";              // 라즈베리파이 핫스팟
const char*    WIFI_PASS   = "12345678";
// ★ 라파 핫스팟 대역이 192.168.137.x 가 아닐 가능성이 높다. 연결 후 시리얼에
//   찍히는 WiFi.localIP() 를 보고 노트북 실제 IP로 고칠 것 (robot_config.py 도 같이).
const char*    PC_IP       = "192.168.137.1";
const uint16_t STREAM_PORT = 9999;               // ESP32 -> PC (PC가 서버)
const uint16_t CMD_PORT    = 8890;               // PC -> ESP32 (ESP32가 서버)

// ==========================================================================
// 핀 배치
// ==========================================================================
const int PIN_MPU_INT   =  4;   // (미사용 — MPU6050_light 는 폴링 방식)
const int PIN_DHT       = 13;
const int PIN_SERVO     = 14;
const int PIN_I2C_SDA   = 21;   // LCD + MPU6050 + ToF 공유
const int PIN_I2C_SCL   = 22;   // LCD + MPU6050 + ToF 공유

// 모터 — 이 파일에서 구동하지 않는다. 부팅 시 LOW 로 내려 오동작만 막는다.
const int PIN_R_EN_R    = 16, PIN_R_EN_L = 17, PIN_R_LPWM = 19, PIN_R_RPWM = 32;
const int PIN_L_EN_R    = 25, PIN_L_EN_L = 26, PIN_L_RPWM = 27, PIN_L_LPWM = 33;
// 엔코더 — 이동 제어와 함께 별도 파일에서 다룬다(34/35 는 입력전용 핀).
const int PIN_ENC_R_A   = 18, PIN_ENC_R_B = 23;
const int PIN_ENC_L_A   = 34, PIN_ENC_L_B = 35;

// ★ 가스 센서: 주신 핀맵에 없어서 여기서 읽지 않는다. 스트림의 "gas" 필드를
//   생략하면 파이썬(route_message)이 None 으로 처리하므로 지금도 안전하다.
//   핀이 정해지면 readGasPpm() 만 추가하고 sendStream() 에 필드를 넣으면 된다.

// ==========================================================================
// 전역 상태
// ==========================================================================
Servo             servo;
Adafruit_VL53L0X  lox;
MPU6050           mpu(Wire);
DHT               dht(PIN_DHT, DHT22);

WiFiServer  cmdServer(CMD_PORT);
WiFiClient  cmdClient;      // 노트북이 붙는 명령 소켓
WiFiClient  streamClient;   // 노트북으로 나가는 데이터 소켓

bool  tofOK = false, mpuOK = false;
float lastTempC = NAN;      // DHT22 는 2초에 한 번만 읽을 수 있어 캐시한다

// 동작 모드
enum Mode { MODE_SWEEP, MODE_SCAN, MODE_HOLD };
Mode mode = MODE_SWEEP;

// 연속 스윕 상태
float    sweepOff = 0.0;    // 현재 오프셋각
int      sweepDir = 1;      // +1 우측, -1 좌측
uint32_t nextStepAt = 0;    // 다음 스텝 시각(ms)

// scan 명령 상태 (논블로킹 — 스캔 중에도 명령 소켓이 살아 있어야 함)
int      scanId = 0;
float    scanFrom = 0, scanTo = 0, scanStep = 3.0, scanCur = 0;
uint8_t  scanSamples = 3;

uint32_t lastDhtAt = 0, lastStreamTryAt = 0;

// ==========================================================================
// 유틸
// ==========================================================================
static float clampOff(float off) {
  if (off < SERVO_MIN_OFF) return SERVO_MIN_OFF;
  if (off > SERVO_MAX_OFF) return SERVO_MAX_OFF;
  return off;
}

// 오프셋각 -> 물리 서보각으로 바꿔 실제로 움직인다.
static void servoWriteOff(float off) {
  float phys = SERVO_CENTER + SERVO_SIGN * clampOff(off);
  if (phys < 0)   phys = 0;
  if (phys > 180) phys = 180;
  servo.write((int)(phys + 0.5f));
}

// 삽입정렬 후 중앙값 (n 이 작아서 이걸로 충분)
static uint16_t medianOf(uint16_t* a, uint8_t n) {
  for (uint8_t i = 1; i < n; i++) {
    uint16_t k = a[i];
    int j = i - 1;
    while (j >= 0 && a[j] > k) { a[j + 1] = a[j]; j--; }
    a[j + 1] = k;
  }
  return a[n / 2];
}

// ── 아주 작은 JSON 리더 ───────────────────────────────────────────────
// 들어오는 명령은 {"cmd":"aim","id":3,"angle":-12.5,"n":5} 처럼 평평한 구조뿐이라
// ArduinoJson 을 붙이지 않고 필요한 키만 직접 뽑는다(버전 차이 문제도 없앤다).
static bool jsonStr(const String& s, const char* key, String& out) {
  String pat = String("\"") + key + "\"";
  int k = s.indexOf(pat);
  if (k < 0) return false;
  int c = s.indexOf(':', k + pat.length());
  if (c < 0) return false;
  int q1 = s.indexOf('"', c + 1);
  if (q1 < 0) return false;
  int q2 = s.indexOf('"', q1 + 1);
  if (q2 < 0) return false;
  out = s.substring(q1 + 1, q2);
  return true;
}

static bool jsonNum(const String& s, const char* key, float& out) {
  String pat = String("\"") + key + "\"";
  int k = s.indexOf(pat);
  if (k < 0) return false;
  int c = s.indexOf(':', k + pat.length());
  if (c < 0) return false;
  int i = c + 1;
  while (i < (int)s.length() && (s[i] == ' ' || s[i] == '\t')) i++;
  int st = i;
  if (i < (int)s.length() && (s[i] == '-' || s[i] == '+')) i++;
  while (i < (int)s.length() && (isdigit(s[i]) || s[i] == '.')) i++;
  if (i == st) return false;
  out = s.substring(st, i).toFloat();
  return true;
}

// ── 스트림 송신 ───────────────────────────────────────────────────────
// 노트북이 줄(\n) 단위로 자르므로 반드시 개행으로 끝낼 것.
static void sendLine(const char* line) {
  if (streamClient && streamClient.connected()) {
    streamClient.print(line);
    streamClient.print('\n');
  }
}

// ==========================================================================
// 센서
// ==========================================================================
// ToF 1회 측정. 반환 = 거리(mm), status 는 원본 RangeStatus 를 그대로 넘긴다.
static uint16_t readTof(uint8_t& status) {
  if (!tofOK) { status = 255; return 0; }
  VL53L0X_RangingMeasurementData_t m;
  lox.rangingTest(&m, false);
  status = m.RangeStatus;
  return m.RangeMilliMeter;
}

static bool tofValid(uint16_t mm, uint8_t status) {
  // RangeStatus 4 = out of range. 범위 밖 값을 그대로 쓰면 허공에 가짜 벽이 생긴다.
  return (status != 4 && status != 255 && mm >= TOF_MIN_MM && mm <= TOF_MAX_MM);
}

static float readYaw() {
  if (!mpuOK) return NAN;
  mpu.update();
  return mpu.getAngleZ();   // 부호 보정(GYRO_SIGN)은 파이썬 쪽에서 한다
}

// DHT22 는 초당 0.5회가 한계라 2초 간격으로만 갱신하고 그 사이엔 캐시를 쓴다.
static void updateTemp() {
  if (millis() - lastDhtAt < 2000) return;
  lastDhtAt = millis();
  float t = dht.readTemperature();
  if (!isnan(t)) lastTempC = t;
}

// 스트림 한 줄 전송. 파이썬 tof_commander.route_message() 가 받는 형식.
static void sendStream(float off, uint16_t mm, uint8_t status, bool valid) {
  float yaw = readYaw();
  char buf[224];
  int n = snprintf(buf, sizeof(buf),
      "{\"type\":\"stream\",\"servo\":%.1f,\"dist\":%u,\"st\":%u,\"valid\":%d,\"t\":%lu",
      off, mm, status, valid ? 1 : 0, (unsigned long)millis());
  if (!isnan(yaw))
    n += snprintf(buf + n, sizeof(buf) - n, ",\"yaw\":%.2f", yaw);
  if (!isnan(lastTempC))
    n += snprintf(buf + n, sizeof(buf) - n, ",\"temp\":%.1f", lastTempC);
  snprintf(buf + n, sizeof(buf) - n, "}");
  sendLine(buf);
}

// ==========================================================================
// 명령 처리
// ==========================================================================
// aim: 한 각도를 겨냥해 여러 번 재고 중앙값을 돌려준다.
// 짧고(약 0.3초) 끝나는 동작이라 블로킹으로 처리한다.
static void doAim(int id, float off, uint8_t samples) {
  off = clampOff(off);
  servoWriteOff(off);
  delay(AIM_SETTLE_MS);

  if (samples < 1) samples = 1;
  if (samples > 10) samples = 10;

  uint16_t vals[10];
  uint8_t got = 0, status = 255;
  for (uint8_t i = 0; i < samples; i++) {
    uint8_t st;
    uint16_t mm = readTof(st);
    if (tofValid(mm, st)) { vals[got++] = mm; status = st; }
    delay(10);
  }

  float yaw = readYaw();
  char buf[192];
  if (got == 0) {
    // 유효값이 하나도 없으면 ok:0 — 파이썬이 'no_valid_range' 로 처리한다.
    snprintf(buf, sizeof(buf),
        "{\"type\":\"aim\",\"id\":%d,\"ok\":0,\"angle\":%.1f,\"n\":0}", id, off);
  } else {
    uint16_t med = medianOf(vals, got);
    int n = snprintf(buf, sizeof(buf),
        "{\"type\":\"aim\",\"id\":%d,\"ok\":1,\"angle\":%.1f,\"dist\":%u,\"st\":%u,\"n\":%u",
        id, off, med, status, got);
    if (!isnan(yaw)) n += snprintf(buf + n, sizeof(buf) - n, ",\"yaw\":%.2f", yaw);
    snprintf(buf + n, sizeof(buf) - n, "}");
  }
  sendLine(buf);
}

static void handleCommand(const String& line) {
  String cmd;
  if (!jsonStr(line, "cmd", cmd)) return;

  if (cmd == "sweep") {
    mode = MODE_SWEEP;
    nextStepAt = 0;
    Serial.println("[CMD] sweep -> 연속 스윕 복귀");

  } else if (cmd == "hold") {
    float ang = 0;
    if (jsonNum(line, "angle", ang)) {
      mode = MODE_HOLD;
      sweepOff = clampOff(ang);
      servoWriteOff(sweepOff);
      Serial.printf("[CMD] hold %.1f도\n", sweepOff);
    }

  } else if (cmd == "aim") {
    float id = 0, ang = 0, n = AIM_SAMPLES;
    if (jsonNum(line, "id", id) && jsonNum(line, "angle", ang)) {
      jsonNum(line, "n", n);
      Serial.printf("[CMD] aim id=%d %.1f도\n", (int)id, ang);
      doAim((int)id, ang, (uint8_t)n);
      // aim 이 끝나면 하던 일(연속 스윕)로 돌아간다.
      nextStepAt = 0;
    }

  } else if (cmd == "scan") {
    float id = 0, f = 0, t = 0, st = 3.0, n = 3;
    if (jsonNum(line, "id", id) && jsonNum(line, "from", f) && jsonNum(line, "to", t)) {
      jsonNum(line, "step", st);
      jsonNum(line, "n", n);
      scanId      = (int)id;
      scanFrom    = clampOff(f);
      scanTo      = clampOff(t);
      scanStep    = (st < 0.5) ? 0.5 : st;    // 0 이면 무한루프가 된다
      scanSamples = (n < 1) ? 1 : (n > 10 ? 10 : (uint8_t)n);
      scanCur     = scanFrom;
      mode        = MODE_SCAN;
      nextStepAt  = 0;
      Serial.printf("[CMD] scan id=%d %.1f~%.1f step %.1f\n",
                    scanId, scanFrom, scanTo, scanStep);
    }

  } else if (cmd == "ping") {
    sendLine("{\"type\":\"pong\"}");
  }
}

// 명령 소켓에서 줄 단위로 읽는다.
static void pollCommands() {
  // 노트북이 아직 안 붙었으면 받아준다.
  // ★ esp32 코어 3.x 기준 accept(). 2.x 를 쓰면 available() 로 바꿔야 한다.
  if (!cmdClient || !cmdClient.connected()) {
    WiFiClient nc = cmdServer.accept();
    if (nc) {
      cmdClient = nc;
      Serial.println("[CMD] 노트북 연결됨");
    }
  }
  if (!cmdClient || !cmdClient.connected()) return;

  static String buf;
  while (cmdClient.available()) {
    char c = cmdClient.read();
    if (c == '\n') {
      buf.trim();
      if (buf.length()) handleCommand(buf);
      buf = "";
    } else if (buf.length() < 200) {
      buf += c;
    }
  }
}

// ==========================================================================
// 스윕 (상태머신 — 논블로킹이라 스캔 중에도 명령을 계속 받을 수 있다)
// ==========================================================================
static void stepSweep() {
  if (millis() < nextStepAt) return;

  servoWriteOff(sweepOff);
  nextStepAt = millis() + SWEEP_SETTLE_MS;

  uint8_t st;
  uint16_t mm = readTof(st);
  sendStream(sweepOff, mm, st, tofValid(mm, st));

  // 다음 각도로. 끝에 닿으면 방향을 뒤집어 계속 왕복한다.
  sweepOff += SWEEP_STEP_DEG * sweepDir;
  if (sweepOff >= SWEEP_MAX_OFF) { sweepOff = SWEEP_MAX_OFF; sweepDir = -1; }
  else if (sweepOff <= SWEEP_MIN_OFF) { sweepOff = SWEEP_MIN_OFF; sweepDir = 1; }
}

static void stepScan() {
  if (millis() < nextStepAt) return;

  servoWriteOff(scanCur);
  nextStepAt = millis() + SWEEP_SETTLE_MS;

  uint16_t vals[10];
  uint8_t got = 0;
  for (uint8_t i = 0; i < scanSamples; i++) {
    uint8_t st;
    uint16_t mm = readTof(st);
    if (tofValid(mm, st)) vals[got++] = mm;
  }

  char buf[128];
  if (got > 0) {
    snprintf(buf, sizeof(buf),
        "{\"type\":\"scan\",\"id\":%d,\"ok\":1,\"angle\":%.1f,\"dist\":%u}",
        scanId, scanCur, medianOf(vals, got));
  } else {
    snprintf(buf, sizeof(buf),
        "{\"type\":\"scan\",\"id\":%d,\"ok\":0,\"angle\":%.1f}", scanId, scanCur);
  }
  sendLine(buf);

  // from > to 인 경우도 있어서 진행 방향을 따로 잡는다.
  float dir = (scanTo >= scanFrom) ? 1.0 : -1.0;
  scanCur += scanStep * dir;
  bool done = (dir > 0) ? (scanCur > scanTo + 0.01) : (scanCur < scanTo - 0.01);
  if (done) {
    snprintf(buf, sizeof(buf), "{\"type\":\"scan_end\",\"id\":%d}", scanId);
    sendLine(buf);
    mode = MODE_SWEEP;      // 스캔이 끝나면 지도 작성 모드로 복귀
    Serial.println("[CMD] scan 완료 -> sweep 복귀");
  }
}

// ==========================================================================
// 연결 유지
// ==========================================================================
static void keepStreamConnected() {
  if (streamClient && streamClient.connected()) return;
  if (millis() - lastStreamTryAt < 3000) return;   // 3초마다 재시도
  lastStreamTryAt = millis();
  if (streamClient.connect(PC_IP, STREAM_PORT)) {
    Serial.printf("[TCP] 스트림 연결됨 %s:%u\n", PC_IP, STREAM_PORT);
  } else {
    Serial.printf("[TCP] 스트림 연결 실패 %s:%u\n", PC_IP, STREAM_PORT);
  }
}

// ==========================================================================
// setup / loop
// ==========================================================================
void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println("\n=== rescue_sensor_node (이동 제외) ===");

  // 모터는 이 파일에서 안 쓴다. EN 을 내려 확실히 정지 상태로 둔다.
  const int motorPins[] = {PIN_R_EN_R, PIN_R_EN_L, PIN_R_LPWM, PIN_R_RPWM,
                           PIN_L_EN_R, PIN_L_EN_L, PIN_L_RPWM, PIN_L_LPWM};
  for (int p : motorPins) { pinMode(p, OUTPUT); digitalWrite(p, LOW); }

  Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);

  // 서보
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);
  servo.setPeriodHertz(50);
  servo.attach(PIN_SERVO, 500, 2500);   // 서보 모델에 따라 펄스폭 조정
  servoWriteOff(0);
  delay(300);

  // ToF
  tofOK = lox.begin();
  Serial.println(tofOK ? "[ToF] VL53L0X OK" : "[ToF] ★초기화 실패 — 배선/주소 확인");

  // MPU6050 (calcOffsets 동안 로봇을 움직이지 말 것)
  Serial.println("[MPU] 오프셋 계산중... 흔들지 마세요");
  mpuOK = (mpu.begin() == 0);
  if (mpuOK) { mpu.calcOffsets(); Serial.println("[MPU] OK"); }
  else       { Serial.println("[MPU] ★초기화 실패 — yaw 없이 진행"); }

  dht.begin();

  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  Serial.printf("[WiFi] %s 접속중", WIFI_SSID);
  while (WiFi.status() != WL_CONNECTED) { delay(400); Serial.print("."); }
  // ★ 이 IP 를 보고 robot_config.py 의 ESP32_IP / PC_IP 대역을 맞출 것.
  Serial.printf("\n[WiFi] 연결됨. 내 IP = %s\n", WiFi.localIP().toString().c_str());

  cmdServer.begin();
  Serial.printf("[CMD] %u 포트에서 노트북 명령 대기\n", CMD_PORT);

  sweepOff = SWEEP_MIN_OFF;   // 왼쪽 끝에서 스윕 시작
}

void loop() {
  if (WiFi.status() != WL_CONNECTED) {
    WiFi.begin(WIFI_SSID, WIFI_PASS);
    delay(500);
    return;
  }

  keepStreamConnected();
  pollCommands();
  updateTemp();

  switch (mode) {
    case MODE_SWEEP: stepSweep(); break;
    case MODE_SCAN:  stepScan();  break;
    case MODE_HOLD:  break;       // 명령이 올 때까지 그 각도에서 정지
  }
}

#include <Arduino.h>
#include <WiFi.h>
#include <LittleFS.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>

constexpr int MOTOR_A_IN1 = 32;
constexpr int MOTOR_A_IN2 = 33;
constexpr int MOTOR_B_IN1 = 25;
constexpr int MOTOR_B_IN2 = 26;
constexpr int BUZZER_PIN = 9;
constexpr int SERVO_1_PIN = 22;
constexpr int SERVO_2_PIN = 21;

constexpr int MOTOR_A_IN1_CH = 0;
constexpr int MOTOR_A_IN2_CH = 1;
constexpr int MOTOR_B_IN1_CH = 2;
constexpr int MOTOR_B_IN2_CH = 3;
constexpr int BUZZER_CH = 4;
constexpr int SERVO_1_CH = 6;
constexpr int SERVO_2_CH = 7;
constexpr int PWM_FREQ = 20000;
constexpr int PWM_RESOLUTION_BITS = 8;
constexpr int SERVO_FREQ = 50;
constexpr int SERVO_RESOLUTION_BITS = 16;
constexpr uint8_t MAX_PWM = 255;
constexpr uint32_t CONTROL_TIMEOUT_MS = 300;
constexpr int JOYSTICK_DEADZONE = 8;
constexpr uint8_t TURN_EXPO_PERCENT = 75;
constexpr uint16_t REVERSE_BEEP_FREQ_HZ = 400;
constexpr uint32_t REVERSE_BEEP_ON_MS = 400;
constexpr uint32_t REVERSE_BEEP_OFF_MS = 500;
constexpr uint16_t SERVO_MIN_PULSE_US = 500;
constexpr uint16_t SERVO_MAX_PULSE_US = 2400;

const char* AP_SSID = "Tomovo-ESP32";
const char* AP_PASS = "12345678";

AsyncWebServer server(80);
AsyncWebSocket ws("/ws");

volatile uint32_t lastControlAtMs = 0;
uint32_t lastWsCleanupAtMs = 0;
bool reverseBeepEnabled = false;
bool reverseBeepToneOn = false;
uint32_t reverseBeepPhaseStartedMs = 0;
uint8_t currentServo1Angle = 90;
uint8_t currentServo2Angle = 90;

void setReverseBeep(bool enabled) {
  if (enabled == reverseBeepEnabled) return;

  reverseBeepEnabled = enabled;
  reverseBeepPhaseStartedMs = millis();

  if (enabled) {
    reverseBeepToneOn = true;
    ledcWriteTone(BUZZER_CH, REVERSE_BEEP_FREQ_HZ);
  } else {
    reverseBeepToneOn = false;
    ledcWriteTone(BUZZER_CH, 0);
  }
}

void updateReverseBeep(uint32_t nowMs) {
  if (!reverseBeepEnabled) return;

  const uint32_t phaseDurationMs = reverseBeepToneOn ? REVERSE_BEEP_ON_MS : REVERSE_BEEP_OFF_MS;
  if (nowMs - reverseBeepPhaseStartedMs < phaseDurationMs) return;

  reverseBeepPhaseStartedMs = nowMs;
  reverseBeepToneOn = !reverseBeepToneOn;
  ledcWriteTone(BUZZER_CH, reverseBeepToneOn ? REVERSE_BEEP_FREQ_HZ : 0);
}

int applyExpo(int value, uint8_t expoPercent) {
  const int clipped = constrain(value, -100, 100);
  const long cubic = (static_cast<long>(clipped) * clipped * clipped) / 10000L;
  const long linearWeight = 100 - expoPercent;
  const long shaped = (linearWeight * clipped + static_cast<long>(expoPercent) * cubic) / 100;
  return static_cast<int>(shaped);
}

bool tryParseXY(const uint8_t* data, size_t len, int& xOut, int& yOut) {
  if (len < 3 || len > 11) return false;

  size_t idx = 0;
  int signX = 1;
  int signY = 1;
  int x = 0;
  int y = 0;

  if (data[idx] == '-') {
    signX = -1;
    ++idx;
  }

  bool hasXDigit = false;
  while (idx < len && data[idx] >= '0' && data[idx] <= '9') {
    hasXDigit = true;
    x = x * 10 + (data[idx] - '0');
    ++idx;
  }

  if (!hasXDigit || idx >= len || data[idx] != ',') return false;
  ++idx;

  if (idx < len && data[idx] == '-') {
    signY = -1;
    ++idx;
  }

  bool hasYDigit = false;
  while (idx < len && data[idx] >= '0' && data[idx] <= '9') {
    hasYDigit = true;
    y = y * 10 + (data[idx] - '0');
    ++idx;
  }

  if (!hasYDigit || idx != len) return false;

  xOut = signX * x;
  yOut = signY * y;
  return true;
}

void setServoAngle(int channel, uint8_t& currentAngle, int angle) {
  currentAngle = static_cast<uint8_t>(constrain(angle, 0, 180));
  const uint32_t pulseUs = map(currentAngle, 0, 180, SERVO_MIN_PULSE_US, SERVO_MAX_PULSE_US);
  const uint32_t maxDuty = (1UL << SERVO_RESOLUTION_BITS) - 1;
  const uint32_t duty = (pulseUs * maxDuty) / 20000UL;
  ledcWrite(channel, duty);
}

bool tryParseAngleParam(const String& value, int& angleOut) {
  if (value.isEmpty()) return false;

  for (size_t idx = 0; idx < value.length(); ++idx) {
    if (!isDigit(value[idx])) return false;
  }

  const int angle = value.toInt();
  if (angle < 0 || angle > 180) return false;

  angleOut = angle;
  return true;
}

void writeMotor(int speed, int in1Channel, int in2Channel) {
  const int clipped = constrain(speed, -static_cast<int>(MAX_PWM), static_cast<int>(MAX_PWM));
  const uint8_t pwm = static_cast<uint8_t>(abs(clipped));

  if (clipped > 0) {
    ledcWrite(in1Channel, pwm);
    ledcWrite(in2Channel, 0);
  } else if (clipped < 0) {
    ledcWrite(in1Channel, 0);
    ledcWrite(in2Channel, pwm);
  } else {
    ledcWrite(in1Channel, 0);
    ledcWrite(in2Channel, 0);
  }
}

void stopTank() {
  writeMotor(0, MOTOR_A_IN1_CH, MOTOR_A_IN2_CH);
  writeMotor(0, MOTOR_B_IN1_CH, MOTOR_B_IN2_CH);
  setReverseBeep(false);
}

void applyDriveFromJoystick(int x, int y) {
  int normalizedX = constrain(x, -100, 100);
  int normalizedY = constrain(y, -100, 100);

  if (abs(normalizedX) < JOYSTICK_DEADZONE) normalizedX = 0;
  if (abs(normalizedY) < JOYSTICK_DEADZONE) normalizedY = 0;

  normalizedX = applyExpo(normalizedX, TURN_EXPO_PERCENT);

  int left = normalizedY + normalizedX;
  int right = normalizedY - normalizedX;

  const int maxAbs = max(abs(left), abs(right));
  if (maxAbs > 100) {
    left = (left * 100) / maxAbs;
    right = (right * 100) / maxAbs;
  }

  const int leftPwm = map(left, -100, 100, -MAX_PWM, MAX_PWM);
  const int rightPwm = map(right, -100, 100, -MAX_PWM, MAX_PWM);

  writeMotor(leftPwm, MOTOR_A_IN1_CH, MOTOR_A_IN2_CH);
  writeMotor(rightPwm, MOTOR_B_IN1_CH, MOTOR_B_IN2_CH);
  setReverseBeep(normalizedY < 0);
  lastControlAtMs = millis();
}

void handleRoot(AsyncWebServerRequest* request) {
  request->send(LittleFS, "/index.html", "text/html; charset=utf-8");
}

void handleServo(AsyncWebServerRequest* request) {
  if (!request->hasParam("angle")) {
    request->send(400, "text/plain", "Missing angle parameter");
    return;
  }

  const String angleValue = request->getParam("angle")->value();
  int angle = 0;
  if (!tryParseAngleParam(angleValue, angle)) {
    request->send(400, "text/plain", "Angle must be 0-180");
    return;
  }

  int servoPin = SERVO_1_PIN;
  if (request->hasParam("pin")) {
    const String pinValue = request->getParam("pin")->value();
    if (pinValue != String(SERVO_1_PIN) && pinValue != String(SERVO_2_PIN)) {
      request->send(400, "text/plain", "Pin must be 21 or 22");
      return;
    }

    servoPin = pinValue.toInt();
  }

  if (servoPin == SERVO_2_PIN) {
    setServoAngle(SERVO_2_CH, currentServo2Angle, angle);
  } else {
    setServoAngle(SERVO_1_CH, currentServo1Angle, angle);
  }

  request->send(204);
}

void handleWsEvent(AsyncWebSocket* serverWs, AsyncWebSocketClient* client, AwsEventType type, void* arg, uint8_t* data, size_t len) {
  (void)serverWs;
  (void)client;

  if (type == WS_EVT_DISCONNECT) {
    stopTank();
    return;
  }

  if (type != WS_EVT_DATA) return;

  AwsFrameInfo* info = static_cast<AwsFrameInfo*>(arg);
  if (!(info->final && info->index == 0 && info->len == len && info->opcode == WS_TEXT)) return;

  int x = 0;
  int y = 0;
  if (!tryParseXY(data, len, x, y)) return;

  applyDriveFromJoystick(x, y);
}

void initTankControl() {
  Serial.begin(115200);
  delay(1000); // Počkáme 1s, abychom stihli otevřít Serial Monitor
  Serial.println("\n\n--- START BOOT SEKVEKCE ---");

  Serial.println("1. Nastavuji PWM a piny motoru/bzucaku...");
  ledcSetup(MOTOR_A_IN1_CH, PWM_FREQ, PWM_RESOLUTION_BITS);
  ledcSetup(MOTOR_A_IN2_CH, PWM_FREQ, PWM_RESOLUTION_BITS);
  ledcSetup(MOTOR_B_IN1_CH, PWM_FREQ, PWM_RESOLUTION_BITS);
  ledcSetup(MOTOR_B_IN2_CH, PWM_FREQ, PWM_RESOLUTION_BITS);
  ledcSetup(BUZZER_CH, 2000, 8);
  ledcSetup(SERVO_1_CH, SERVO_FREQ, SERVO_RESOLUTION_BITS);
  ledcSetup(SERVO_2_CH, SERVO_FREQ, SERVO_RESOLUTION_BITS);

  ledcAttachPin(MOTOR_A_IN1, MOTOR_A_IN1_CH);
  ledcAttachPin(MOTOR_A_IN2, MOTOR_A_IN2_CH);
  ledcAttachPin(MOTOR_B_IN1, MOTOR_B_IN1_CH);
  ledcAttachPin(MOTOR_B_IN2, MOTOR_B_IN2_CH);
  ledcAttachPin(BUZZER_PIN, BUZZER_CH);
  ledcAttachPin(SERVO_1_PIN, SERVO_1_CH);
  ledcAttachPin(SERVO_2_PIN, SERVO_2_CH);
  setServoAngle(SERVO_1_CH, currentServo1Angle, currentServo1Angle);
  setServoAngle(SERVO_2_CH, currentServo2Angle, currentServo2Angle);

  Serial.println("2. Zastavuji motory...");
  stopTank();
  lastControlAtMs = millis();
    
  Serial.println("3. Zapinam WiFi v rezimu AP...");
  WiFi.disconnect(true, true);
  delay(100);
  WiFi.mode(WIFI_AP);
  
  // Omezeni vysilaciho vykonu - resi problemy s restarty a vypadky AP u ESP32-C3 kvuli zdrojum
  WiFi.setTxPower(WIFI_POWER_8_5dBm);
  
  Serial.printf("   SSID: %s, Heslo: %s\n", AP_SSID, AP_PASS);
  
  // Zruseni vynuceni kanalu (0) a zadna max omezeni, vyhneme se moznym bugum ESP knihoven
  bool apOk = WiFi.softAP(AP_SSID, AP_PASS); 
  if (apOk) {
    Serial.printf("   AP uspesne vytvoreno. IP adresa: %s\n", WiFi.softAPIP().toString().c_str());
  } else {
    Serial.println("   CHYBA: Selhalo vytvoreni AP!");
  }

  Serial.println("4. Pripojuji souborovy system (LittleFS)...");
  if (!LittleFS.begin(true)) {
    Serial.println("   CHYBA: Nastala chyba pri montovani LittleFS!");
  } else {
    Serial.println("   LittleFS uspesne pripojen.");
  }

  Serial.println("5. Startuji WebServer a WebSocket...");
  ws.onEvent(handleWsEvent);
  server.addHandler(&ws);
  server.on("/", HTTP_GET, handleRoot);
  server.on("/servo", HTTP_GET, handleServo);
  server.begin();
  Serial.println("   WebServer bezi.");
  Serial.println("--- BOOT DOKONCEN ---\n");
}

void setup() {
  // play melody on pin 21 to indicate startup
  /*
  const int melody[] = { 262, 294, 330, 349, 392, 440, 494, 523 };
  ledcSetup(4, 2000, 8);
  ledcAttachPin(10, 4);
  for (int freq : melody) {
    ledcWriteTone(4, freq);
    delay(100);
  }
  ledcWriteTone(4, 0);
  */

  initTankControl();
  Serial.println("\nTank inicializovan (asynchronni ovladani)");
  Serial.print("AP SSID: ");
  Serial.println(AP_SSID);
}

void loop() {
  const uint32_t now = millis();
  if (now - lastWsCleanupAtMs >= 250) {
    ws.cleanupClients();
    lastWsCleanupAtMs = now;
  }

  if (now - lastControlAtMs > CONTROL_TIMEOUT_MS) {
    stopTank();
    lastControlAtMs = now;
  }

  updateReverseBeep(now);
  
  // Debug vypis zive kapacity RAM pro detekci uniku nebo preteceni
  static uint32_t lastMemDebug = 0;
  if (now - lastMemDebug > 5000) {
    lastMemDebug = now;
    Serial.printf("Volna pamet: %d bytu. Pripojeno klientu: %d\n", esp_get_free_heap_size(), WiFi.softAPgetStationNum());
  }

  delay(2);
}

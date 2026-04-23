#include <Arduino.h>
#include <ESPmDNS.h>
#include <Preferences.h>
#include <WiFi.h>
#include <LittleFS.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <Adafruit_NeoPixel.h>

constexpr int MOTOR_A_IN1 = 16;
constexpr int MOTOR_A_IN2 = 17;
constexpr int MOTOR_B_IN1 = 14;
constexpr int MOTOR_B_IN2 = 27;
constexpr int LED_STRIP_PIN = 4;
constexpr uint8_t LED_STRIP_COUNT = 4;
constexpr int SERVO_1_PIN = 22;
constexpr int SERVO_2_PIN = 21;
constexpr int SERVO_3_PIN = 19;
constexpr int SERVO_4_PIN = 18;

constexpr int MOTOR_A_IN1_CH = 0;
constexpr int MOTOR_A_IN2_CH = 1;
constexpr int MOTOR_B_IN1_CH = 2;
constexpr int MOTOR_B_IN2_CH = 3;
constexpr int BUZZER_CH = 4;
constexpr int SERVO_1_CH = 6;
constexpr int SERVO_2_CH = 7;
constexpr int SERVO_3_CH = 8;
constexpr int SERVO_4_CH = 9;
constexpr size_t SERVO_COUNT = 4;
constexpr int PWM_FREQ = 20000;
constexpr int PWM_RESOLUTION_BITS = 8;
constexpr int SERVO_FREQ = 50;
constexpr int SERVO_RESOLUTION_BITS = 16;
constexpr uint8_t MAX_PWM = 255;
constexpr uint8_t DEFAULT_LAUNCH_SENSITIVITY_PERCENT = 100;
constexpr uint8_t MIN_LAUNCH_SENSITIVITY_PERCENT = 25;
constexpr uint8_t MAX_LAUNCH_SENSITIVITY_PERCENT = 150;
constexpr uint8_t MOTOR_LAUNCH_BOOST_PWM = 160;
constexpr uint16_t MIN_LAUNCH_BOOST_MS = 120;
constexpr uint16_t MAX_LAUNCH_BOOST_MS = 360;
constexpr uint32_t CONTROL_TIMEOUT_MS = 300;
constexpr uint8_t DEFAULT_JOYSTICK_DEADZONE = 8;
constexpr uint8_t MIN_JOYSTICK_DEADZONE = 0;
constexpr uint8_t MAX_JOYSTICK_DEADZONE = 20;
constexpr uint8_t DEFAULT_MOTOR_START_COMMAND_PERCENT = 45;
constexpr uint8_t MIN_MOTOR_START_COMMAND_PERCENT = 0;
constexpr uint8_t MAX_MOTOR_START_COMMAND_PERCENT = 80;
constexpr uint8_t MOTOR_MIN_RUNNING_PWM = 95;
constexpr uint8_t TURN_EXPO_PERCENT = 75;
constexpr uint16_t REVERSE_BEEP_FREQ_HZ = 400;
constexpr uint32_t REVERSE_BEEP_ON_MS = 400;
constexpr uint32_t REVERSE_BEEP_OFF_MS = 500;
constexpr uint16_t SERVO_MIN_PULSE_US = 500;
constexpr uint16_t SERVO_MAX_PULSE_US = 2400;
constexpr const char* SETTINGS_NAMESPACE = "pickbot";
constexpr const char* MDNS_HOSTNAME = "pickbot";
constexpr const char* DEFAULT_AP_SSID = "muj-pickbot";
constexpr const char* DEFAULT_AP_PASS = "12345678";
constexpr uint8_t AP_SSID_MAX_LENGTH = 31;
constexpr uint8_t AP_PASS_MIN_LENGTH = 8;
constexpr uint8_t AP_PASS_MAX_LENGTH = 63;
constexpr uint32_t SETTINGS_RESTART_DELAY_MS = 1200;

String apSsid = DEFAULT_AP_SSID;
String apPass = DEFAULT_AP_PASS;

AsyncWebServer server(80);
AsyncWebSocket ws("/ws");
Preferences preferences;
Adafruit_NeoPixel ledStrip(LED_STRIP_COUNT, LED_STRIP_PIN, NEO_GRB + NEO_KHZ800);

const int SERVO_PINS[SERVO_COUNT] = { SERVO_1_PIN, SERVO_2_PIN, SERVO_3_PIN, SERVO_4_PIN };
const int SERVO_CHANNELS[SERVO_COUNT] = { SERVO_1_CH, SERVO_2_CH, SERVO_3_CH, SERVO_4_CH };

struct MotorState {
  int targetPwm = 0;
  uint32_t boostUntilMs = 0;
};

struct ControlSettings {
  uint8_t launchSensitivityPercent = DEFAULT_LAUNCH_SENSITIVITY_PERCENT;
  uint8_t joystickDeadzone = DEFAULT_JOYSTICK_DEADZONE;
  uint8_t motorStartCommandPercent = DEFAULT_MOTOR_START_COMMAND_PERCENT;
  uint8_t ledRed = 0;
  uint8_t ledGreen = 80;
  uint8_t ledBlue = 180;
  bool servoEnabled18 = true;
  bool servoEnabled19 = true;
  bool servoEnabled21 = true;
  bool servoEnabled22 = true;
  bool invertSteeringAxis = false;
  bool invertDriveAxis = false;
  bool invertLeftMotor = false;
  bool invertRightMotor = false;
};

volatile uint32_t lastControlAtMs = 0;
uint32_t lastWsCleanupAtMs = 0;
bool reverseBeepEnabled = false;
bool reverseBeepToneOn = false;
uint32_t reverseBeepPhaseStartedMs = 0;
uint8_t currentServoAngles[SERVO_COUNT] = { 90, 90, 90, 90 };
MotorState leftMotorState;
MotorState rightMotorState;
ControlSettings controlSettings;
bool restartScheduled = false;
uint32_t restartAtMs = 0;

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

int applyDeadzoneAndRescale(int value, int deadzone) {
  const int clipped = constrain(value, -100, 100);
  const int magnitude = abs(clipped);
  if (magnitude <= deadzone) return 0;

  const int scaledMagnitude = map(magnitude, deadzone + 1, 100, 1, 100);
  return clipped > 0 ? scaledMagnitude : -scaledMagnitude;
}

int mapSignedPercentToPwm(int valuePercent) {
  const int clipped = constrain(valuePercent, -100, 100);
  if (clipped == 0) return 0;

  const int magnitude = map(abs(clipped), 1, 100, MOTOR_MIN_RUNNING_PWM, MAX_PWM);
  return clipped > 0 ? magnitude : -magnitude;
}

int applyMotorStartOffset(int valuePercent) {
  const int clipped = constrain(valuePercent, -100, 100);
  if (clipped == 0) return 0;

  const int magnitude = map(abs(clipped), 1, 100, controlSettings.motorStartCommandPercent, 100);
  return clipped > 0 ? magnitude : -magnitude;
}

bool tryParseBoundedInt(const uint8_t* data, size_t len, size_t& idx, int minValue, int maxValue, int& valueOut) {
  if (idx >= len) return false;

  int sign = 1;
  if (data[idx] == '-') {
    sign = -1;
    ++idx;
  }

  bool hasDigit = false;
  int value = 0;
  while (idx < len && data[idx] >= '0' && data[idx] <= '9') {
    hasDigit = true;
    value = value * 10 + (data[idx] - '0');
    ++idx;
  }

  if (!hasDigit) return false;

  value *= sign;
  if (value < minValue || value > maxValue) return false;

  valueOut = value;
  return true;
}

bool tryParseControlMessage(const uint8_t* data, size_t len, int& xOut, int& yOut, int& sensitivityOut) {
  if (len < 3 || len > 15) return false;

  size_t idx = 0;

  int x = 0;
  int y = 0;
  int sensitivity = DEFAULT_LAUNCH_SENSITIVITY_PERCENT;

  if (!tryParseBoundedInt(data, len, idx, -100, 100, x)) return false;
  if (idx >= len || data[idx] != ',') return false;
  ++idx;

  if (!tryParseBoundedInt(data, len, idx, -100, 100, y)) return false;

  if (idx < len) {
    if (data[idx] != ',') return false;
    ++idx;
    if (!tryParseBoundedInt(data, len, idx, MIN_LAUNCH_SENSITIVITY_PERCENT, MAX_LAUNCH_SENSITIVITY_PERCENT, sensitivity)) return false;
  }

  if (idx != len) return false;

  xOut = x;
  yOut = y;
  sensitivityOut = sensitivity;
  return true;
}

bool tryParsePercentParam(const String& value, int minValue, int maxValue, int& parsedOut) {
  if (value.isEmpty()) return false;

  for (size_t idx = 0; idx < value.length(); ++idx) {
    if (!isDigit(value[idx])) return false;
  }

  const int parsed = value.toInt();
  if (parsed < minValue || parsed > maxValue) return false;

  parsedOut = parsed;
  return true;
}

bool tryParseBoolValue(String value, bool& parsedOut) {
  value.trim();
  value.toLowerCase();

  if (value == "1" || value == "true" || value == "on") {
    parsedOut = true;
    return true;
  }

  if (value == "0" || value == "false" || value == "off") {
    parsedOut = false;
    return true;
  }

  return false;
}

bool tryParseApSsid(const String& value, String& parsedOut) {
  if (value.isEmpty() || value.length() > AP_SSID_MAX_LENGTH) return false;
  parsedOut = value;
  return true;
}

bool tryParseApPassword(const String& value, String& parsedOut) {
  if (value.length() < AP_PASS_MIN_LENGTH || value.length() > AP_PASS_MAX_LENGTH) return false;
  parsedOut = value;
  return true;
}

String escapeJsonString(const String& value) {
  String escaped;
  escaped.reserve(value.length() + 8);

  for (size_t idx = 0; idx < value.length(); ++idx) {
    const char ch = value[idx];
    if (ch == '\\' || ch == '"') {
      escaped += '\\';
      escaped += ch;
    } else if (ch == '\n') {
      escaped += "\\n";
    } else if (ch == '\r') {
      escaped += "\\r";
    } else if (ch == '\t') {
      escaped += "\\t";
    } else {
      escaped += ch;
    }
  }

  return escaped;
}

int getServoIndexByPin(int servoPin) {
  for (size_t idx = 0; idx < SERVO_COUNT; ++idx) {
    if (SERVO_PINS[idx] == servoPin) return static_cast<int>(idx);
  }

  return -1;
}

bool isServoEnabledForPin(int servoPin) {
  switch (servoPin) {
    case SERVO_4_PIN: return controlSettings.servoEnabled18;
    case SERVO_3_PIN: return controlSettings.servoEnabled19;
    case SERVO_2_PIN: return controlSettings.servoEnabled21;
    case SERVO_1_PIN: return controlSettings.servoEnabled22;
    default: return false;
  }
}

void sanitizeControlSettings(ControlSettings& settings) {
  settings.launchSensitivityPercent = static_cast<uint8_t>(constrain(settings.launchSensitivityPercent,
                                                                    MIN_LAUNCH_SENSITIVITY_PERCENT,
                                                                    MAX_LAUNCH_SENSITIVITY_PERCENT));
  settings.joystickDeadzone = static_cast<uint8_t>(constrain(settings.joystickDeadzone,
                                                             MIN_JOYSTICK_DEADZONE,
                                                             MAX_JOYSTICK_DEADZONE));
  settings.motorStartCommandPercent = static_cast<uint8_t>(constrain(settings.motorStartCommandPercent,
                                                                     MIN_MOTOR_START_COMMAND_PERCENT,
                                                                     MAX_MOTOR_START_COMMAND_PERCENT));
}

void applyLedColor(uint8_t red, uint8_t green, uint8_t blue) {
  for (uint8_t idx = 0; idx < LED_STRIP_COUNT; ++idx) {
    ledStrip.setPixelColor(idx, ledStrip.Color(red, green, blue));
  }
  ledStrip.show();
}

String getControlSettingsJson(bool restartRequired = false) {
  String json = "{";
  json += "\"launchSensitivity\":";
  json += String(controlSettings.launchSensitivityPercent);
  json += ",\"joystickDeadzone\":";
  json += String(controlSettings.joystickDeadzone);
  json += ",\"motorStartCommandPercent\":";
  json += String(controlSettings.motorStartCommandPercent);
  json += ",\"ledRed\":";
  json += String(controlSettings.ledRed);
  json += ",\"ledGreen\":";
  json += String(controlSettings.ledGreen);
  json += ",\"ledBlue\":";
  json += String(controlSettings.ledBlue);
  json += ",\"servoEnabled18\":";
  json += controlSettings.servoEnabled18 ? "true" : "false";
  json += ",\"servoEnabled19\":";
  json += controlSettings.servoEnabled19 ? "true" : "false";
  json += ",\"servoEnabled21\":";
  json += controlSettings.servoEnabled21 ? "true" : "false";
  json += ",\"servoEnabled22\":";
  json += controlSettings.servoEnabled22 ? "true" : "false";
  json += ",\"apSsid\":\"";
  json += escapeJsonString(apSsid);
  json += "\"";
  json += ",\"apPassword\":\"";
  json += escapeJsonString(apPass);
  json += "\"";
  json += ",\"invertSteeringAxis\":";
  json += controlSettings.invertSteeringAxis ? "true" : "false";
  json += ",\"invertDriveAxis\":";
  json += controlSettings.invertDriveAxis ? "true" : "false";
  json += ",\"invertLeftMotor\":";
  json += controlSettings.invertLeftMotor ? "true" : "false";
  json += ",\"invertRightMotor\":";
  json += controlSettings.invertRightMotor ? "true" : "false";
  json += ",\"restartRequired\":";
  json += restartRequired ? "true" : "false";
  json += "}";
  return json;
}

void loadControlSettings() {
  controlSettings.launchSensitivityPercent = preferences.getUChar("launchPct", DEFAULT_LAUNCH_SENSITIVITY_PERCENT);
  controlSettings.joystickDeadzone = preferences.getUChar("deadzone", DEFAULT_JOYSTICK_DEADZONE);
  controlSettings.motorStartCommandPercent = preferences.getUChar("motorStartPct", DEFAULT_MOTOR_START_COMMAND_PERCENT);
  controlSettings.ledRed = preferences.getUChar("ledR", 0);
  controlSettings.ledGreen = preferences.getUChar("ledG", 80);
  controlSettings.ledBlue = preferences.getUChar("ledB", 180);
  controlSettings.servoEnabled18 = preferences.getBool("servo18", true);
  controlSettings.servoEnabled19 = preferences.getBool("servo19", true);
  controlSettings.servoEnabled21 = preferences.getBool("servo21", true);
  controlSettings.servoEnabled22 = preferences.getBool("servo22", true);
  controlSettings.invertSteeringAxis = preferences.getBool("invSteer", false);
  controlSettings.invertDriveAxis = preferences.getBool("invDrive", false);
  controlSettings.invertLeftMotor = preferences.getBool("invLeft", false);
  controlSettings.invertRightMotor = preferences.getBool("invRight", false);
  apSsid = preferences.getString("apSsid", DEFAULT_AP_SSID);
  apPass = preferences.getString("apPass", DEFAULT_AP_PASS);
  sanitizeControlSettings(controlSettings);
}

void saveControlSettings() {
  sanitizeControlSettings(controlSettings);
  preferences.putUChar("launchPct", controlSettings.launchSensitivityPercent);
  preferences.putUChar("deadzone", controlSettings.joystickDeadzone);
  preferences.putUChar("motorStartPct", controlSettings.motorStartCommandPercent);
  preferences.putUChar("ledR", controlSettings.ledRed);
  preferences.putUChar("ledG", controlSettings.ledGreen);
  preferences.putUChar("ledB", controlSettings.ledBlue);
  preferences.putBool("servo18", controlSettings.servoEnabled18);
  preferences.putBool("servo19", controlSettings.servoEnabled19);
  preferences.putBool("servo21", controlSettings.servoEnabled21);
  preferences.putBool("servo22", controlSettings.servoEnabled22);
  preferences.putBool("invSteer", controlSettings.invertSteeringAxis);
  preferences.putBool("invDrive", controlSettings.invertDriveAxis);
  preferences.putBool("invLeft", controlSettings.invertLeftMotor);
  preferences.putBool("invRight", controlSettings.invertRightMotor);
  preferences.putString("apSsid", apSsid);
  preferences.putString("apPass", apPass);
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

bool tryParseRgbParam(const String& value, uint8_t& parsedOut) {
  int parsed = 0;
  if (!tryParsePercentParam(value, 0, 255, parsed)) return false;
  parsedOut = static_cast<uint8_t>(parsed);
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

bool shouldStartLaunchBoost(int previousTargetPwm, int newTargetPwm) {
  if (newTargetPwm == 0) return false;
  if (abs(newTargetPwm) >= MOTOR_LAUNCH_BOOST_PWM) return false;
  if (previousTargetPwm == 0) return true;
  return (previousTargetPwm > 0 && newTargetPwm < 0) || (previousTargetPwm < 0 && newTargetPwm > 0);
}

uint32_t getLaunchBoostDurationMs() {
  return map(controlSettings.launchSensitivityPercent,
             MIN_LAUNCH_SENSITIVITY_PERCENT,
             MAX_LAUNCH_SENSITIVITY_PERCENT,
             MIN_LAUNCH_BOOST_MS,
             MAX_LAUNCH_BOOST_MS);
}

void updateMotorTarget(MotorState& state, int targetPwm, uint32_t nowMs) {
  const int clippedTarget = constrain(targetPwm, -static_cast<int>(MAX_PWM), static_cast<int>(MAX_PWM));
  if (shouldStartLaunchBoost(state.targetPwm, clippedTarget)) {
    state.boostUntilMs = nowMs + getLaunchBoostDurationMs();
  } else if (clippedTarget == 0) {
    state.boostUntilMs = 0;
  }

  state.targetPwm = clippedTarget;
}

int getEffectiveMotorPwm(const MotorState& state, uint32_t nowMs) {
  if (state.targetPwm == 0) return 0;
  if (nowMs >= state.boostUntilMs) return state.targetPwm;

  const int direction = state.targetPwm > 0 ? 1 : -1;
  return direction * max(abs(state.targetPwm), static_cast<int>(MOTOR_LAUNCH_BOOST_PWM));
}

void updateMotorOutputs(uint32_t nowMs) {
  writeMotor(getEffectiveMotorPwm(leftMotorState, nowMs), MOTOR_A_IN1_CH, MOTOR_A_IN2_CH);
  writeMotor(getEffectiveMotorPwm(rightMotorState, nowMs), MOTOR_B_IN1_CH, MOTOR_B_IN2_CH);
}

void stopTank() {
  leftMotorState.targetPwm = 0;
  leftMotorState.boostUntilMs = 0;
  rightMotorState.targetPwm = 0;
  rightMotorState.boostUntilMs = 0;
  updateMotorOutputs(millis());
  setReverseBeep(false);
}

void applyDriveFromJoystick(int x, int y, int sensitivityPercent) {
  int normalizedX = constrain(x, -100, 100);
  int normalizedY = constrain(y, -100, 100);
  (void)sensitivityPercent;

  if (controlSettings.invertSteeringAxis) normalizedX = -normalizedX;
  if (controlSettings.invertDriveAxis) normalizedY = -normalizedY;

  normalizedX = applyDeadzoneAndRescale(normalizedX, controlSettings.joystickDeadzone);
  normalizedY = applyDeadzoneAndRescale(normalizedY, controlSettings.joystickDeadzone);

  normalizedX = applyExpo(normalizedX, TURN_EXPO_PERCENT);

  int left = normalizedY + normalizedX;
  int right = normalizedY - normalizedX;

  const int maxAbs = max(abs(left), abs(right));
  if (maxAbs > 100) {
    left = (left * 100) / maxAbs;
    right = (right * 100) / maxAbs;
  }

  left = applyMotorStartOffset(left);
  right = applyMotorStartOffset(right);

  const int leftPwm = mapSignedPercentToPwm(left);
  const int rightPwm = mapSignedPercentToPwm(right);
  const int adjustedLeftPwm = controlSettings.invertLeftMotor ? -leftPwm : leftPwm;
  const int adjustedRightPwm = controlSettings.invertRightMotor ? -rightPwm : rightPwm;

  const uint32_t nowMs = millis();

  updateMotorTarget(leftMotorState, adjustedLeftPwm, nowMs);
  updateMotorTarget(rightMotorState, adjustedRightPwm, nowMs);
  updateMotorOutputs(nowMs);
  setReverseBeep(normalizedY < 0);
  lastControlAtMs = nowMs;
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
    if (pinValue != String(SERVO_1_PIN) && pinValue != String(SERVO_2_PIN) &&
        pinValue != String(SERVO_3_PIN) && pinValue != String(SERVO_4_PIN)) {
      request->send(400, "text/plain", "Pin must be 18, 19, 21 or 22");
      return;
    }

    servoPin = pinValue.toInt();
  }

  if (!isServoEnabledForPin(servoPin)) {
    request->send(409, "text/plain", "Servo slider is disabled");
    return;
  }

  const int servoIndex = getServoIndexByPin(servoPin);
  if (servoIndex < 0) {
    request->send(400, "text/plain", "Unknown servo pin");
    return;
  }

  setServoAngle(SERVO_CHANNELS[servoIndex], currentServoAngles[servoIndex], angle);

  request->send(204);
}

void handleSettingsGet(AsyncWebServerRequest* request) {
  request->send(200, "application/json; charset=utf-8", getControlSettingsJson());
}

void handleLed(AsyncWebServerRequest* request) {
  if (!request->hasParam("r") || !request->hasParam("g") || !request->hasParam("b")) {
    request->send(400, "text/plain", "Missing r/g/b parameter");
    return;
  }

  uint8_t red = 0;
  uint8_t green = 0;
  uint8_t blue = 0;
  if (!tryParseRgbParam(request->getParam("r")->value(), red) ||
      !tryParseRgbParam(request->getParam("g")->value(), green) ||
      !tryParseRgbParam(request->getParam("b")->value(), blue)) {
    request->send(400, "text/plain", "r/g/b must be 0-255");
    return;
  }

  controlSettings.ledRed = red;
  controlSettings.ledGreen = green;
  controlSettings.ledBlue = blue;
  applyLedColor(red, green, blue);
  request->send(204);
}

void handleSettingsPost(AsyncWebServerRequest* request) {
  ControlSettings updatedSettings = controlSettings;
  String updatedApSsid = apSsid;
  String updatedApPass = apPass;
  bool restartRequired = false;

  if (request->hasParam("launchSensitivity", true)) {
    int parsedSensitivity = 0;
    if (!tryParsePercentParam(request->getParam("launchSensitivity", true)->value(),
                              MIN_LAUNCH_SENSITIVITY_PERCENT,
                              MAX_LAUNCH_SENSITIVITY_PERCENT,
                              parsedSensitivity)) {
      request->send(400, "text/plain", "launchSensitivity must be 25-150");
      return;
    }
    updatedSettings.launchSensitivityPercent = static_cast<uint8_t>(parsedSensitivity);
  }

  if (request->hasParam("joystickDeadzone", true)) {
    int parsedDeadzone = 0;
    if (!tryParsePercentParam(request->getParam("joystickDeadzone", true)->value(),
                              MIN_JOYSTICK_DEADZONE,
                              MAX_JOYSTICK_DEADZONE,
                              parsedDeadzone)) {
      request->send(400, "text/plain", "joystickDeadzone must be 0-20");
      return;
    }
    updatedSettings.joystickDeadzone = static_cast<uint8_t>(parsedDeadzone);
  }

  if (request->hasParam("motorStartCommandPercent", true)) {
    int parsedStartCommand = 0;
    if (!tryParsePercentParam(request->getParam("motorStartCommandPercent", true)->value(),
                              MIN_MOTOR_START_COMMAND_PERCENT,
                              MAX_MOTOR_START_COMMAND_PERCENT,
                              parsedStartCommand)) {
      request->send(400, "text/plain", "motorStartCommandPercent must be 0-80");
      return;
    }
    updatedSettings.motorStartCommandPercent = static_cast<uint8_t>(parsedStartCommand);
  }

  if (request->hasParam("ledRed", true)) {
    if (!tryParseRgbParam(request->getParam("ledRed", true)->value(), updatedSettings.ledRed)) {
      request->send(400, "text/plain", "ledRed must be 0-255");
      return;
    }
  }

  if (request->hasParam("ledGreen", true)) {
    if (!tryParseRgbParam(request->getParam("ledGreen", true)->value(), updatedSettings.ledGreen)) {
      request->send(400, "text/plain", "ledGreen must be 0-255");
      return;
    }
  }

  if (request->hasParam("ledBlue", true)) {
    if (!tryParseRgbParam(request->getParam("ledBlue", true)->value(), updatedSettings.ledBlue)) {
      request->send(400, "text/plain", "ledBlue must be 0-255");
      return;
    }
  }

  auto parseServoEnabledParam = [&](const char* name, bool& targetValue) -> bool {
    if (!request->hasParam(name, true)) return true;

    bool parsed = false;
    if (!tryParseBoolValue(request->getParam(name, true)->value(), parsed)) {
      request->send(400, "text/plain", String(name) + " must be true/false");
      return false;
    }

    targetValue = parsed;
    return true;
  };

  if (!parseServoEnabledParam("servoEnabled18", updatedSettings.servoEnabled18)) return;
  if (!parseServoEnabledParam("servoEnabled19", updatedSettings.servoEnabled19)) return;
  if (!parseServoEnabledParam("servoEnabled21", updatedSettings.servoEnabled21)) return;
  if (!parseServoEnabledParam("servoEnabled22", updatedSettings.servoEnabled22)) return;

  if (request->hasParam("apSsid", true)) {
    if (!tryParseApSsid(request->getParam("apSsid", true)->value(), updatedApSsid)) {
      request->send(400, "text/plain", "apSsid must be 1-31 characters");
      return;
    }
  }

  if (request->hasParam("apPassword", true)) {
    if (!tryParseApPassword(request->getParam("apPassword", true)->value(), updatedApPass)) {
      request->send(400, "text/plain", "apPassword must be 8-63 characters");
      return;
    }
  }

  auto parseBoolParam = [&](const char* name, bool& targetValue) -> bool {
    if (!request->hasParam(name, true)) return true;

    bool parsed = false;
    if (!tryParseBoolValue(request->getParam(name, true)->value(), parsed)) {
      request->send(400, "text/plain", String(name) + " must be true/false");
      return false;
    }

    targetValue = parsed;
    return true;
  };

  if (!parseBoolParam("invertSteeringAxis", updatedSettings.invertSteeringAxis)) return;
  if (!parseBoolParam("invertDriveAxis", updatedSettings.invertDriveAxis)) return;
  if (!parseBoolParam("invertLeftMotor", updatedSettings.invertLeftMotor)) return;
  if (!parseBoolParam("invertRightMotor", updatedSettings.invertRightMotor)) return;

  sanitizeControlSettings(updatedSettings);
  restartRequired = (updatedApSsid != apSsid) || (updatedApPass != apPass);
  controlSettings = updatedSettings;
  apSsid = updatedApSsid;
  apPass = updatedApPass;
  saveControlSettings();
  applyLedColor(controlSettings.ledRed, controlSettings.ledGreen, controlSettings.ledBlue);
  if (restartRequired) {
    restartScheduled = true;
    restartAtMs = millis() + SETTINGS_RESTART_DELAY_MS;
  }
  request->send(200, "application/json; charset=utf-8", getControlSettingsJson(restartRequired));
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
  int sensitivity = DEFAULT_LAUNCH_SENSITIVITY_PERCENT;
  if (!tryParseControlMessage(data, len, x, y, sensitivity)) return;

  applyDriveFromJoystick(x, y, sensitivity);
}

void initTankControl() {
  Serial.begin(115200);
  delay(1000); // Počkáme 1s, abychom stihli otevřít Serial Monitor
  Serial.println("\n\n--- START BOOT SEKVEKCE ---");

  if (!preferences.begin(SETTINGS_NAMESPACE, false)) {
    Serial.println("0. CHYBA: Nepodarilo se otevrit Preferences.");
  } else {
    loadControlSettings();
  }

  Serial.println("1. Nastavuji PWM a piny motoru/servosystemu...");
  ledcSetup(MOTOR_A_IN1_CH, PWM_FREQ, PWM_RESOLUTION_BITS);
  ledcSetup(MOTOR_A_IN2_CH, PWM_FREQ, PWM_RESOLUTION_BITS);
  ledcSetup(MOTOR_B_IN1_CH, PWM_FREQ, PWM_RESOLUTION_BITS);
  ledcSetup(MOTOR_B_IN2_CH, PWM_FREQ, PWM_RESOLUTION_BITS);
  ledcSetup(SERVO_1_CH, SERVO_FREQ, SERVO_RESOLUTION_BITS);
  ledcSetup(SERVO_2_CH, SERVO_FREQ, SERVO_RESOLUTION_BITS);
  ledcSetup(SERVO_3_CH, SERVO_FREQ, SERVO_RESOLUTION_BITS);
  ledcSetup(SERVO_4_CH, SERVO_FREQ, SERVO_RESOLUTION_BITS);

  ledcAttachPin(MOTOR_A_IN1, MOTOR_A_IN1_CH);
  ledcAttachPin(MOTOR_A_IN2, MOTOR_A_IN2_CH);
  ledcAttachPin(MOTOR_B_IN1, MOTOR_B_IN1_CH);
  ledcAttachPin(MOTOR_B_IN2, MOTOR_B_IN2_CH);
  ledcAttachPin(SERVO_1_PIN, SERVO_1_CH);
  ledcAttachPin(SERVO_2_PIN, SERVO_2_CH);
  ledcAttachPin(SERVO_3_PIN, SERVO_3_CH);
  ledcAttachPin(SERVO_4_PIN, SERVO_4_CH);
  ledStrip.begin();
  applyLedColor(controlSettings.ledRed, controlSettings.ledGreen, controlSettings.ledBlue);
  for (size_t idx = 0; idx < SERVO_COUNT; ++idx) {
    setServoAngle(SERVO_CHANNELS[idx], currentServoAngles[idx], currentServoAngles[idx]);
  }

  Serial.println("2. Zastavuji motory...");
  stopTank();
  lastControlAtMs = millis();
    
  Serial.println("3. Zapinam WiFi v rezimu AP...");
  WiFi.disconnect(true, true);
  delay(100);
  WiFi.mode(WIFI_AP);
  
  // Omezeni vysilaciho vykonu - resi problemy s restarty a vypadky AP u ESP32-C3 kvuli zdrojum
  WiFi.setTxPower(WIFI_POWER_8_5dBm);
  
  Serial.printf("   SSID: %s, Heslo: %s\n", apSsid.c_str(), apPass.c_str());
  
  // Zruseni vynuceni kanalu (0) a zadna max omezeni, vyhneme se moznym bugum ESP knihoven
  bool apOk = WiFi.softAP(apSsid.c_str(), apPass.c_str()); 
  if (apOk) {
    Serial.printf("   AP uspesne vytvoreno. IP adresa: %s\n", WiFi.softAPIP().toString().c_str());
  } else {
    Serial.println("   CHYBA: Selhalo vytvoreni AP!");
  }

  Serial.println("3a. Startuji mDNS...");
  if (MDNS.begin(MDNS_HOSTNAME)) {
    MDNS.addService("http", "tcp", 80);
    Serial.printf("   mDNS aktivni: http://%s.local\n", MDNS_HOSTNAME);
  } else {
    Serial.println("   CHYBA: mDNS se nepodarilo spustit.");
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
  server.on("/led", HTTP_GET, handleLed);
  server.on("/settings", HTTP_GET, handleSettingsGet);
  server.on("/settings", HTTP_POST, handleSettingsPost);
  server.begin();
  Serial.println("   WebServer bezi.");
  Serial.println("--- BOOT DOKONCEN ---\n");
}

void setup() {
  // play melody on pin 21 to indicate startup

  const int melody[] = { 262, 294, 330, 349, 392, 440, 494, 523 };
  ledcSetup(4, 2000, 8);
  ledcAttachPin(22, 4);
  for (int freq : melody) {
    ledcWriteTone(4, freq);
    delay(100);
  }
  ledcWriteTone(4, 0);
  // deatach
  ledcDetachPin(22);

  /*
    // play full starwars melody on pin 22 to indicate startup
    const int starWarsMelody[] = { 440, 440, 440, 349, 523, 440, 349, 523, 440 };
    const int starWarsDurations[] = { 500, 500, 500, 350, 150, 500, 350, 150, 1000 };

    ledcSetup(4, 2000, 8);
    ledcAttachPin(22, 4);
    for (size_t i = 0; i < sizeof(starWarsMelody) / sizeof(starWarsMelody[0]); ++i) {
      ledcWriteTone(4, starWarsMelody[i]);
      delay(starWarsDurations[i]);
    }
    ledcWriteTone(4, 0);
    delay(1000);
  }
  */

  initTankControl();
  Serial.println("\nTank inicializovan (asynchronni ovladani)");
  Serial.print("AP SSID: ");
  Serial.println(apSsid);
}

void loop() {
  const uint32_t now = millis();
  if (restartScheduled && now >= restartAtMs) {
    Serial.println("Restartuji ESP32 kvuli zmene WiFi nastaveni...");
    delay(50);
    ESP.restart();
  }

  if (now - lastWsCleanupAtMs >= 250) {
    ws.cleanupClients();
    lastWsCleanupAtMs = now;
  }

  if (now - lastControlAtMs > CONTROL_TIMEOUT_MS) {
    stopTank();
    lastControlAtMs = now;
  }

  updateReverseBeep(now);
  updateMotorOutputs(now);
  
  // Debug vypis zive kapacity RAM pro detekci uniku nebo preteceni
  static uint32_t lastMemDebug = 0;
  if (now - lastMemDebug > 5000) {
    lastMemDebug = now;
    Serial.printf("Volna pamet: %d bytu. Pripojeno klientu: %d\n", esp_get_free_heap_size(), WiFi.softAPgetStationNum());
  }

  delay(2);
}

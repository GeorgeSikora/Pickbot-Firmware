#include <Arduino.h>
#include <Adafruit_NeoPixel.h>

constexpr int MOTOR_A_IN1 = 16;
constexpr int MOTOR_A_IN2 = 17;
constexpr int MOTOR_B_IN1 = 14;
constexpr int MOTOR_B_IN2 = 27;
constexpr int MOTOR_C_IN1 = 32;
constexpr int MOTOR_C_IN2 = 33;
constexpr int MOTOR_D_IN1 = 25;
constexpr int MOTOR_D_IN2 = 26;

constexpr int SERVO_1_PIN = 22;
constexpr int SERVO_2_PIN = 21;
constexpr int SERVO_3_PIN = 19;
constexpr int SERVO_4_PIN = 18;

constexpr int LED_STRIP_PIN = 4;
constexpr uint16_t LED_STRIP_COUNT = 4;

constexpr int MOTOR_A_IN1_CH = 0;
constexpr int MOTOR_A_IN2_CH = 1;
constexpr int MOTOR_B_IN1_CH = 2;
constexpr int MOTOR_B_IN2_CH = 3;
constexpr int MOTOR_C_IN1_CH = 4;
constexpr int MOTOR_C_IN2_CH = 5;
constexpr int MOTOR_D_IN1_CH = 10;
constexpr int MOTOR_D_IN2_CH = 11;

constexpr int SERVO_1_CH = 6;
constexpr int SERVO_2_CH = 7;
constexpr int SERVO_3_CH = 8;
constexpr int SERVO_4_CH = 9;

constexpr int MOTOR_PWM_FREQ = 20000;
constexpr int MOTOR_PWM_BITS = 8;
constexpr int SERVO_PWM_FREQ = 50;
constexpr int SERVO_PWM_BITS = 16;

constexpr uint16_t SERVO_MIN_US = 500;
constexpr uint16_t SERVO_MAX_US = 2400;

constexpr uint32_t MOTOR_STEP_INTERVAL_MS = 200;
constexpr int MOTOR_STEP = 20;
constexpr int MOTOR_MIN_PWM = 90;
constexpr int MOTOR_MAX_PWM = 255;

constexpr uint32_t SERVO_STEP_INTERVAL_MS = 20;
constexpr int SERVO_STEP_DEG = 2;

constexpr uint32_t RAINBOW_STEP_INTERVAL_MS = 25;

struct MotorOutput {
  int pin;
  int channel;
};

struct ServoOutput {
  int pin;
  int channel;
  int angle;
  int direction;
};

MotorOutput motors[] = {
  {MOTOR_A_IN1, MOTOR_A_IN1_CH},
  {MOTOR_A_IN2, MOTOR_A_IN2_CH},
  {MOTOR_B_IN1, MOTOR_B_IN1_CH},
  {MOTOR_B_IN2, MOTOR_B_IN2_CH},
  {MOTOR_C_IN1, MOTOR_C_IN1_CH},
  {MOTOR_C_IN2, MOTOR_C_IN2_CH},
  {MOTOR_D_IN1, MOTOR_D_IN1_CH},
  {MOTOR_D_IN2, MOTOR_D_IN2_CH},
};

ServoOutput servos[] = {
  {SERVO_1_PIN, SERVO_1_CH, 0, 1},
  {SERVO_2_PIN, SERVO_2_CH, 0, 1},
  {SERVO_3_PIN, SERVO_3_CH, 0, 1},
  {SERVO_4_PIN, SERVO_4_CH, 0, 1},
};

Adafruit_NeoPixel ledStrip(LED_STRIP_COUNT, LED_STRIP_PIN, NEO_GRB + NEO_KHZ800);

int motorPwm = MOTOR_MIN_PWM;
int motorDirection = 1;
uint8_t rainbowHue = 0;

uint32_t lastMotorStepMs = 0;
uint32_t lastServoStepMs = 0;
uint32_t lastRainbowStepMs = 0;

uint32_t servoPulseToDuty(uint16_t pulseUs) {
  const uint32_t periodUs = 1000000UL / SERVO_PWM_FREQ;
  const uint32_t maxDuty = (1UL << SERVO_PWM_BITS) - 1;
  return (static_cast<uint64_t>(pulseUs) * maxDuty) / periodUs;
}

uint32_t angleToServoDuty(int angle) {
  const int constrainedAngle = constrain(angle, 0, 180);
  const uint16_t pulseUs = map(constrainedAngle, 0, 180, SERVO_MIN_US, SERVO_MAX_US);
  return servoPulseToDuty(pulseUs);
}

void updateMotors(uint32_t nowMs) {
  if (nowMs - lastMotorStepMs < MOTOR_STEP_INTERVAL_MS) {
    return;
  }

  lastMotorStepMs = nowMs;
  motorPwm += MOTOR_STEP * motorDirection;

  if (motorPwm >= MOTOR_MAX_PWM) {
    motorPwm = MOTOR_MAX_PWM;
    motorDirection = -1;
  } else if (motorPwm <= MOTOR_MIN_PWM) {
    motorPwm = MOTOR_MIN_PWM;
    motorDirection = 1;
  }

  // Alternate direction every speed step by flipping active side for each pair.
  const bool forward = motorDirection > 0;

  ledcWrite(MOTOR_A_IN1_CH, forward ? motorPwm : 0);
  ledcWrite(MOTOR_A_IN2_CH, forward ? 0 : motorPwm);
  ledcWrite(MOTOR_B_IN1_CH, forward ? motorPwm : 0);
  ledcWrite(MOTOR_B_IN2_CH, forward ? 0 : motorPwm);
  ledcWrite(MOTOR_C_IN1_CH, forward ? motorPwm : 0);
  ledcWrite(MOTOR_C_IN2_CH, forward ? 0 : motorPwm);
  ledcWrite(MOTOR_D_IN1_CH, forward ? motorPwm : 0);
  ledcWrite(MOTOR_D_IN2_CH, forward ? 0 : motorPwm);
}

void updateServos(uint32_t nowMs) {
  if (nowMs - lastServoStepMs < SERVO_STEP_INTERVAL_MS) {
    return;
  }

  lastServoStepMs = nowMs;

  for (auto& servo : servos) {
    if (servo.pin < 0) {
      continue;
    }

    servo.angle += SERVO_STEP_DEG * servo.direction;

    if (servo.angle >= 180) {
      servo.angle = 180;
      servo.direction = -1;
    } else if (servo.angle <= 0) {
      servo.angle = 0;
      servo.direction = 1;
    }

    ledcWrite(servo.channel, angleToServoDuty(servo.angle));
  }
}

void updateRainbow(uint32_t nowMs) {
  if (nowMs - lastRainbowStepMs < RAINBOW_STEP_INTERVAL_MS) {
    return;
  }

  lastRainbowStepMs = nowMs;

  for (uint16_t idx = 0; idx < LED_STRIP_COUNT; ++idx) {
    const uint8_t pixelHue = rainbowHue + (idx * 256 / LED_STRIP_COUNT);
    const uint32_t color = ledStrip.gamma32(ledStrip.ColorHSV(static_cast<uint16_t>(pixelHue) * 256));
    ledStrip.setPixelColor(idx, color);
  }

  ledStrip.show();
  ++rainbowHue;
}

void setup() {
  for (const auto& motor : motors) {
    ledcSetup(motor.channel, MOTOR_PWM_FREQ, MOTOR_PWM_BITS);
    ledcAttachPin(motor.pin, motor.channel);
    ledcWrite(motor.channel, 0);
  }

  for (auto& servo : servos) {
    if (servo.pin < 0) {
      continue;
    }

    ledcSetup(servo.channel, SERVO_PWM_FREQ, SERVO_PWM_BITS);
    ledcAttachPin(servo.pin, servo.channel);
    ledcWrite(servo.channel, angleToServoDuty(servo.angle));
  }

  ledStrip.begin();
  ledStrip.clear();
  ledStrip.show();
}

void loop() {
  const uint32_t nowMs = millis();
  updateMotors(nowMs);
  updateServos(nowMs);
  updateRainbow(nowMs);
}

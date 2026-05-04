#include <Arduino.h>
#include <Basicmicro.h>
#include <HX711_ADC.h>
#include <Wire.h>
#include <Adafruit_LSM6DSOX.h>
#include <Adafruit_Sensor.h>

#if defined(ESP8266) || defined(ESP32) || defined(AVR)
  #include <EEPROM.h>
#endif

// ---------- Logging ----------
#define LOG_SERIAL 1
const uint32_t SERIAL_BAUD     = 57600;
const uint32_t CONTROLLER_BAUD = 38400;
const uint32_t LOG_PERIOD_MS   = 500;
const uint32_t COMMAND_PERIOD_MS = 50;

// ---------- RoboClaw / Basicmicro ----------
#define MOTOR_ADDRESS 128
#define LIBRARY_READ_TIMEOUT 10000
Basicmicro controller(&Serial1, LIBRARY_READ_TIMEOUT);

const int MOTOR1_SIGN = +1;
const int MOTOR2_SIGN = +1;

Adafruit_LSM6DSOX lsm6dsox;

// ---------- Pins ----------

// HX711
const int HX_DOUT = 22;
const int HX_SCK  = 23;
HX711_ADC LoadCell1(HX_DOUT, HX_SCK);

// LEDs
const int ledMotor1Pin = 11;
const int ledMotor2Pin = 8;

// ---------- Calibration ----------
float cal1 = 105.76;

// ---------- Assist behavior ----------
const float LOAD_START_THRESHOLD = 80.0f;
const float LOAD_STOP_THRESHOLD  = 55.0f;
const int   LOAD_FULL = 2000;

const int32_t SPEED_STOP = 0;
const int32_t SPEED_MIN  = 300;
const int32_t SPEED_MAX  = 2600;

const float ALPHA = 0.20f;

const float ANGLE_MIN_DEG = 0.0f;
const float ANGLE_MAX_DEG = 90.0f;

// ---------- State Machine ----------
enum AssistState : uint8_t {
  IDLE = 0,
  ASSIST = 1,
};

AssistState state = IDLE;

// ---------- IMU data ----------
struct ImuData {
  float ax, ay, az;
  float gx, gy, gz;
  float roll, pitch, yaw;
};

// ---------- Runtime variables ----------
float rawLoad = 0.0f;
float loadFilt = 0.0f;

int32_t desiredAssistSpeed = 0;
int32_t motor1Cmd = 0;
int32_t motor2Cmd = 0;

uint32_t lastCommandMs = 0;
uint32_t lastLogMs = 0;

float angleDeg = 0.0f;
ImuData imuData{};

bool initImu() {
  Wire.begin();
  if (!lsm6dsox.begin_I2C()) return false;
  return true;
}

ImuData readImu() {
  sensors_event_t accel, gyro, temp;
  lsm6dsox.getEvent(&accel, &gyro, &temp);

  ImuData d{};
  d.ax = accel.acceleration.x;
  d.ay = accel.acceleration.y;
  d.az = accel.acceleration.z;
  d.gx = gyro.gyro.x;
  d.gy = gyro.gyro.y;
  d.gz = gyro.gyro.z;

  // Basic Tilt Calculation (Standard Physics)
  // Roll = atan2(Y, Z), Pitch = atan2(-X, sqrt(Y^2 + Z^2))
  d.roll  = atan2(d.ay, d.az) * 180.0 / M_PI;
  d.pitch = atan2(-d.ax, sqrt(d.ay * d.ay + d.az * d.az)) * 180.0 / M_PI;
  d.yaw   = -1.0f;
  return d;
}

// ---------- Helpers ----------
void allLedsOff() {
  digitalWrite(ledMotor1Pin, LOW);
  digitalWrite(ledMotor2Pin, LOW);
}

int32_t speedFromLoad(float loadVal) {
  if (abs(loadVal) <= LOAD_START_THRESHOLD) return SPEED_MIN;
  if (abs(loadVal) >= LOAD_FULL) return SPEED_MAX;

  float frac = (abs(loadVal) - LOAD_START_THRESHOLD) / (LOAD_FULL - LOAD_START_THRESHOLD);
  int32_t speed = (int32_t)(SPEED_MIN + frac * (SPEED_MAX - SPEED_MIN));
  return constrain(speed, SPEED_MIN, SPEED_MAX);
}

void stopMotors() {
  motor1Cmd = 0;
  motor2Cmd = 0;

  bool success = controller.SpeedM1M2(
    MOTOR_ADDRESS,
    (uint32_t)motor1Cmd,
    (uint32_t)motor2Cmd
  );

#if LOG_SERIAL
  if (!success) {
    Serial.println("Failed to send STOP command.");
  }
#endif
}

void sendMotorCommand(int32_t desiredSpeed) {
  motor1Cmd = 0;
  motor2Cmd = 0;
  
  if (rawLoad >= 0) {
    if (angleDeg < ANGLE_MAX_DEG) {
      motor1Cmd = MOTOR1_SIGN * desiredSpeed;
      motor2Cmd = MOTOR2_SIGN * desiredSpeed;
    }
  }
  else if (rawLoad < 0) {
    if (angleDeg > ANGLE_MIN_DEG) {
      motor1Cmd = -MOTOR1_SIGN * desiredSpeed;
      motor2Cmd = -MOTOR2_SIGN * desiredSpeed;
    }
  }

  bool success = controller.SpeedM1M2(
    MOTOR_ADDRESS,
    (uint32_t)motor1Cmd,
    (uint32_t)motor2Cmd
  );

#if LOG_SERIAL
  if (!success) {
    Serial.println("SpeedM1M2 command failed.");
  }
#endif
}

// ---------- Main logic ----------
void updateSensors() {
  LoadCell1.update();
  rawLoad = LoadCell1.getData();
  loadFilt = ALPHA * rawLoad + (1.0f - ALPHA) * loadFilt;
  imuData = readImu();
  angleDeg = imuData.pitch;
}

void updateStateMachine() {
  switch (state) {
    case IDLE:
      if (abs(loadFilt) > LOAD_START_THRESHOLD) {
        state = ASSIST;
      }
      break;

    case ASSIST:
      if (abs(loadFilt) < LOAD_STOP_THRESHOLD) {
        state = IDLE;
      }
      break;
  }
}

void applyControl() {
  allLedsOff();

  if (state == ASSIST) {
    desiredAssistSpeed = speedFromLoad(loadFilt);
    digitalWrite(ledMotor1Pin, HIGH);
    digitalWrite(ledMotor2Pin, HIGH);
  } else {
    desiredAssistSpeed = SPEED_STOP;
  }
}

void serviceMotorController() {
  uint32_t now = millis();
  if (now - lastCommandMs < COMMAND_PERIOD_MS) return;
  lastCommandMs = now;

  if (desiredAssistSpeed == SPEED_STOP) {
    stopMotors();
  } else {
    sendMotorCommand(desiredAssistSpeed);
  }
}

void maybeLog() {
#if LOG_SERIAL
  uint32_t now = millis();
  if (now - lastLogMs < LOG_PERIOD_MS) return;
  lastLogMs = now;

  // Serial.print("rawLoad: ");
  // Serial.println(rawLoad, 3);

  Serial.print("loadFilt: ");
  Serial.println(loadFilt, 3);

  Serial.print("state: ");
  Serial.println((int)state);

  Serial.print("desiredAssistSpeed: ");
  Serial.println(desiredAssistSpeed);

  Serial.print("motor1Cmd: ");
  Serial.println(motor1Cmd);

  Serial.print("motor2Cmd: ");
  Serial.println(motor2Cmd);

  Serial.print("roll: ");
  Serial.println(imuData.roll, 3);

  Serial.print("pitch: ");
  Serial.println(imuData.pitch, 3);

  Serial.print("angleDeg: ");
  Serial.println(angleDeg, 3);

  Serial.println("--------------------------------");
#endif
}

// ---------- Setup / Loop ----------
void setup() {
#if LOG_SERIAL
  Serial.begin(SERIAL_BAUD);
  delay(50);
  Serial.println();
  Serial.println("Starting one-load-cell dual-motor assist...");
#endif

  pinMode(ledMotor1Pin, OUTPUT);
  pinMode(ledMotor2Pin, OUTPUT);
  allLedsOff();

  const unsigned long stabilizingtime = 2000;
  const bool doTare = true;

  LoadCell1.begin();
  LoadCell1.setSamplesInUse(4);
  LoadCell1.start(stabilizingtime, doTare);

  if (LoadCell1.getTareTimeoutFlag()) {
#if LOG_SERIAL
    Serial.println("Timeout on LoadCell1. Check wiring.");
#endif
    while (1) {}
  }

  LoadCell1.setCalFactor(cal1);

#if LOG_SERIAL
  Serial.println("HX711 startup complete.");
#endif

  if (!initImu()) {
#if LOG_SERIAL
    Serial.println("IMU init failed (LSM6DSOX). Check wiring/I2C address.");
#endif
    while (1) {}
  }

#if LOG_SERIAL
  Serial.println("IMU init OK");
#endif


  Serial1.begin(CONTROLLER_BAUD);
  controller.begin(CONTROLLER_BAUD);
  delay(100);

  controller.SetM1VelocityPID(MOTOR_ADDRESS, 14.19221, 0.60826, 0.0, 2640);
  controller.SetM2VelocityPID(MOTOR_ADDRESS, 14.19223, 0.62845, 0.0, 2640);

#if LOG_SERIAL
  Serial.println("Controller initialized.");
#endif

  stopMotors();
}

void loop() {
  updateSensors();
  updateStateMachine();
  applyControl();
  serviceMotorController();
  maybeLog();
}
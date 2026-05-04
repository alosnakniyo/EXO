#include <Arduino.h>
#include <Basicmicro.h>
#include <HX711_ADC.h>
#include <Wire.h>
#include <Adafruit_LSM6DSOX.h>
#include <Adafruit_Sensor.h>

#if defined(ESP8266) || defined(ESP32) || defined(AVR)
  #include <EEPROM.h>
#endif

#define LOG_SERIAL 2
const uint32_t SERIAL_BAUD = 57600;
const uint32_t CONTROLLER_BAUD = 38400;
const uint32_t LOG_PERIOD_MS = 500;
const uint32_t COMMAND_PERIOD_MS = 50;

#define MOTOR_ADDRESS 128
#define LIBRARY_READ_TIMEOUT 10000
Basicmicro controller(&Serial2, LIBRARY_READ_TIMEOUT);

const int MOTOR3_SIGN = +1;
const int MOTOR4_SIGN = +1;

Adafruit_LSM6DSOX lsm6dsox;

const int HX_DOUT_C = 24;
const int HX_SCK_C  = 25;
const int HX_DOUT_CC = 26;
const int HX_SCK_CC  = 27;

HX711_ADC LoadCell2(HX_DOUT_C, HX_SCK_C);
HX711_ADC LoadCell3(HX_DOUT_CC, HX_SCK_CC);

const int ledMotor3Pin = 11;
const int ledMotor4Pin = 8;

float calC = 102.77f;
float calCC = 101.08f;

const float LOAD_START_THRESHOLD = 80.0f;
const float LOAD_STOP_THRESHOLD  = 55.0f;
const int LOAD_FULL = 2000;

const int32_t SPEED_STOP = 0;
const int32_t SPEED_MIN  = 300;
const int32_t SPEED_MAX  = 2600;

const float ALPHA = 0.20f;

const float ANGLE_MIN_DEG = 0.0f;
const float ANGLE_MAX_DEG = 90.0f;

enum AssistState : uint8_t {
  IDLE = 0,
  ASSIST_C = 1,
  ASSIST_CC = 2
};

AssistState state = IDLE;

struct ImuData {
  float ax, ay, az;
  float gx, gy, gz;
  float roll, pitch, yaw;
};

float rawLoadC = 0.0f;
float loadFiltC = 0.0f;
float rawLoadCC = 0.0f;
float loadFiltCC = 0.0f;
float activeLoad = 0.0f;

int32_t desiredAssistSpeed = 0;
int32_t motor3Cmd = 0;
int32_t motor4Cmd = 0;

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

  d.roll  = atan2(d.ay, d.az) * 180.0f / M_PI;
  d.pitch = atan2(-d.ax, sqrt(d.ay * d.ay + d.az * d.az)) * 180.0f / M_PI;
  d.yaw   = -1.0f;

  return d;
}

void allLedsOff() {
  digitalWrite(ledMotor3Pin, LOW);
  digitalWrite(ledMotor4Pin, LOW);
}

int32_t speedFromLoad(float loadVal) {
  if (loadVal <= LOAD_START_THRESHOLD) return SPEED_MIN;
  if (loadVal >= LOAD_FULL) return SPEED_MAX;

  float frac = (loadVal - LOAD_START_THRESHOLD) / (float)(LOAD_FULL - LOAD_START_THRESHOLD);
  int32_t speed = (int32_t)(SPEED_MIN + frac * (SPEED_MAX - SPEED_MIN));
  return constrain(speed, SPEED_MIN, SPEED_MAX);
}

void stopMotors() {
  motor3Cmd = 0;
  motor4Cmd = 0;

  bool success = controller.SpeedM1M2(
    MOTOR_ADDRESS,
    (uint32_t)motor3Cmd,
    (uint32_t)motor4Cmd
  );

#if LOG_SERIAL
  if (!success) {
    Serial.println("Failed to send STOP command.");
  }
#endif
}

void sendMotorCommand(int32_t desiredSpeed) {
  motor3Cmd = 0;
  motor4Cmd = 0;

  if (state == ASSIST_C) {
    if (angleDeg < ANGLE_MAX_DEG) {
      motor3Cmd = MOTOR3_SIGN * desiredSpeed;
      motor4Cmd = MOTOR4_SIGN * desiredSpeed;
    }
  } else if (state == ASSIST_CC) {
    if (angleDeg > ANGLE_MIN_DEG) {
      motor3Cmd = -MOTOR3_SIGN * desiredSpeed;
      motor4Cmd = -MOTOR4_SIGN * desiredSpeed;
    }
  }

  bool success = controller.SpeedM1M2(
    MOTOR_ADDRESS,
    (uint32_t)motor3Cmd,
    (uint32_t)motor4Cmd
  );

#if LOG_SERIAL
  if (!success) {
    Serial.println("SpeedM1M2 command failed.");
  }
#endif
}

void updateSensors() {
  LoadCell2.update();
  LoadCell3.update();

  rawLoadC = LoadCell2.getData();
  rawLoadCC = LoadCell3.getData();

  loadFiltC = ALPHA * rawLoadC + (1.0f - ALPHA) * loadFiltC;
  loadFiltCC = ALPHA * rawLoadCC + (1.0f - ALPHA) * loadFiltCC;

  imuData = readImu();
  angleDeg = imuData.pitch;
}

void updateStateMachine() {
  switch (state) {
    case IDLE: {
      bool CStart = (loadFiltC > LOAD_START_THRESHOLD);
      bool CCStart = (loadFiltCC > LOAD_START_THRESHOLD);

      if (CStart || CCStart) {
        if (CStart && (!CCStart || loadFiltC >= loadFiltCC)) {
          state = ASSIST_C;
        } else {
          state = ASSIST_CC;
        }
      }
      break;
    }

    case ASSIST_C:
      if (loadFiltC < LOAD_STOP_THRESHOLD) {
        if (loadFiltCC > LOAD_START_THRESHOLD) state = ASSIST_CC;
        else state = IDLE;
      }
      break;

    case ASSIST_CC:
      if (loadFiltCC < LOAD_STOP_THRESHOLD) {
        if (loadFiltC > LOAD_START_THRESHOLD) state = ASSIST_C;
        else state = IDLE;
      }
      break;
  }
}

void applyControl() {
  allLedsOff();
  activeLoad = 0.0f;

  if (state == ASSIST_C) {
    activeLoad = loadFiltC;

    if (angleDeg < ANGLE_MAX_DEG) {
      desiredAssistSpeed = speedFromLoad(activeLoad);
      digitalWrite(ledMotor3Pin, HIGH);
    } else {
      desiredAssistSpeed = SPEED_STOP;
    }
  } else if (state == ASSIST_CC) {
    activeLoad = loadFiltCC;

    if (angleDeg > ANGLE_MIN_DEG) {
      desiredAssistSpeed = speedFromLoad(activeLoad);
      digitalWrite(ledMotor4Pin, HIGH);
    } else {
      desiredAssistSpeed = SPEED_STOP;
    }
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

  Serial.print("rawLoadC: ");
  Serial.println(rawLoadC, 3);

  Serial.print("rawLoadCC: ");
  Serial.println(rawLoadCC, 3);

  Serial.print("loadFiltC: ");
  Serial.println(loadFiltC, 3);

  Serial.print("loadFiltCC: ");
  Serial.println(loadFiltCC, 3);

  Serial.print("state: ");
  Serial.println((int)state);

  Serial.print("desiredAssistSpeed: ");
  Serial.println(desiredAssistSpeed);

  Serial.print("motor3Cmd: ");
  Serial.println(motor3Cmd);

  Serial.print("motor4Cmd: ");
  Serial.println(motor4Cmd);

  Serial.print("roll: ");
  Serial.println(imuData.roll, 3);

  Serial.print("pitch: ");
  Serial.println(imuData.pitch, 3);

  Serial.print("angleDeg: ");
  Serial.println(angleDeg, 3);

  Serial.println("--------------------------------");
#endif
}

void setup() {
#if LOG_SERIAL
  Serial.begin(SERIAL_BAUD);
  delay(50);
  Serial.println();
  Serial.println("Starting two-load-cell dual-motor assist...");
#endif

  pinMode(ledMotor3Pin, OUTPUT);
  pinMode(ledMotor4Pin, OUTPUT);
  allLedsOff();

  const unsigned long stabilizingtime = 2000;
  const bool doTare = true;

  LoadCell2.begin();
  LoadCell2.setSamplesInUse(4);
  LoadCell2.start(stabilizingtime, doTare);

  if (LoadCell2.getTareTimeoutFlag()) {
#if LOG_SERIAL
    Serial.println("Timeout on LoadCell2. Check wiring.");
#endif
    while (1) {}
  }

  LoadCell2.setCalFactor(calC);

  LoadCell3.begin();
  LoadCell3.setSamplesInUse(4);
  LoadCell3.start(stabilizingtime, doTare);

  if (LoadCell3.getTareTimeoutFlag()) {
#if LOG_SERIAL
    Serial.println("Timeout on LoadCell3. Check wiring.");
#endif
    while (1) {}
  }

  LoadCell3.setCalFactor(calCC);

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

  Serial2.begin(CONTROLLER_BAUD);
  controller.begin(CONTROLLER_BAUD);
  delay(100);

  controller.SetM1VelocityPID(MOTOR_ADDRESS, 14.19221, 0.62384, 0.0, 2970);
  controller.SetM2VelocityPID(MOTOR_ADDRESS, 14.19221, 0.63315, 0.0, 2970);

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
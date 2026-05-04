// make lateral pitch, front roll
// imu wires to the back of the user, green light is up
// add pots later
// make it so that the elbow does not move when shoulder does
// make it possible for front and lat to move at the same time
// 2nd imu pins 20 (SDA) and 21 (SCL)

#include <Arduino.h>
#include <Basicmicro.h>
#include <HX711_ADC.h>
#include <Wire.h>
#include <Adafruit_LSM6DSOX.h>
#include <Adafruit_Sensor.h>
#include <math.h>

#if defined(ESP8266) || defined(ESP32) || defined(AVR)
  #include <EEPROM.h>
#endif

// ---------- Logging / Timing ----------
#define LOG_SERIAL 1

const uint32_t SERIAL_BAUD         = 57600;
const uint32_t CONTROLLER_BAUD_1   = 38400;
const uint32_t CONTROLLER_BAUD_2   = 38400;
const uint32_t LOG_PERIOD_MS       = 500;
const uint32_t COMMAND_PERIOD_MS   = 50;

// ---------- RoboClaw / Basicmicro ----------
#define MOTOR_ADDRESS 128
#define LIBRARY_READ_TIMEOUT 10000

Basicmicro controller_1(&Serial1, LIBRARY_READ_TIMEOUT); // lateral
Basicmicro controller_2(&Serial2, LIBRARY_READ_TIMEOUT); // front

const int MOTOR1_SIGN = +1;
const int MOTOR2_SIGN = +1;
const int MOTOR3_SIGN = +1;
const int MOTOR4_SIGN = +1;

Adafruit_LSM6DSOX lsm6dsox;

// ---------- Pins ----------
// HX711
const int HX_DOUT    = 22;
const int HX_SCK     = 23;
const int HX_DOUT_C  = 24;
const int HX_SCK_C   = 25;
const int HX_DOUT_CC = 26;
const int HX_SCK_CC  = 27;

HX711_ADC LoadCell1(HX_DOUT, HX_SCK);       // lateral
HX711_ADC LoadCell2(HX_DOUT_C, HX_SCK_C);   // front C
HX711_ADC LoadCell3(HX_DOUT_CC, HX_SCK_CC); // front CC

// ---------- Calibration ----------
float cal1  = 105.76f;
float calC  = 102.77f;
float calCC = 101.08f;

// ---------- Assist behavior ----------
const float LOAD_START_THRESHOLD = 80.0f;
const float LOAD_STOP_THRESHOLD  = 55.0f;
const int   LOAD_FULL            = 2000;

const int32_t SPEED_STOP = 0;
const int32_t SPEED_MIN  = 300;
const int32_t SPEED_MAX  = 2600;

const float ALPHA = 0.20f;

const float LAT_ANGLE_MIN_DEG   = 0.0f;
const float LAT_ANGLE_MAX_DEG   = 90.0f;
const float FRONT_ANGLE_MIN_DEG = 0.0f;
const float FRONT_ANGLE_MAX_DEG = 90.0f;

// ---------- State Machines ----------
enum LateralAssistState : uint8_t {
  LAT_IDLE = 0,
  LAT_POS  = 1,
  LAT_NEG  = 2
};

enum FrontAssistState : uint8_t {
  FRONT_IDLE = 0,
  FRONT_C    = 1,
  FRONT_CC   = 2
};

LateralAssistState latState = LAT_IDLE;
FrontAssistState frontState = FRONT_IDLE;

// ---------- IMU data ----------
struct ImuData {
  float ax, ay, az;
  float gx, gy, gz;
  float roll, pitch, yaw;
};

// ---------- Runtime variables ----------
float rawLoad     = 0.0f;
float loadFilt    = 0.0f;

float rawLoadC    = 0.0f;
float loadFiltC   = 0.0f;

float rawLoadCC   = 0.0f;
float loadFiltCC  = 0.0f;

int32_t desiredAssistSpeedLat   = 0;
int32_t desiredAssistSpeedFront = 0;

int32_t motor1Cmd = 0;
int32_t motor2Cmd = 0;
int32_t motor3Cmd = 0;
int32_t motor4Cmd = 0;

uint32_t lastCommandMs = 0;
uint32_t lastLogMs = 0;

float angleDegPitch = 0.0f; // lateral uses pitch
float angleDegRoll  = 0.0f; // front uses roll

ImuData imuData{};

// ---------- IMU ----------
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

  d.roll  = atan2(d.ay, d.az) * 180.0f / PI;
  d.pitch = atan2(-d.ax, sqrt(d.ay * d.ay + d.az * d.az)) * 180.0f / PI;
  d.yaw   = -1.0f; // placeholder
  return d;
}

// ---------- Helpers ----------
int32_t speedFromLoad(float loadVal) {
  float mag = fabs(loadVal);

  if (mag <= LOAD_START_THRESHOLD) return SPEED_MIN;
  if (mag >= LOAD_FULL) return SPEED_MAX;

  float frac = (mag - LOAD_START_THRESHOLD) / (LOAD_FULL - LOAD_START_THRESHOLD);
  int32_t speed = (int32_t)(SPEED_MIN + frac * (SPEED_MAX - SPEED_MIN));
  return constrain(speed, SPEED_MIN, SPEED_MAX);
}

void stopMotorsLat() {
  motor1Cmd = 0;
  motor2Cmd = 0;

  bool success = controller_1.SpeedM1M2(
    MOTOR_ADDRESS,
    motor1Cmd,
    motor2Cmd
  );

#if LOG_SERIAL
  if (!success) {
    Serial.println("Failed to send lateral STOP command.");
  }
#endif
}

void stopMotorsFront() {
  motor3Cmd = 0;
  motor4Cmd = 0;

  bool success = controller_2.SpeedM1M2(
    MOTOR_ADDRESS,
    motor3Cmd,
    motor4Cmd
  );

#if LOG_SERIAL
  if (!success) {
    Serial.println("Failed to send front STOP command.");
  }
#endif
}

void sendMotorCommandLat(int32_t desiredSpeed) {
  motor1Cmd = 0;
  motor2Cmd = 0;

  if (latState == LAT_POS) {
    if (angleDegPitch < LAT_ANGLE_MAX_DEG) {
      motor1Cmd =  MOTOR1_SIGN * desiredSpeed;
      motor2Cmd =  MOTOR2_SIGN * desiredSpeed;
    }
  } else if (latState == LAT_NEG) {
    if (angleDegPitch > LAT_ANGLE_MIN_DEG) {
      motor1Cmd = -MOTOR1_SIGN * desiredSpeed;
      motor2Cmd = -MOTOR2_SIGN * desiredSpeed;
    }
  }

  bool success = controller_1.SpeedM1M2(
    MOTOR_ADDRESS,
    motor1Cmd,
    motor2Cmd
  );

#if LOG_SERIAL
  if (!success) {
    Serial.println("Lateral SpeedM1M2 command failed.");
  }
#endif
}

void sendMotorCommandFront(int32_t desiredSpeed) {
  motor3Cmd = 0;
  motor4Cmd = 0;

  if (frontState == FRONT_C) {
    if (angleDegRoll < FRONT_ANGLE_MAX_DEG) {
      motor3Cmd =  MOTOR3_SIGN * desiredSpeed;
      motor4Cmd =  MOTOR4_SIGN * desiredSpeed;
    }
  } else if (frontState == FRONT_CC) {
    if (angleDegRoll > FRONT_ANGLE_MIN_DEG) {
      motor3Cmd = -MOTOR3_SIGN * desiredSpeed;
      motor4Cmd = -MOTOR4_SIGN * desiredSpeed;
    }
  }

  bool success = controller_2.SpeedM1M2(
    MOTOR_ADDRESS,
    motor3Cmd,
    motor4Cmd
  );

#if LOG_SERIAL
  if (!success) {
    Serial.println("Front SpeedM1M2 command failed.");
  }
#endif
}

// ---------- Main logic ----------
void updateSensors() {
  LoadCell1.update();
  LoadCell2.update();
  LoadCell3.update();

  rawLoad   = LoadCell1.getData();
  rawLoadC  = LoadCell2.getData();
  rawLoadCC = LoadCell3.getData();

  loadFilt   = ALPHA * rawLoad   + (1.0f - ALPHA) * loadFilt;
  loadFiltC  = ALPHA * rawLoadC  + (1.0f - ALPHA) * loadFiltC;
  loadFiltCC = ALPHA * rawLoadCC + (1.0f - ALPHA) * loadFiltCC;

  imuData = readImu();
  angleDegPitch = imuData.pitch; // lateral = pitch
  angleDegRoll  = imuData.roll;  // front   = roll
}

void updateLateralState() {
  bool posStart = (loadFilt >  LOAD_START_THRESHOLD);
  bool negStart = (loadFilt < -LOAD_START_THRESHOLD);

  switch (latState) {
    case LAT_IDLE:
      if (posStart) {
        latState = LAT_POS;
      } else if (negStart) {
        latState = LAT_NEG;
      }
      break;

    case LAT_POS:
      if (negStart) {
        latState = LAT_NEG;
      } else if (loadFilt < LOAD_STOP_THRESHOLD) {
        latState = LAT_IDLE;
      }
      break;

    case LAT_NEG:
      if (posStart) {
        latState = LAT_POS;
      } else if (loadFilt > -LOAD_STOP_THRESHOLD) {
        latState = LAT_IDLE;
      }
      break;
  }
}

void updateFrontState() {
  bool cStart  = (loadFiltC  > LOAD_START_THRESHOLD);
  bool ccStart = (loadFiltCC > LOAD_START_THRESHOLD);

  switch (frontState) {
    case FRONT_IDLE:
      if (cStart || ccStart) {
        if (cStart && (!ccStart || loadFiltC >= loadFiltCC)) {
          frontState = FRONT_C;
        } else {
          frontState = FRONT_CC;
        }
      }
      break;

    case FRONT_C:
      if (loadFiltC < LOAD_STOP_THRESHOLD) {
        if (ccStart) frontState = FRONT_CC;
        else frontState = FRONT_IDLE;
      }
      break;

    case FRONT_CC:
      if (loadFiltCC < LOAD_STOP_THRESHOLD) {
        if (cStart) frontState = FRONT_C;
        else frontState = FRONT_IDLE;
      }
      break;
  }
}

void updateStateMachine() {
  updateLateralState();
  updateFrontState();
}

void applyControl() {
  desiredAssistSpeedLat   = SPEED_STOP;
  desiredAssistSpeedFront = SPEED_STOP;

  // ----- Lateral -----
  if (latState == LAT_POS) {
    if (angleDegPitch < LAT_ANGLE_MAX_DEG) {
      desiredAssistSpeedLat = speedFromLoad(loadFilt);
    }
  } else if (latState == LAT_NEG) {
    if (angleDegPitch > LAT_ANGLE_MIN_DEG) {
      desiredAssistSpeedLat = speedFromLoad(loadFilt);
    }
  }

  // ----- Front -----
  if (frontState == FRONT_C) {
    if (angleDegRoll < FRONT_ANGLE_MAX_DEG) {
      desiredAssistSpeedFront = speedFromLoad(loadFiltC);
    }
  } else if (frontState == FRONT_CC) {
    if (angleDegRoll > FRONT_ANGLE_MIN_DEG) {
      desiredAssistSpeedFront = speedFromLoad(loadFiltCC);
    }
  }
}

void serviceMotorController() {
  uint32_t now = millis();
  if (now - lastCommandMs < COMMAND_PERIOD_MS) return;
  lastCommandMs = now;

  // Service both axes independently so they can move simultaneously
  if (desiredAssistSpeedLat == SPEED_STOP) {
    stopMotorsLat();
  } else {
    sendMotorCommandLat(desiredAssistSpeedLat);
  }

  if (desiredAssistSpeedFront == SPEED_STOP) {
    stopMotorsFront();
  } else {
    sendMotorCommandFront(desiredAssistSpeedFront);
  }
}

void maybeLog() {
#if LOG_SERIAL
  uint32_t now = millis();
  if (now - lastLogMs < LOG_PERIOD_MS) return;
  lastLogMs = now;

  Serial.print("loadFilt: ");
  Serial.println(loadFilt, 3);

  Serial.print("loadFiltC: ");
  Serial.println(loadFiltC, 3);

  Serial.print("loadFiltCC: ");
  Serial.println(loadFiltCC, 3);

  Serial.print("latState: ");
  Serial.println((int)latState);

  Serial.print("frontState: ");
  Serial.println((int)frontState);

  Serial.print("desiredAssistSpeedLat: ");
  Serial.println(desiredAssistSpeedLat);

  Serial.print("desiredAssistSpeedFront: ");
  Serial.println(desiredAssistSpeedFront);

  Serial.print("motor1Cmd: ");
  Serial.println(motor1Cmd);

  Serial.print("motor2Cmd: ");
  Serial.println(motor2Cmd);

  Serial.print("motor3Cmd: ");
  Serial.println(motor3Cmd);

  Serial.print("motor4Cmd: ");
  Serial.println(motor4Cmd);

  Serial.print("pitch (for lateral): ");
  Serial.println(imuData.pitch, 3);

  // Serial.print("angleDegRoll: ");
  // Serial.println(angleDegRoll, 3);

  Serial.print("roll (for frontal): ");
  Serial.println(imuData.roll, 3);

  // Serial.print("angleDegPitch: ");
  // Serial.println(angleDegPitch, 3);

  Serial.println("--------------------------------");
#endif
}

// ---------- Setup / Loop ----------
void setup() {
#if LOG_SERIAL
  Serial.begin(SERIAL_BAUD);
  delay(50);
  Serial.println();
  Serial.println("Starting three-load-cell assist...");
#endif

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

  Serial1.begin(CONTROLLER_BAUD_1);
  Serial2.begin(CONTROLLER_BAUD_2);

  controller_1.begin(CONTROLLER_BAUD_1);
  controller_2.begin(CONTROLLER_BAUD_2);

  delay(100);

  // Set PID for both motors on both controllers
  controller_1.SetM1VelocityPID(MOTOR_ADDRESS, 14.19221, 0.60826, 0.0, 2640);
  controller_1.SetM2VelocityPID(MOTOR_ADDRESS, 14.19221, 0.60826, 0.0, 2640);

  controller_2.SetM1VelocityPID(MOTOR_ADDRESS, 14.19223, 0.62845, 0.0, 2640);
  controller_2.SetM2VelocityPID(MOTOR_ADDRESS, 14.19223, 0.62845, 0.0, 2640);

#if LOG_SERIAL
  Serial.println("Controllers initialized.");
#endif

  stopMotorsLat();
  stopMotorsFront();
}

void loop() {
  updateSensors();
  updateStateMachine();
  applyControl();
  serviceMotorController();
  maybeLog();
}
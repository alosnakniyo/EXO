// up: whole thing needs to move clockwise (looking at it head-on), so motor need to move counterclockwise
// down: whole thing needs to move counterclockwise (looking at it head-on), so motor need to move clockwise
// for lateral: negative load is up, positive load is down
// max lat raised is 20.323 (raw: 2212)

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

// ============================================================
// Logging / Timing
// ============================================================
#define LOG_SERIAL 1

const uint32_t SERIAL_BAUD       = 57600;
const uint32_t LATERAL_BAUD      = 38400;
const uint32_t FRONT_BAUD        = 38400;
const uint32_t LOG_PERIOD_MS     = 500;
const uint32_t COMMAND_PERIOD_MS = 50;

// ============================================================
// RoboClaw / Basicmicro (Shoulder)
// ============================================================
#define MOTOR_ADDRESS 128
#define LIBRARY_READ_TIMEOUT 10000

Basicmicro lateralController(&Serial1, LIBRARY_READ_TIMEOUT);
Basicmicro frontController(&Serial2, LIBRARY_READ_TIMEOUT);

const int LATERAL_MOTOR_1_SIGN = -1;
const int LATERAL_MOTOR_2_SIGN = -1;
const int FRONT_MOTOR_1_SIGN   = -1;
const int FRONT_MOTOR_2_SIGN   = -1;

// ============================================================
// IMUs (Shoulder)
// ============================================================
Adafruit_LSM6DSOX frontImu;
Adafruit_LSM6DSOX lateralImu;

// ============================================================
// Shoulder Pins
// ============================================================
const int lateralPotPin = A0;
const int frontPotPin   = A2;

const int HX_DOUT_LATERAL    = 22;
const int HX_SCK_LATERAL     = 23;
const int HX_DOUT_FRONT_DOWN = 32;
const int HX_SCK_FRONT_DOWN  = 33;
const int HX_DOUT_FRONT_UP   = 48;
const int HX_SCK_FRONT_UP    = 49;

HX711_ADC lateralLoadCell(HX_DOUT_LATERAL, HX_SCK_LATERAL);
HX711_ADC frontDownLoadCell(HX_DOUT_FRONT_DOWN, HX_SCK_FRONT_DOWN);
HX711_ADC frontUpLoadCell(HX_DOUT_FRONT_UP, HX_SCK_FRONT_UP);

// ============================================================
// Elbow Pins
// ============================================================
const int HX_DOUT_ELBOW_EXTEND = 9;
const int HX_SCK_ELBOW_EXTEND  = 10;
const int HX_DOUT_ELBOW_FLEX   = 6;
const int HX_SCK_ELBOW_FLEX    = 7;

HX711_ADC elbowExtendLoadCell(HX_DOUT_ELBOW_EXTEND, HX_SCK_ELBOW_EXTEND);
HX711_ADC elbowFlexLoadCell(HX_DOUT_ELBOW_FLEX, HX_SCK_ELBOW_FLEX);

const int elbowPwmPin = 5;
const int elbowDirPin = 4;
const int elbowEnaPin = 3;
const bool ELBOW_ENA_ACTIVE_LOW = true;

const int elbowLedFlexPin   = 8;
const int elbowLedExtendPin = 11;
const int elbowPotPin       = A1;
const uint8_t pgPin         = 2;

// ============================================================
// FSR Cuff Pins
// ============================================================
const int forearmFsrUpPin    = A3;
const int forearmFsrDownPin  = A4;
const int forearmFsrLeftPin  = A5;
const int forearmFsrRightPin = A6;

const int bicepFsrUpPin      = A10;
const int bicepFsrDownPin    = A11;
const int bicepFsrLeftPin    = A12;
const int bicepFsrRightPin   = A13;

// ============================================================
// Calibration
// ============================================================
// Shoulder
float lateralLoadCellCal   = 105.76f;
float frontDownLoadCellCal = 102.77f;
float frontUpLoadCellCal   = 101.08f;

// Elbow
float elbowExtendLoadCellCal = -105.53f;
float elbowFlexLoadCellCal   = -105.98f;

// ============================================================
// Shoulder Assist Behavior
// ============================================================
const float SHOULDER_LOAD_START_THRESHOLD = 80.0f;
const float SHOULDER_LOAD_STOP_THRESHOLD  = 55.0f;
const int   SHOULDER_LOAD_FULL            = 3000;

const int32_t SHOULDER_SPEED_STOP = 0;
const int32_t SHOULDER_SPEED_MIN  = 300;
const int32_t SHOULDER_SPEED_MAX  = 2640;

const float SHOULDER_ALPHA = 0.20f;

// ============================================================
// Elbow Assist Behavior
// ============================================================
const float ELBOW_LOAD_START_THRESHOLD = 80.0f;
const float ELBOW_LOAD_STOP_THRESHOLD  = 80.0f;

const int ELBOW_PWM_STOP = 0;
const int ELBOW_PWM_MIN  = 60;
const int ELBOW_PWM_MAX  = 255;
const int ELBOW_LOAD_FULL = 9000;

const float ELBOW_ALPHA = 0.20f; 

// 0 deg = fully flexed, 180 deg = fully extended
const float ELBOW_EXTEND_MAX_DEG = 135.0f;
const float ELBOW_FLEX_MIN_DEG   = 35.0f;

// ============================================================
// FSR Safety Behavior
// ============================================================
// const float FSR_ALPHA = 0.20f;
const float FSR_STOP_THRESHOLD = 2000.0f; // PUT BACK TO 200

// ============================================================
// State Machines
// ============================================================
enum LateralAssistState : uint8_t {
  LATERAL_IDLE = 0,
  LATERAL_DOWN = 1,
  LATERAL_UP   = 2
};

enum FrontAssistState : uint8_t {
  FRONT_IDLE = 0,
  FRONT_DOWN = 1,
  FRONT_UP   = 2
};

enum ElbowAssistState : uint8_t {
  ELBOW_IDLE   = 0,
  ELBOW_EXTEND = 1,
  ELBOW_FLEX   = 2
};

LateralAssistState lateralState = LATERAL_IDLE;
FrontAssistState frontState = FRONT_IDLE;
ElbowAssistState elbowState = ELBOW_IDLE;

// ============================================================
// IMU Data
// ============================================================
struct ImuData {
  float ax, ay, az;
  float gx, gy, gz;
  float roll, pitch, yaw;
};

// ============================================================
// Shoulder Runtime Variables
// ============================================================
float rawLoadLateral        = 0.0f;
float filteredLoadLateral   = 0.0f;

float rawLoadFrontDown      = 0.0f;
float filteredLoadFrontDown = 0.0f;

float rawLoadFrontUp        = 0.0f;
float filteredLoadFrontUp   = 0.0f;

int32_t desiredLateralAssistSpeed = 0;
int32_t desiredFrontAssistSpeed   = 0;

int32_t lateralMotor1Cmd = 0;
int32_t lateralMotor2Cmd = 0;
int32_t frontMotor1Cmd   = 0;
int32_t frontMotor2Cmd   = 0;

float lateralAngleDeg = 0.0f;
float frontAngleDeg   = 0.0f;

int lateralPotRaw     = 0;
float lateralPotAngle = 0.0f;

int frontPotRaw       = 0;
float frontPotAngle   = 0.0f;

ImuData frontImuData{};
ImuData lateralImuData{};

// ============================================================
// Elbow Runtime Variables
// ============================================================
float rawLoadElbowExtend      = 0.0f;
float rawLoadElbowFlex        = 0.0f;
float filteredLoadElbowExtend = 0.0f;
float filteredLoadElbowFlex   = 0.0f;

int elbowPotRaw        = 0;
float elbowAngleDeg    = 0.0f;

bool desiredElbowRun      = false;
bool desiredElbowDirHigh  = false;
int  desiredElbowPwm      = ELBOW_PWM_STOP;
bool elbowExtendLedOn     = false;
bool elbowFlexLedOn       = false;
bool elbowSuppressedByShoulder = false;

// ============================================================
// FSR Runtime Variables
// ============================================================
float forearmFsrUpRaw    = 0.0f;
float forearmFsrDownRaw  = 0.0f;
float forearmFsrLeftRaw  = 0.0f;
float forearmFsrRightRaw = 0.0f;

float bicepFsrUpRaw      = 0.0f;
float bicepFsrDownRaw    = 0.0f;
float bicepFsrLeftRaw    = 0.0f;
float bicepFsrRightRaw   = 0.0f;

// float forearmFsrUpFilt    = 0.0f;
// float forearmFsrDownFilt  = 0.0f;
// float forearmFsrLeftFilt  = 0.0f;
// float forearmFsrRightFilt = 0.0f;

// float bicepFsrUpFilt      = 0.0f;
// float bicepFsrDownFilt    = 0.0f;
// float bicepFsrLeftFilt    = 0.0f;
// float bicepFsrRightFilt   = 0.0f;

bool fsrSafetyTripped = false;

// ============================================================
// Shared Runtime Variables
// ============================================================
uint32_t lastCommandMs = 0;
uint32_t lastLogMs     = 0;

volatile uint32_t pgPulseCount = 0;
uint32_t lastPgPulseCount = 0;
uint32_t lastPgMs = 0;
float pgHz = 0.0f;

// ============================================================
// ISR
// ============================================================
void pgIsr() {
  pgPulseCount++;
}

// ============================================================
// IMU Helpers
// ============================================================
bool initImus() {
  Wire.begin();

  if (!frontImu.begin_I2C(0x6A, &Wire)) {
#if LOG_SERIAL
    Serial.println("Front IMU not found");
#endif
    return false;
  }

  if (!lateralImu.begin_I2C(0x6B, &Wire)) {
#if LOG_SERIAL
    Serial.println("Lateral IMU not found");
#endif
    return false;
  }

#if LOG_SERIAL
  Serial.println("Both IMUs found");
#endif

  return true;
}

ImuData buildImuData(const sensors_event_t& accel, const sensors_event_t& gyro) {
  ImuData d{};

  d.ax = accel.acceleration.x;
  d.ay = accel.acceleration.y;
  d.az = accel.acceleration.z;

  d.gx = gyro.gyro.x;
  d.gy = gyro.gyro.y;
  d.gz = gyro.gyro.z;

  d.roll  = atan2(d.ay, d.az) * 180.0f / PI;
  d.pitch = atan2(-d.ax, sqrt(d.ay * d.ay + d.az * d.az)) * 180.0f / PI;
  d.yaw   = -1.0f;

  return d;
}

void readImus(ImuData &frontData, ImuData &lateralData) {
  sensors_event_t frontAccel, frontGyro, frontTemp;
  sensors_event_t lateralAccel, lateralGyro, lateralTemp;

  frontImu.getEvent(&frontAccel, &frontGyro, &frontTemp);
  lateralImu.getEvent(&lateralAccel, &lateralGyro, &lateralTemp);

  frontData   = buildImuData(frontAccel, frontGyro);
  lateralData = buildImuData(lateralAccel, lateralGyro);
}

// ============================================================
// Elbow Helpers
// ============================================================
void setElbowEnable(bool enabled) {
  if (ELBOW_ENA_ACTIVE_LOW) digitalWrite(elbowEnaPin, enabled ? LOW : HIGH);
  else                      digitalWrite(elbowEnaPin, enabled ? HIGH : LOW);
}

void setElbowMotor(int pwm, bool dirHigh) {
  digitalWrite(elbowDirPin, dirHigh ? HIGH : LOW);
  analogWrite(elbowPwmPin, constrain(pwm, 0, 255));
}

void allElbowLedsOff() {
  digitalWrite(elbowLedExtendPin, LOW);
  digitalWrite(elbowLedFlexPin, LOW);
}

int pwmFromElbowLoad(float loadVal) {
  float magnitude = fabs(loadVal);

  if (magnitude <= ELBOW_LOAD_START_THRESHOLD) return ELBOW_PWM_MIN;
  if (magnitude >= ELBOW_LOAD_FULL) return ELBOW_PWM_MAX;

  float fraction = (magnitude - ELBOW_LOAD_START_THRESHOLD) /
                   (ELBOW_LOAD_FULL - ELBOW_LOAD_START_THRESHOLD);

  int pwm = (int)(ELBOW_PWM_MIN + fraction * (ELBOW_PWM_MAX - ELBOW_PWM_MIN));
  return constrain(pwm, ELBOW_PWM_MIN, ELBOW_PWM_MAX);
}

// ============================================================
// Shoulder Helpers
// ============================================================
int32_t speedFromShoulderLoad(float loadVal) {
  float magnitude = fabs(loadVal);

  if (magnitude <= SHOULDER_LOAD_START_THRESHOLD) return SHOULDER_SPEED_MIN;
  if (magnitude >= SHOULDER_LOAD_FULL) return SHOULDER_SPEED_MAX;

  float fraction = (magnitude - SHOULDER_LOAD_START_THRESHOLD) /
                   (SHOULDER_LOAD_FULL - SHOULDER_LOAD_START_THRESHOLD);

  int32_t speed = (int32_t)(SHOULDER_SPEED_MIN + fraction * (SHOULDER_SPEED_MAX - SHOULDER_SPEED_MIN));
  return constrain(speed, SHOULDER_SPEED_MIN, SHOULDER_SPEED_MAX);
}

bool lateralDownAllowed() {
  // return (lateralPotAngle > 125.0f) && (lateralAngleDeg < 90.0f);
  return (lateralPotAngle < 50.0f) && (lateralAngleDeg < 90.0f);
}

bool lateralUpAllowed() {
  // return (lateralPotAngle < 205.0f) && (lateralAngleDeg > 0.0f);
  return (lateralPotAngle > 0.0f) && (lateralAngleDeg > 0.0f);
}

bool frontDownAllowed() {
  // return (frontPotAngle > 125.0f) && (frontAngleDeg < 90.0f);
  return (frontPotAngle > 125.0f) && (frontAngleDeg < 90.0f);
}

bool frontUpAllowed() {
  // return (frontPotAngle < 205.0f) && (frontAngleDeg > 0.0f);
  return (frontPotAngle < 205.0f) && (frontAngleDeg > 0.0f);
}

void stopLateralMotors() {
  lateralMotor1Cmd = 0;
  lateralMotor2Cmd = 0;

  bool success = lateralController.SpeedM1M2(
    MOTOR_ADDRESS,
    lateralMotor1Cmd,
    lateralMotor2Cmd
  );

#if LOG_SERIAL
  if (!success) {
    Serial.println("Failed to send lateral STOP command.");
  }
#endif
}

void stopFrontMotors() {
  frontMotor1Cmd = 0;
  frontMotor2Cmd = 0;

  bool success = frontController.SpeedM1M2(
    MOTOR_ADDRESS,
    frontMotor1Cmd,
    frontMotor2Cmd
  );

#if LOG_SERIAL
  if (!success) {
    Serial.println("Failed to send front STOP command.");
  }
#endif
}

void sendLateralMotorCommand(int32_t desiredSpeed) {
  lateralMotor1Cmd = 0;
  lateralMotor2Cmd = 0;

  if (lateralState == LATERAL_DOWN) {
    lateralMotor1Cmd =  -LATERAL_MOTOR_1_SIGN * desiredSpeed;
    lateralMotor2Cmd =  -LATERAL_MOTOR_2_SIGN * desiredSpeed;
  } else if (lateralState == LATERAL_UP) {
    lateralMotor1Cmd = LATERAL_MOTOR_1_SIGN * desiredSpeed;
    lateralMotor2Cmd = LATERAL_MOTOR_2_SIGN * desiredSpeed;
  }

  bool success = lateralController.SpeedM1M2(
    MOTOR_ADDRESS,
    lateralMotor1Cmd,
    lateralMotor2Cmd
  );

#if LOG_SERIAL
  if (!success) {
    Serial.println("Lateral SpeedM1M2 command failed.");
  }
#endif
}

void sendFrontMotorCommand(int32_t desiredSpeed) {
  frontMotor1Cmd = 0;
  frontMotor2Cmd = 0;

  if (frontState == FRONT_DOWN) {
    frontMotor1Cmd =  -FRONT_MOTOR_1_SIGN * desiredSpeed;
    frontMotor2Cmd =  -FRONT_MOTOR_2_SIGN * desiredSpeed;
  } else if (frontState == FRONT_UP) {
    frontMotor1Cmd = FRONT_MOTOR_1_SIGN * desiredSpeed;
    frontMotor2Cmd = FRONT_MOTOR_2_SIGN * desiredSpeed;
  }

  bool success = frontController.SpeedM1M2(
    MOTOR_ADDRESS,
    frontMotor1Cmd,
    frontMotor2Cmd
  );

#if LOG_SERIAL
  if (!success) {
    Serial.println("Front SpeedM1M2 command failed.");
  }
#endif
}

// ============================================================
// FSR Helpers
// ============================================================
// float updateFsrFilter(float previousValue, float rawValue) {
//   return FSR_ALPHA * rawValue + (1.0f - FSR_ALPHA) * previousValue;
// }

bool anyFsrOverThreshold() {
  return
    // (forearmFsrUpFilt    >= FSR_STOP_THRESHOLD) ||
    // (forearmFsrDownFilt  >= FSR_STOP_THRESHOLD) ||
    // (forearmFsrLeftFilt  >= FSR_STOP_THRESHOLD) ||
    // (forearmFsrRightFilt >= FSR_STOP_THRESHOLD) ||
    // (bicepFsrUpFilt      >= FSR_STOP_THRESHOLD) ||
    // (bicepFsrDownFilt    >= FSR_STOP_THRESHOLD) ||
    // (bicepFsrLeftFilt    >= FSR_STOP_THRESHOLD) ||
    // (bicepFsrRightFilt   >= FSR_STOP_THRESHOLD);
    (forearmFsrUpRaw    >= FSR_STOP_THRESHOLD) ||
    (forearmFsrDownRaw  >= FSR_STOP_THRESHOLD) ||
    (forearmFsrLeftRaw  >= FSR_STOP_THRESHOLD) ||
    (forearmFsrRightRaw >= FSR_STOP_THRESHOLD) ||
    (bicepFsrUpRaw      >= FSR_STOP_THRESHOLD) ||
    (bicepFsrDownRaw    >= FSR_STOP_THRESHOLD) ||
    (bicepFsrLeftRaw    >= FSR_STOP_THRESHOLD) ||
    (bicepFsrRightRaw   >= FSR_STOP_THRESHOLD);
}

// ============================================================
// Sensor Update
// ============================================================
void updateSensors() {
  // ----- Shoulder load cells -----
  lateralLoadCell.update();
  frontDownLoadCell.update();
  frontUpLoadCell.update();

  rawLoadLateral   = lateralLoadCell.getData();
  rawLoadFrontDown = frontDownLoadCell.getData();
  rawLoadFrontUp   = frontUpLoadCell.getData();

  filteredLoadLateral   = SHOULDER_ALPHA * rawLoadLateral   + (1.0f - SHOULDER_ALPHA) * filteredLoadLateral;
  filteredLoadFrontDown = SHOULDER_ALPHA * rawLoadFrontDown + (1.0f - SHOULDER_ALPHA) * filteredLoadFrontDown;
  filteredLoadFrontUp   = SHOULDER_ALPHA * rawLoadFrontUp   + (1.0f - SHOULDER_ALPHA) * filteredLoadFrontUp;

  // ----- Shoulder IMUs -----
  readImus(frontImuData, lateralImuData);
  frontAngleDeg   = frontImuData.roll;
  lateralAngleDeg = lateralImuData.roll;

  // ----- Shoulder pots -----
  lateralPotRaw = analogRead(lateralPotPin);
  frontPotRaw   = analogRead(frontPotPin);

  lateralPotAngle = 270.0f * (float)lateralPotRaw / 1023.0f;
  frontPotAngle   = 270.0f * (float)frontPotRaw / 1023.0f;

  // ----- Elbow load cells -----
  elbowExtendLoadCell.update();
  elbowFlexLoadCell.update();

  rawLoadElbowExtend = elbowExtendLoadCell.getData();
  rawLoadElbowFlex   = elbowFlexLoadCell.getData();

  filteredLoadElbowExtend = ELBOW_ALPHA * rawLoadElbowExtend + (1.0f - ELBOW_ALPHA) * filteredLoadElbowExtend;
  filteredLoadElbowFlex   = ELBOW_ALPHA * rawLoadElbowFlex   + (1.0f - ELBOW_ALPHA) * filteredLoadElbowFlex;

  // ----- Elbow pot -----
  elbowPotRaw = analogRead(elbowPotPin);
  // elbowAngleDeg = 270.0f * (float)elbowPotRaw / 1023.0f;
  elbowAngleDeg = 75.0f;

  // ----- FSR cuffs -----
  forearmFsrUpRaw    = analogRead(forearmFsrUpPin);
  forearmFsrDownRaw  = analogRead(forearmFsrDownPin);
  forearmFsrLeftRaw  = analogRead(forearmFsrLeftPin);
  forearmFsrRightRaw = analogRead(forearmFsrRightPin);

  bicepFsrUpRaw      = analogRead(bicepFsrUpPin);
  bicepFsrDownRaw    = analogRead(bicepFsrDownPin);
  bicepFsrLeftRaw    = analogRead(bicepFsrLeftPin);
  bicepFsrRightRaw   = analogRead(bicepFsrRightPin);

  // forearmFsrUpFilt    = updateFsrFilter(forearmFsrUpFilt,    forearmFsrUpRaw);
  // forearmFsrDownFilt  = updateFsrFilter(forearmFsrDownFilt,  forearmFsrDownRaw);
  // forearmFsrLeftFilt  = updateFsrFilter(forearmFsrLeftFilt,  forearmFsrLeftRaw);
  // forearmFsrRightFilt = updateFsrFilter(forearmFsrRightFilt, forearmFsrRightRaw);

  // bicepFsrUpFilt      = updateFsrFilter(bicepFsrUpFilt,      bicepFsrUpRaw);
  // bicepFsrDownFilt    = updateFsrFilter(bicepFsrDownFilt,    bicepFsrDownRaw);
  // bicepFsrLeftFilt    = updateFsrFilter(bicepFsrLeftFilt,    bicepFsrLeftRaw);
  // bicepFsrRightFilt   = updateFsrFilter(bicepFsrRightFilt,   bicepFsrRightRaw);

  fsrSafetyTripped = anyFsrOverThreshold();
}

// ============================================================
// State Updates
// ============================================================
void updateLateralState() {
  bool lateralDownStart = (filteredLoadLateral >  SHOULDER_LOAD_START_THRESHOLD);
  bool lateralUpStart   = (filteredLoadLateral < -SHOULDER_LOAD_START_THRESHOLD);

  switch (lateralState) {
    case LATERAL_IDLE:
      if (lateralDownStart) {
        lateralState = LATERAL_DOWN;
      } else if (lateralUpStart) {
        lateralState = LATERAL_UP;
      }
      break;

    case LATERAL_DOWN:
      if (lateralUpStart) {
        lateralState = LATERAL_UP;
      } else if (filteredLoadLateral < SHOULDER_LOAD_STOP_THRESHOLD) {
        lateralState = LATERAL_IDLE;
      }
      break;

    case LATERAL_UP:
      if (lateralDownStart) {
        lateralState = LATERAL_DOWN;
      } else if (filteredLoadLateral > -SHOULDER_LOAD_STOP_THRESHOLD) {
        lateralState = LATERAL_IDLE;
      }
      break;
  }
}

void updateFrontState() {
  bool frontDownStart = (filteredLoadFrontDown > SHOULDER_LOAD_START_THRESHOLD);
  bool frontUpStart   = (filteredLoadFrontUp   > SHOULDER_LOAD_START_THRESHOLD);

  switch (frontState) {
    case FRONT_IDLE:
      if (frontDownStart || frontUpStart) {
        if (frontDownStart && (!frontUpStart || filteredLoadFrontDown >= filteredLoadFrontUp)) {
          frontState = FRONT_DOWN;
        } else {
          frontState = FRONT_UP;
        }
      }
      break;

    case FRONT_DOWN:
      if (filteredLoadFrontDown < SHOULDER_LOAD_STOP_THRESHOLD) {
        if (frontUpStart) frontState = FRONT_UP;
        else frontState = FRONT_IDLE;
      }
      break;

    case FRONT_UP:
      if (filteredLoadFrontUp < SHOULDER_LOAD_STOP_THRESHOLD) {
        if (frontDownStart) frontState = FRONT_DOWN;
        else frontState = FRONT_IDLE;
      }
      break;
  }
}

void updateElbowState() {
  bool elbowExtendStart = (filteredLoadElbowExtend > ELBOW_LOAD_START_THRESHOLD);
  bool elbowFlexStart   = (filteredLoadElbowFlex   > ELBOW_LOAD_START_THRESHOLD);

  switch (elbowState) {
    case ELBOW_IDLE:
      if (elbowExtendStart || elbowFlexStart) {
        if (elbowExtendStart && (!elbowFlexStart || filteredLoadElbowExtend >= filteredLoadElbowFlex)) {
          elbowState = ELBOW_EXTEND;
        } else {
          elbowState = ELBOW_FLEX;
        }
      }
      break;

    case ELBOW_EXTEND:
      if (filteredLoadElbowExtend < ELBOW_LOAD_STOP_THRESHOLD) {
        elbowState = ELBOW_IDLE;
      }
      break;

    case ELBOW_FLEX:
      if (filteredLoadElbowFlex < ELBOW_LOAD_STOP_THRESHOLD) {
        elbowState = ELBOW_IDLE;
      }
      break;
  }
}

void updateStateMachine() {
  updateLateralState();
  updateFrontState();
  updateElbowState();
}

// ============================================================
// Control
// ============================================================
void applyControl() {
  // ----- Defaults -----
  desiredLateralAssistSpeed = SHOULDER_SPEED_STOP;
  desiredFrontAssistSpeed   = SHOULDER_SPEED_STOP;

  desiredElbowRun      = false;
  desiredElbowDirHigh  = false;
  desiredElbowPwm      = ELBOW_PWM_STOP;
  elbowExtendLedOn     = false;
  elbowFlexLedOn       = false;
  elbowSuppressedByShoulder = false;

  // ----- Global FSR safety -----
  if (fsrSafetyTripped) {
    return;
  }

  // ----- Shoulder: Lateral -----
  if (lateralState == LATERAL_DOWN) {
    if (lateralDownAllowed()) {
      desiredLateralAssistSpeed = speedFromShoulderLoad(filteredLoadLateral);
    }
  } else if (lateralState == LATERAL_UP) {
    if (lateralUpAllowed()) {
      desiredLateralAssistSpeed = speedFromShoulderLoad(filteredLoadLateral);
    }
  }

  // ----- Shoulder: Front -----
  if (frontState == FRONT_DOWN) {
    if (frontDownAllowed()) {
      desiredFrontAssistSpeed = speedFromShoulderLoad(filteredLoadFrontDown);
    }
  } else if (frontState == FRONT_UP) {
    if (frontUpAllowed()) {
      desiredFrontAssistSpeed = speedFromShoulderLoad(filteredLoadFrontUp);
    }
  }

  // ----- Elbow is still while shoulder is active -----
  bool shoulderMotionActive =
    (desiredLateralAssistSpeed != SHOULDER_SPEED_STOP) ||
    (desiredFrontAssistSpeed   != SHOULDER_SPEED_STOP);

  if (shoulderMotionActive) {
    elbowSuppressedByShoulder = true;
    return;
  }

  // ----- Elbow -----
  if (elbowState == ELBOW_EXTEND) {
    if (elbowAngleDeg < ELBOW_EXTEND_MAX_DEG) {
      desiredElbowRun     = true;
      desiredElbowDirHigh = true; // extension
      desiredElbowPwm     = pwmFromElbowLoad(filteredLoadElbowExtend);
      elbowExtendLedOn    = true;
    }
  } else if (elbowState == ELBOW_FLEX) {
    if (elbowAngleDeg > ELBOW_FLEX_MIN_DEG) {
      desiredElbowRun     = true;
      desiredElbowDirHigh = false; // flexion
      desiredElbowPwm     = pwmFromElbowLoad(filteredLoadElbowFlex);
      elbowFlexLedOn      = true;
    }
  }
}

// ============================================================
// Actuator Service
// ============================================================
void serviceActuators() {
  uint32_t now = millis();
  if (now - lastCommandMs < COMMAND_PERIOD_MS) return;
  lastCommandMs = now;

  // ----- Shoulder -----
  if (desiredLateralAssistSpeed == SHOULDER_SPEED_STOP) {
    stopLateralMotors();
  } else {
    sendLateralMotorCommand(desiredLateralAssistSpeed);
  }

  if (desiredFrontAssistSpeed == SHOULDER_SPEED_STOP) {
    stopFrontMotors();
  } else {
    sendFrontMotorCommand(desiredFrontAssistSpeed);
  }

  // ----- Elbow LEDs -----
  digitalWrite(elbowLedExtendPin, elbowExtendLedOn ? HIGH : LOW);
  digitalWrite(elbowLedFlexPin,   elbowFlexLedOn   ? HIGH : LOW);

  // ----- Elbow motor -----
  if (desiredElbowRun) {
    setElbowEnable(true);
    setElbowMotor(desiredElbowPwm, desiredElbowDirHigh);
  } else {
    setElbowMotor(ELBOW_PWM_STOP, false);
    setElbowEnable(false);
  }
}

// ============================================================
// Logging
// ============================================================
void maybeLog() {
#if LOG_SERIAL
  uint32_t now = millis();
  if (now - lastLogMs < LOG_PERIOD_MS) return;
  lastLogMs = now;

  uint32_t dtMs = now - lastPgMs;

  uint32_t pulses;
  noInterrupts();
  pulses = pgPulseCount;
  interrupts();

  uint32_t dp = pulses - lastPgPulseCount;
  float dtSec = (dtMs > 0) ? (dtMs / 1000.0f) : 0.0f;
  pgHz = (dtSec > 0.0f) ? (dp / dtSec) : 0.0f;

  lastPgMs = now;
  lastPgPulseCount = pulses;

  Serial.print("filteredLoadLateral: ");
  Serial.println(-filteredLoadLateral, 3);

  Serial.print("filteredLoadFrontDown: ");
  Serial.println(filteredLoadFrontDown, 3);

  Serial.print("filteredLoadFrontUp: ");
  Serial.println(filteredLoadFrontUp, 3);

  Serial.print("lateralState: ");
  Serial.println((int)lateralState);

  Serial.print("frontState: ");
  Serial.println((int)frontState);

  Serial.print("desiredLateralAssistSpeed: ");
  Serial.println(desiredLateralAssistSpeed);

  Serial.print("desiredFrontAssistSpeed: ");
  Serial.println(desiredFrontAssistSpeed);

  Serial.print("lateralMotor1Cmd: ");
  Serial.println(lateralMotor1Cmd);

  Serial.print("lateralMotor2Cmd: ");
  Serial.println(lateralMotor2Cmd);

  Serial.print("frontMotor1Cmd: ");
  Serial.println(frontMotor1Cmd);

  Serial.print("frontMotor2Cmd: ");
  Serial.println(frontMotor2Cmd);

  Serial.print("lateralAngleDeg: ");
  Serial.println(lateralAngleDeg, 3);

  Serial.print("lateralPotRaw: ");
  Serial.println(lateralPotRaw, 3);

  Serial.print("lateralPotAngle: ");
  Serial.println(lateralPotAngle, 3);

  Serial.print("frontAngleDeg: ");
  Serial.println(frontAngleDeg, 3);

  Serial.print("frontPotAngle: ");
  Serial.println(frontPotAngle, 3);

  Serial.print("filteredLoadElbowExtend: ");
  Serial.println(filteredLoadElbowExtend, 3);

  Serial.print("filteredLoadElbowFlex: ");
  Serial.println(filteredLoadElbowFlex, 3);

  Serial.print("elbowState: ");
  Serial.println((int)elbowState);

  Serial.print("elbowAngleDeg: ");
  Serial.println(elbowAngleDeg, 3);

  Serial.print("desiredElbowRun: ");
  Serial.println((int)desiredElbowRun);

  Serial.print("desiredElbowDirHigh: ");
  Serial.println((int)desiredElbowDirHigh);

  Serial.print("desiredElbowPwm: ");
  Serial.println(desiredElbowPwm);

  Serial.print("elbowSuppressedByShoulder: ");
  Serial.println((int)elbowSuppressedByShoulder);

  Serial.print("fsrSafetyTripped: ");
  Serial.println((int)fsrSafetyTripped);

  // Serial.print("forearmFsrUpRaw: ");
  // Serial.println(forearmFsrUpRaw, 1);
  // Serial.print("forearmFsrDownRaw: ");
  // Serial.println(forearmFsrDownRaw, 1);
  // Serial.print("forearmFsrLeftRaw: ");
  // Serial.println(forearmFsrLeftRaw, 1);
  // Serial.print("forearmFsrRightRaw: ");
  // Serial.println(forearmFsrRightRaw, 1);

  // Serial.print("bicepFsrUpRaw: ");
  // Serial.println(bicepFsrUpRaw, 1);
  // Serial.print("bicepFsrDownRaw: ");
  // Serial.println(bicepFsrDownRaw, 1);
  // Serial.print("bicepFsrLeftRaw: ");
  // Serial.println(bicepFsrLeftRaw, 1);
  // Serial.print("bicepFsrRightRaw: ");
  // Serial.println(bicepFsrRightRaw, 1);

  // Serial.print("forearmFsrUpFilt: ");
  // Serial.println(forearmFsrUpFilt, 1);
  // Serial.print("forearmFsrDownFilt: ");
  // Serial.println(forearmFsrDownFilt, 1);
  // Serial.print("forearmFsrLeftFilt: ");
  // Serial.println(forearmFsrLeftFilt, 1);
  // Serial.print("forearmFsrRightFilt: ");
  // Serial.println(forearmFsrRightFilt, 1);

  // Serial.print("bicepFsrUpFilt: ");
  // Serial.println(bicepFsrUpFilt, 1);
  // Serial.print("bicepFsrDownFilt: ");
  // Serial.println(bicepFsrDownFilt, 1);
  // Serial.print("bicepFsrLeftFilt: ");
  // Serial.println(bicepFsrLeftFilt, 1);
  // Serial.print("bicepFsrRightFilt: ");
  // Serial.println(bicepFsrRightFilt, 1);

  Serial.print("pgHz: ");
  Serial.println(pgHz, 3);

  Serial.println("--------------------------------");
#endif
}

// ============================================================
// Setup / Loop
// ============================================================
void setup() {
#if LOG_SERIAL
  Serial.begin(SERIAL_BAUD);
  delay(50);
  Serial.println();
  Serial.println("Starting combined shoulder + elbow assist with FSR cuff safety...");
#endif

  // ----- Shoulder analog inputs -----
  pinMode(lateralPotPin, INPUT);
  pinMode(frontPotPin, INPUT);

  // ----- Elbow IO -----
  pinMode(elbowLedFlexPin, OUTPUT);
  pinMode(elbowLedExtendPin, OUTPUT);

  pinMode(elbowPwmPin, OUTPUT);
  pinMode(elbowDirPin, OUTPUT);
  pinMode(elbowEnaPin, OUTPUT);

  pinMode(elbowPotPin, INPUT);

  // ----- FSR cuff inputs -----
  pinMode(forearmFsrUpPin, INPUT);
  pinMode(forearmFsrDownPin, INPUT);
  pinMode(forearmFsrLeftPin, INPUT);
  pinMode(forearmFsrRightPin, INPUT);

  pinMode(bicepFsrUpPin, INPUT);
  pinMode(bicepFsrDownPin, INPUT);
  pinMode(bicepFsrLeftPin, INPUT);
  pinMode(bicepFsrRightPin, INPUT);

  // ----- PG interrupt -----
  pinMode(pgPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(pgPin), pgIsr, RISING);
  lastPgMs = millis();
  lastPgPulseCount = 0;

  allElbowLedsOff();
  setElbowEnable(false);
  setElbowMotor(ELBOW_PWM_STOP, false);

  const unsigned long stabilizingTime = 2000;
  const bool doTare = true;

  // ----- Shoulder load cells -----
  lateralLoadCell.begin();
  lateralLoadCell.setSamplesInUse(4);
  lateralLoadCell.start(stabilizingTime, doTare);
  if (lateralLoadCell.getTareTimeoutFlag()) {
#if LOG_SERIAL
    Serial.println("Timeout on lateral load cell. Check wiring.");
#endif
    while (1) {}
  }
  lateralLoadCell.setCalFactor(lateralLoadCellCal);

  frontDownLoadCell.begin();
  frontDownLoadCell.setSamplesInUse(4);
  frontDownLoadCell.start(stabilizingTime, doTare);
  if (frontDownLoadCell.getTareTimeoutFlag()) {
#if LOG_SERIAL
    Serial.println("Timeout on front-down load cell. Check wiring.");
#endif
    while (1) {}
  }
  frontDownLoadCell.setCalFactor(frontDownLoadCellCal);

  frontUpLoadCell.begin();
  frontUpLoadCell.setSamplesInUse(4);
  frontUpLoadCell.start(stabilizingTime, doTare);
  if (frontUpLoadCell.getTareTimeoutFlag()) {
#if LOG_SERIAL
    Serial.println("Timeout on front-up load cell. Check wiring.");
#endif
    while (1) {}
  }
  frontUpLoadCell.setCalFactor(frontUpLoadCellCal);

  // ----- Elbow load cells -----
  elbowExtendLoadCell.begin();
  elbowExtendLoadCell.setSamplesInUse(4);
  elbowExtendLoadCell.start(stabilizingTime, doTare);
  if (elbowExtendLoadCell.getTareTimeoutFlag()) {
#if LOG_SERIAL
    Serial.println("Timeout on elbow-extend load cell. Check wiring.");
#endif
    while (1) {}
  }
  elbowExtendLoadCell.setCalFactor(elbowExtendLoadCellCal);

  elbowFlexLoadCell.begin();
  elbowFlexLoadCell.setSamplesInUse(4);
  elbowFlexLoadCell.start(stabilizingTime, doTare);
  if (elbowFlexLoadCell.getTareTimeoutFlag()) {
#if LOG_SERIAL
    Serial.println("Timeout on elbow-flex load cell. Check wiring.");
#endif
    while (1) {}
  }
  elbowFlexLoadCell.setCalFactor(elbowFlexLoadCellCal);

#if LOG_SERIAL
  Serial.println("HX711 startup complete.");
#endif

  // ----- IMUs -----
  if (!initImus()) {
#if LOG_SERIAL
    Serial.println("IMU init failed. Check wiring / I2C addresses.");
#endif
    while (1) {}
  }

#if LOG_SERIAL
  Serial.println("IMU init OK");
#endif

  // ----- Shoulder motor controllers -----
  Serial1.begin(LATERAL_BAUD);
  Serial2.begin(FRONT_BAUD);

  lateralController.begin(LATERAL_BAUD);
  frontController.begin(FRONT_BAUD);

  delay(100);

  lateralController.SetM1VelocityPID(MOTOR_ADDRESS, 14.19221, 0.60396, 0.0, 2970);
  lateralController.SetM2VelocityPID(MOTOR_ADDRESS, 12.57797, 0.50146, 0.0, 2970);

  frontController.SetM1VelocityPID(MOTOR_ADDRESS, 14.19220, 0.56960, 0.0, 2970);
  frontController.SetM2VelocityPID(MOTOR_ADDRESS, 14.19220, 0.57732, 0.0, 2640);

#if LOG_SERIAL
  Serial.println("Controllers initialized.");
#endif

  stopLateralMotors();
  stopFrontMotors();
  setElbowMotor(ELBOW_PWM_STOP, false);
  setElbowEnable(false);
}

void loop() {
  updateSensors();
  updateStateMachine();
  applyControl();
  serviceActuators();
  maybeLog();
}
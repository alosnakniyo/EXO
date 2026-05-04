// ---------- HX711 ----------
#include <HX711_ADC.h>
#if defined(ESP8266) || defined(ESP32) || defined(AVR)
  #include <EEPROM.h>
#endif

#define ENABLE_IMU 1

#if ENABLE_IMU
  #include <Wire.h>
  #include <Adafruit_LSM6DSOX.h>
  #include <Adafruit_Sensor.h>
  Adafruit_LSM6DSOX lsm6dsox;
#endif

// ---------- Logging ----------
#define LOG_SERIAL 1
const uint32_t SERIAL_BAUD   = 57600;
const uint32_t LOG_PERIOD_MS = 2000UL; // log every 2 seconds

// ---------- Pins ----------
const int HX_BLUE_DOUT = 9;
const int HX_BLUE_SCK  = 10;
HX711_ADC LoadCellBlue(HX_BLUE_DOUT, HX_BLUE_SCK);

const int HX_RED_DOUT = 6;
const int HX_RED_SCK  = 7;
HX711_ADC LoadCellRed(HX_RED_DOUT, HX_RED_SCK);

const int ledRedPin  = 8;
const int ledBluePin = 11;

// Motor driver pins
const int pwmPin = 5;   // PWM output
const int dirPin = 4;   // direction
const int enaPin = 3;   // enable
const bool ENA_ACTIVE_LOW = true;

const int potPin = A0;

// fsr
const int fsrUpPin = A1;

// PG pulse input pin (interrupt-capable pin)
const uint8_t pgPin = 2;

// ---------- Calibration ----------
float calBlue = -105.53;
float calRed  = -105.98;

// ---------- Assist behavior ----------
const float LOAD_START_THRESHOLD = 00.0;
const float LOAD_STOP_THRESHOLD  = 375.0;
const int   PWM_STOP = 0;
const int   PWM_MIN  = 60;
const int   PWM_MAX  = 255;
// const int   LOAD_FULL_LB = 18143.7;
const int   LOAD_FULL_LB = 9000;

float loadBlueFilt = 0.0f;
float loadRedFilt = 0.0f;

// ---------- State Machine ----------
enum AssistState : uint8_t {
  IDLE = 0,
  ASSIST_BLUE = 1,
  ASSIST_RED  = 2
};

AssistState state = IDLE;

// ---------- Runtime variables ----------
float loadBlue = 0.0f;
float loadRed  = 0.0f;
int   potRaw   = 0;
float angleDeg = 0.0f;

// fsr
float fsrUpReading = 0.0f;

int pwmCmd = 0;
int dirCmd = 0; // 0/1 for logging

uint32_t lastLogMs = 0;

// PG pulse counting
volatile uint32_t pgPulseCount = 0;
uint32_t lastPgPulseCount = 0;
uint32_t lastPgMs = 0;

// ---------- Data model for logging ----------
struct ImuData {
  float ax, ay, az;
  float gx, gy, gz;
  float roll, pitch, yaw;
};

struct Sample {
  uint32_t t_ms;

  float loadBlue_lb;
  float loadRed_lb;

  int   pot_raw;
  float angle_deg;

  float fsr_up;

  uint8_t state;
  int dir_cmd;
  int pwm_cmd;

  float pg_hz;
  float motor_rpm;
  float output_rpm;

  ImuData imu;
};

// ---------- Forward declarations ----------
void pgIsr();
void setEnable(bool enabled);
void setMotor(int pwm, bool dirHigh);
void allLedsOff();
int  pwmFromLoad(float loadLb);

float readMotorRpmStub();
float readOutputRpmStub();
ImuData readImuStub();

#if ENABLE_IMU
bool initImu();
ImuData readImu();
#endif

void printCsvHeader();
void logSample(const Sample& s);

void updateSensors();
void updateStateMachine();
void applyControl();
void maybeLog();

// ---------- ISR ----------
void pgIsr() {
  pgPulseCount++;
}

// ---------- Helpers ----------
void setEnable(bool enabled) {
  if (ENA_ACTIVE_LOW) digitalWrite(enaPin, enabled ? LOW : HIGH);
  else                digitalWrite(enaPin, enabled ? HIGH : LOW);
}

void setMotor(int pwm, bool dirHigh) {
  digitalWrite(dirPin, dirHigh ? HIGH : LOW);
  analogWrite(pwmPin, constrain(pwm, 0, 255));
}

void allLedsOff() {
  digitalWrite(ledBluePin, LOW);
  digitalWrite(ledRedPin, LOW);
}

// Map load (lb) -> PWM
// int pwmFromLoad(float loadLb) {
//   if (loadLb < 0) loadLb = 0;
//   if (loadLb > LOAD_FULL_LB) loadLb = LOAD_FULL_LB;

//   float frac = loadLb / (float)LOAD_FULL_LB; // 0..1
//   int pwm = (int)(PWM_MIN + frac * (PWM_MAX - PWM_MIN));
//   return constrain(pwm, 0, 255);
// }
int pwmFromLoad(float loadLb) {
  if (loadLb <= LOAD_START_THRESHOLD) return PWM_MIN;
  if (loadLb >= LOAD_FULL_LB) return PWM_MAX;

  float frac = (loadLb - LOAD_START_THRESHOLD) / (LOAD_FULL_LB - LOAD_START_THRESHOLD); // 0..1
  int pwm = (int)(PWM_MIN + frac * (PWM_MAX - PWM_MIN));
  // int pwm = (int)(constantidk + (PWM_MIN + frac * (PWM_MAX - PWM_MIN)))
  return constrain(pwm, PWM_MIN, PWM_MAX);
}

// Stubs
float readMotorRpmStub()  { return -1.0f; }
float readOutputRpmStub() { return -1.0f; }

ImuData readImuStub() {
  ImuData d{};
  d.ax = d.ay = d.az = -1.0f;
  d.gx = d.gy = d.gz = -1.0f;
  d.roll = d.pitch = d.yaw = -1.0f;
  return d;
}

#if ENABLE_IMU
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
#endif

void printCsvHeader() {
#if LOG_SERIAL
  Serial.println(
    "t_ms,loadBlue_lb,loadRed_lb,pot_raw,angle_deg,fsr_up,state,dir_cmd,pwm_cmd,pg_hz,motor_rpm,output_rpm,"
    "ax,ay,az,gx,gy,gz,roll,pitch,yaw"
  );
#endif
}

void logSample(const Sample& s) {
#if LOG_SERIAL
Serial.print("time (s): "); Serial.print(s.t_ms / 1000); Serial.println(',');

  Serial.print("loadBlue: "); Serial.print(s.loadBlue_lb, 3); Serial.println(',');
  Serial.print("loadRed: "); Serial.print(s.loadRed_lb,  3); Serial.println(',');

  Serial.print("pot_raw: "); Serial.print(s.pot_raw);   Serial.println(',');
  Serial.print("angle_deg: "); Serial.print(s.angle_deg, 2); Serial.println(',');
  Serial.print("fsr_up: "); Serial.print(s.fsr_up, 2); Serial.println(',');

  Serial.print("state: "); Serial.print(s.state);     Serial.println(',');
  Serial.print("dir_cmd: "); Serial.print(s.dir_cmd);   Serial.println(',');
  Serial.print("pwm_cmd: "); Serial.print(s.pwm_cmd);   Serial.println(',');

  Serial.print("pg: "); Serial.print(s.pg_hz, 2); Serial.println(',');
  Serial.print("motor_rpm: "); Serial.print(s.motor_rpm, 2);  Serial.println(',');
  Serial.print("output_rpm: "); Serial.print(s.output_rpm, 2); Serial.println(',');

  Serial.print("ax: "); Serial.print(s.imu.ax, 3); Serial.println(',');
  Serial.print("ay: "); Serial.print(s.imu.ay, 3); Serial.println(',');
  Serial.print("az: "); Serial.print(s.imu.az, 3); Serial.println(',');
  Serial.print("gx: "); Serial.print(s.imu.gx, 3); Serial.println(',');
  Serial.print("gy: "); Serial.print(s.imu.gy, 3); Serial.println(',');
  Serial.print("gz: "); Serial.print(s.imu.gz, 3); Serial.println(',');
  Serial.print("roll: "); Serial.print(s.imu.roll, 3);  Serial.println(',');
  Serial.print("pitch: "); Serial.print(s.imu.pitch, 3); Serial.println(',');
  Serial.print("yaw: "); Serial.println(s.imu.yaw, 3);

  Serial.println("======================================================================================");
#endif
}

// Read sensors once per loop iteration
void updateSensors() {
  LoadCellBlue.update();
  LoadCellRed.update();

  // loadBlue = LoadCellBlue.getData();
  // loadRed  = LoadCellRed.getData();
  float rawBlue = LoadCellBlue.getData();
  float rawRed  = LoadCellRed.getData();

  const float ALPHA = 0.2f;  // try 0.15–0.3

  loadBlueFilt = ALPHA * rawBlue + (1.0f - ALPHA) * loadBlueFilt;
  loadRedFilt  = ALPHA * rawRed  + (1.0f - ALPHA) * loadRedFilt;


  potRaw = analogRead(potPin);
  // angleDeg = 270.0f - (270.0f * (float)potRaw / 1023.0f); // 0..1023 -> 270..0
  angleDeg = 270.0f * (float)potRaw / 1023.0f; // 0..1023 -> 270..0

  fsrUpReading = analogRead(fsrUpPin);  
}

void updateStateMachine() {
  switch (state) {
    case IDLE: {
      bool blueStart = (loadBlueFilt > LOAD_START_THRESHOLD);
      bool redStart  = (loadRedFilt  > LOAD_START_THRESHOLD);
      // 90 degree thing: replace LOAD_START_THRESHOLD with 90 degree angle pot or we coulddd do speed sensing and differentiate
      if (blueStart || redStart) {
        if (blueStart && (!redStart || loadBlueFilt >= loadRedFilt)) state = ASSIST_BLUE;
        else state = ASSIST_RED;
      }
      break;
    }

    case ASSIST_BLUE:
      if (loadBlueFilt < LOAD_STOP_THRESHOLD) state = IDLE;
      break;

    case ASSIST_RED:
      if (loadRedFilt < LOAD_STOP_THRESHOLD) state = IDLE;
      break;
  }
}

// Apply outputs based on current state
// // 35 to 135 with 0 being fully flexed and 180 being fully extended
// // when it is greater than 35 you can flex
// // when it is less than 135 you can extend
// blue is true (to in setMotor) is high is extenstion

void applyControl() {
  allLedsOff();

  // Default: motor off unless we explicitly enable it below
  bool runMotor = false;
  bool dirHigh  = false;
  int  pwm      = PWM_STOP;

  if (state == ASSIST_BLUE) {
    if (angleDeg < 135.0f) {
      if (fsrUpReading < 200.0f) {
        digitalWrite(ledBluePin, HIGH);
        dirCmd = 1;
        pwmCmd = pwmFromLoad(loadBlueFilt);

        runMotor = true;
        dirHigh  = true;
        pwm      = pwmCmd;
      }
    } else {
      // Angle not allowed -> command stop
      dirCmd = 0;
      pwmCmd = PWM_STOP;
    }

  } else if (state == ASSIST_RED) {
    if (angleDeg > 35.0f) {
      // if (fsrDownReading < 200.0f) {
        digitalWrite(ledRedPin, HIGH);
        dirCmd = 0;
        pwmCmd = pwmFromLoad(loadRedFilt);

        runMotor = true;
        dirHigh  = false;
        pwm      = pwmCmd;
      // }
    } else {
      dirCmd = 0;
      pwmCmd = PWM_STOP;
    }

  } else {
    // IDLE
    dirCmd = 0;
    pwmCmd = PWM_STOP;
  }

  // Apply outputs
  if (runMotor) {
    setEnable(true);
    setMotor(pwm, dirHigh);
  } else {
    setMotor(PWM_STOP, false);
    setEnable(false);  // disables driver when not running
  }
}

void maybeLog() {
  uint32_t now = millis();
  if (now - lastLogMs < LOG_PERIOD_MS) return;
  lastLogMs = now;

  Sample s{};
  s.t_ms = now;

  s.loadBlue_lb = loadBlueFilt;
  s.loadRed_lb  = loadRedFilt;

  s.pot_raw   = potRaw;
  s.angle_deg = angleDeg;
  s.fsr_up = fsrUpReading;

  s.state   = (uint8_t)state;
  s.dir_cmd = dirCmd;
  s.pwm_cmd = pwmCmd;

  uint32_t dtMs = now - lastPgMs;

  uint32_t pulses;
  noInterrupts();
  pulses = pgPulseCount;
  interrupts();

  uint32_t dp = pulses - lastPgPulseCount;

  float dtSec = (dtMs > 0) ? (dtMs / 1000.0f) : 0.0f;
  s.pg_hz = (dtSec > 0.0f) ? (dp / dtSec) : 0.0f;

  lastPgMs = now;
  lastPgPulseCount = pulses;

  s.motor_rpm  = readMotorRpmStub();
  s.output_rpm = readOutputRpmStub();

#if ENABLE_IMU
  s.imu = readImu();
#else
  s.imu = readImuStub();
#endif

  logSample(s);
}

void setup() {
#if LOG_SERIAL
  Serial.begin(SERIAL_BAUD);
  delay(10);
  Serial.println();
  Serial.println("Starting...");
#endif

  pinMode(ledRedPin, OUTPUT);
  pinMode(ledBluePin, OUTPUT);

  pinMode(pwmPin, OUTPUT);
  pinMode(dirPin, OUTPUT);
  pinMode(enaPin, OUTPUT);

  // PG input interrupt (DO NOT touch pwmPin here)
  pinMode(pgPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(pgPin), pgIsr, RISING);
  lastPgMs = millis();
  lastPgPulseCount = 0;

  allLedsOff();
  setEnable(false);
  setMotor(PWM_STOP, false);

  // HX711 init
  const unsigned long stabilizingtime = 2000;
  const bool doTare = true;

  LoadCellBlue.begin();
  LoadCellBlue.setSamplesInUse(4);
  LoadCellBlue.start(stabilizingtime, doTare);
  if (LoadCellBlue.getTareTimeoutFlag()) {
#if LOG_SERIAL
    Serial.println("Timeout on LoadCellBlue (blue), check wiring and pin designations");
#endif
    while (1) {}
  }
  LoadCellBlue.setCalFactor(calBlue);

  LoadCellRed.begin();
  LoadCellRed.setSamplesInUse(4);
  LoadCellRed.start(stabilizingtime, doTare);
  if (LoadCellRed.getTareTimeoutFlag()) {
#if LOG_SERIAL
    Serial.println("Timeout on LoadCellRed (red), check wiring and pin designations");
#endif
    while (1) {}
  }
  LoadCellRed.setCalFactor(calRed);

#if LOG_SERIAL
  Serial.println("HX711 startup is complete");
#endif

#if ENABLE_IMU
#if LOG_SERIAL
  if (!initImu()) Serial.println("IMU init failed (LSM6DSOX). Check wiring/I2C address.");
  else            Serial.println("IMU init OK");
#endif
#endif

  printCsvHeader();
  lastLogMs = millis();
}

void loop() {
  updateSensors();
  updateStateMachine();
  applyControl();
  maybeLog();
}

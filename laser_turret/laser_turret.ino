/*
 * laser_turret.ino
 * Double-axis turret controlled by tilting an MPU-6050.
 */
#include <Wire.h>
#include <Servo.h>
const uint8_t MPU_ADDR = 0x68;          // I2C address (AD0 left unconnected)

const uint8_t REG_PWR_MGMT_1   = 0x6B;  // power management, chip starts asleep
const uint8_t REG_ACCEL_CONFIG = 0x1C;  // accelerometer range setting
const uint8_t REG_WHO_AM_I     = 0x75;  // identity register, normally 0x68

const uint8_t REG_ACCEL_XOUT_H = 0x3B;  // first of 6 accel bytes (XH XL YH YL ZH ZL)

const uint16_t SAMPLE_PERIOD_MS = 10;   // read sensor at ~100 Hz
const uint16_t PRINT_PERIOD_MS  = 100;  // print at ~10 Hz

const float SMOOTHING = 0.2f;   // 0..1, lower = smoother but laggier

const uint8_t PAN_PIN  = 9;    // servo that follows roll  (left/right)
const uint8_t TILT_PIN = 10;   // servo that follows pitch (up/down)

// If a servo moves the wrong way, flip its flag to true.
const bool INVERT_PAN  = false;
const bool INVERT_TILT = false;

const uint8_t WINDOW_SIZE = 128;   // rolling window length required by the trial

struct AccelData {
  int16_t x;
  int16_t y;
  int16_t z;
};

/*
 * RollingStats: keeps the most recent WINDOW_SIZE samples in a fixed size
 * circular buffer and computes mean / standard deviation on demand.
 * The buffer is a plain array inside the object
 */
class RollingStats {
 public:
  RollingStats() : head_(0), count_(0) {
    for (uint8_t i = 0; i < WINDOW_SIZE; i++) {
      samples_[i] = 0;
    }
  }

  // Add one sample, once full, the oldest sample is overwritten.
  void add(int16_t value) {
    samples_[head_] = value;
    head_ = (head_ + 1) % WINDOW_SIZE;
    if (count_ < WINDOW_SIZE) {
      count_++;
    }
  }

  uint8_t size() const { return count_; }

  float mean() const {
    if (count_ == 0) return 0.0f;
    int32_t sum = 0;   // 128 * 32768 fits in 32 bits
    for (uint8_t i = 0; i < count_; i++) {
      sum += samples_[i];
    }
    return (float)sum / (float)count_;
  }

  // Population standard deviation: sqrt( sum((x - mean)^2) / N )
  float stdDev() const {
    if (count_ < 2) return 0.0f;
    float m = mean();
    float acc = 0.0f;
    for (uint8_t i = 0; i < count_; i++) {
      float d = (float)samples_[i] - m;
      acc += d * d;
    }
    return sqrt(acc / (float)count_);
  }

 private:
  int16_t samples_[WINDOW_SIZE];
  uint8_t head_;    // index where the next sample will be written
  uint8_t count_;   // how many valid samples we hold (max WINDOW_SIZE)
};

AccelData latest = {0, 0, 0};
unsigned long lastSampleMs = 0;
unsigned long lastPrintMs  = 0;

unsigned long lastBeatMs = 0;
bool ledOn = false;

float latestRoll  = 0.0f;   // degrees, -90..90
float latestPitch = 0.0f;

float filteredRoll  = 0.0f;
float filteredPitch = 0.0f;

Servo panServo;
Servo tiltServo;
int panCmd  = 90;
int tiltCmd = 90;

RollingStats axStats;   // rolling stats of the RAW X axis reading

// Try every 7-bit address and print the ones that answer.
// void scanI2C() {
//   Serial.println(F("Scanning I2C bus..."));
//   uint8_t found = 0;
//   for (uint8_t addr = 1; addr < 127; addr++) {
//     Wire.beginTransmission(addr);
//     if (Wire.endTransmission() == 0) {   // 0 = device acknowledged
//       Serial.print(F("  device at 0x"));
//       Serial.println(addr, HEX);
//       found++;
//     }
//   }
//   if (found == 0) {
//     Serial.println(F("  no devices found - check wiring"));
//   }
// }
// Write one byte to one MPU register. Returns true on success.
bool mpuWrite(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.write(value);
  return Wire.endTransmission() == 0;   // 0 means the MPU acknowledged
}

// Read the WHO_AM_I register. Returns -1 if the MPU did not respond.
int mpuWhoAmI() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(REG_WHO_AM_I);
  if (Wire.endTransmission(false) != 0) return -1;
  if (Wire.requestFrom(MPU_ADDR, (uint8_t)1) != 1) return -1;
  return Wire.read();
}

// Combine two bytes, high first, into one signed 16 bit value.
int16_t readInt16() {
  uint8_t high = Wire.read();
  uint8_t low  = Wire.read();
  return (int16_t)(((uint16_t)high << 8) | low);
}

// Read raw X, Y, Z acceleration (6 bytes starting at ACCEL_XOUT_H).
bool readAccel(AccelData &out) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(REG_ACCEL_XOUT_H);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom(MPU_ADDR, (uint8_t)6) != 6) return false;
  out.x = readInt16();
  out.y = readInt16();
  out.z = readInt16();
  return true;
}

float clampAngle(float a) {
  if (a > 90.0f)  return 90.0f;
  if (a < -90.0f) return -90.0f;
  return a;
}

// Convert a tilt angle (-90..+90 degrees) to a servo command (0..180).
int angleToServo(float angleDeg, bool invert) {
  if (angleDeg > 90.0f)  angleDeg = 90.0f;
  if (angleDeg < -90.0f) angleDeg = -90.0f;
  if (invert) angleDeg = -angleDeg;
  return (int)(90.0f + angleDeg + 0.5f);   // +0.5 rounds to nearest
}

#define RUN_SELF_TESTS 1   // set to 0 to skip the self test at boot

#if RUN_SELF_TESTS
bool nearlyEqual(float a, float b, float tol) {
  return fabs(a - b) <= tol;
}

void report(const __FlashStringHelper* name, bool pass) {
  Serial.print(F("SELFTEST "));
  Serial.print(name);
  Serial.println(pass ? F(": PASS") : F(": FAIL"));
}

void runSelfTests() {
  {  // full window of 1..128
    RollingStats s;
    for (int16_t i = 1; i <= WINDOW_SIZE; i++) s.add(i);
    report(F("full window"),
           s.size() == WINDOW_SIZE &&
           nearlyEqual(s.mean(), 64.5f, 0.01f) &&
           nearlyEqual(s.stdDev(), 36.95f, 0.05f));

    s.add(129);   // oldest (1) drops out; window is now 2..129
    report(F("rolling slide"),
           s.size() == WINDOW_SIZE &&
           nearlyEqual(s.mean(), 65.5f, 0.01f) &&
           nearlyEqual(s.stdDev(), 36.95f, 0.05f));
  }
  {  // constant input has zero spread
    RollingStats c;
    for (uint8_t i = 0; i < WINDOW_SIZE; i++) c.add(500);
    report(F("constant input"),
           nearlyEqual(c.mean(), 500.0f, 0.01f) &&
           nearlyEqual(c.stdDev(), 0.0f, 0.01f));
  }
  {  // partially filled window: {10, 20} -> mean 15, std 5
    RollingStats p;
    p.add(10);
    p.add(20);
    report(F("partial window"),
           p.size() == 2 &&
           nearlyEqual(p.mean(), 15.0f, 0.01f) &&
           nearlyEqual(p.stdDev(), 5.0f, 0.01f));
  }
}
#endif

void setup() {
  Serial.begin(115200);
  #if RUN_SELF_TESTS
  runSelfTests();
  #endif
  Serial.println(F("Turret firmware booting..."));

  Wire.begin();
  Wire.setClock(400000);   // 400 kHz fast mode

  // Wake the MPU (it boots in sleep mode).
  if (!mpuWrite(REG_PWR_MGMT_1, 0x00)) {
    while (true) {
      Serial.println(F("ERROR: MPU-6050 not responding. Check VCC/GND/SDA/SCL wiring."));
      delay(2000);
    }
  }
  mpuWrite(REG_ACCEL_CONFIG, 0x00);   // +/-2 g full scale range

  int id = mpuWhoAmI();

  Serial.print(F("WHO_AM_I = 0x"));
  Serial.println(id, HEX);
  panServo.attach(PAN_PIN);
  tiltServo.attach(TILT_PIN);
  panServo.write(90);    // start centered
  tiltServo.write(90);
  Serial.println(F("Turret ready. Tilt the MPU-6050."));
}

void loop() {
  unsigned long now = millis();

  if (now - lastSampleMs >= SAMPLE_PERIOD_MS) {
    lastSampleMs = now;
    AccelData a;
    if (readAccel(a)) {
      latest = a;
      axStats.add(a.x);   // rolling window of RAW x readings

      float fx = (float)a.x;
      float fy = (float)a.y;
      float fz = (float)a.z;

      float roll  = clampAngle(atan2(fy, fz) * RAD_TO_DEG);
      float pitch = clampAngle(atan2(-fx, sqrt(fy * fy + fz * fz)) * RAD_TO_DEG);

      filteredRoll  = SMOOTHING * roll  + (1.0f - SMOOTHING) * filteredRoll;
      filteredPitch = SMOOTHING * pitch + (1.0f - SMOOTHING) * filteredPitch;

      latestRoll  = filteredRoll;
      latestPitch = filteredPitch;

      panCmd  = angleToServo(filteredRoll,  INVERT_PAN);
      tiltCmd = angleToServo(filteredPitch, INVERT_TILT);
      panServo.write(panCmd);
      tiltServo.write(tiltCmd);
    }
  }

  if (now - lastPrintMs >= PRINT_PERIOD_MS) {
    lastPrintMs = now;
    Serial.print(F("ax=")); Serial.print(latest.x);
    Serial.print(F(" ay=")); Serial.print(latest.y);
    Serial.print(F(" az=")); Serial.print(latest.z);
    Serial.print(F(" | roll=")); Serial.print(latestRoll, 1);
    Serial.print(F(" pitch=")); Serial.println(latestPitch, 1);
    Serial.print(F(" | pan=")); Serial.print(panCmd);
    Serial.print(F(" tilt=")); Serial.println(tiltCmd);
    Serial.print(F(" | ax[n=")); Serial.print(axStats.size());
    Serial.print(F("] mean=")); Serial.print(axStats.mean(), 1);
    Serial.print(F(" std=")); Serial.println(axStats.stdDev(), 1);
  }
}
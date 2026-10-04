/*
 * laser_turret.ino
 * Double-axis turret controlled by tilting an MPU-6050.
 */
#include <Wire.h>
const uint8_t MPU_ADDR = 0x68;          // I2C address (AD0 left unconnected)

const uint8_t REG_PWR_MGMT_1   = 0x6B;  // power management, chip starts asleep
const uint8_t REG_ACCEL_CONFIG = 0x1C;  // accelerometer range setting
const uint8_t REG_WHO_AM_I     = 0x75;  // identity register, normally 0x68

const uint8_t REG_ACCEL_XOUT_H = 0x3B;  // first of 6 accel bytes (XH XL YH YL ZH ZL)

const uint16_t SAMPLE_PERIOD_MS = 10;   // read sensor at ~100 Hz
const uint16_t PRINT_PERIOD_MS  = 100;  // print at ~10 Hz

struct AccelData {
  int16_t x;
  int16_t y;
  int16_t z;
};

AccelData latest = {0, 0, 0};
unsigned long lastSampleMs = 0;
unsigned long lastPrintMs  = 0;

unsigned long lastBeatMs = 0;
bool ledOn = false;

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

void setup() {
  Serial.begin(115200);
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
}

void loop() {
  unsigned long now = millis();

  // Sample at a fixed rate
  if (now - lastSampleMs >= SAMPLE_PERIOD_MS) {
    lastSampleMs = now;
    AccelData a;
    if (readAccel(a)) {
      latest = a;
    }
  }

  // Print at a slower rate
  if (now - lastPrintMs >= PRINT_PERIOD_MS) {
    lastPrintMs = now;
    Serial.print(F("ax=")); Serial.print(latest.x);
    Serial.print(F(" ay=")); Serial.print(latest.y);
    Serial.print(F(" az=")); Serial.println(latest.z);
  }
}
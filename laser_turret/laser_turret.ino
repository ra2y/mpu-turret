/*
 * laser_turret.ino
 * Double-axis turret controlled by tilting an MPU-6050.
 */
#include <Wire.h>
const uint8_t MPU_ADDR = 0x68;          // I2C address (AD0 left unconnected)

const uint8_t REG_PWR_MGMT_1   = 0x6B;  // power management, chip starts asleep
const uint8_t REG_ACCEL_CONFIG = 0x1C;  // accelerometer range setting
const uint8_t REG_WHO_AM_I     = 0x75;  // identity register, normally 0x68
const unsigned long HEARTBEAT_MS = 500;

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

void setup() {
  Serial.begin(115200);
  pinMode(LED_BUILTIN, OUTPUT);
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
  if (now - lastBeatMs >= HEARTBEAT_MS) {
    lastBeatMs = now;
    ledOn = !ledOn;
    digitalWrite(LED_BUILTIN, ledOn);
    Serial.println(F("heartbeat"));
  }
}

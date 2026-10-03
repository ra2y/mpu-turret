/*
 * laser_turret.ino
 * Double-axis turret controlled by tilting an MPU-6050.
 */
#include <Wire.h>
const unsigned long HEARTBEAT_MS = 500;

unsigned long lastBeatMs = 0;
bool ledOn = false;

// Try every 7-bit address and print the ones that answer.
void scanI2C() {
  Serial.println(F("Scanning I2C bus..."));
  uint8_t found = 0;
  for (uint8_t addr = 1; addr < 127; addr++) {
    Wire.beginTransmission(addr);
    if (Wire.endTransmission() == 0) {   // 0 = device acknowledged
      Serial.print(F("  device at 0x"));
      Serial.println(addr, HEX);
      found++;
    }
  }
  if (found == 0) {
    Serial.println(F("  no devices found - check wiring"));
  }
}

void setup() {
  Serial.begin(115200);
  pinMode(LED_BUILTIN, OUTPUT);
  Serial.println(F("Turret firmware booting..."));
  Wire.begin();   // join the bus as controller (A4 = SDA, A5 = SCL)
  scanI2C();
}

void loop() {
  unsigned long now = millis();
  if (now - lastBeatMs >= HEARTBEAT_MS) {
    lastBeatMs = now;
    ledOn = !ledOn;
    digitalWrite(LED_BUILTIN, ledOn);
    Serial.println(F("heartbeat"));
  }
  stop
}

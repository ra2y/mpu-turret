/*
 * laser_turret.ino
 * Double axis turret controlled by tilting an MPU-6050.
 */

const unsigned long HEARTBEAT_MS = 500;

unsigned long lastBeatMs = 0;
bool ledOn = false;

void setup() {
  Serial.begin(115200);
  pinMode(LED_BUILTIN, OUTPUT);
  Serial.println(F("Turret firmware booting..."));
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
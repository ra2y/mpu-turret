#include <Servo.h>

const uint8_t PAN_PIN  = 9;
const uint8_t TILT_PIN = 10;

Servo panServo;
Servo tiltServo;

// Sweep one servo 45 -> 135 -> 45 (a safe range away from the mechanical stops).
void sweep(Servo &servo) {
  for (int a = 45; a <= 135; a++) { servo.write(a); delay(15); }
  for (int a = 135; a >= 45; a--) { servo.write(a); delay(15); }
  servo.write(90);
}

void setup() {
  Serial.begin(115200);
  panServo.attach(PAN_PIN);
  tiltServo.attach(TILT_PIN);
  panServo.write(90);
  tiltServo.write(90);
  delay(1000);
}

void loop() {
  Serial.println(F("Sweeping PAN"));
  sweep(panServo);
  delay(500);
  Serial.println(F("Sweeping TILT"));
  sweep(tiltServo);
  delay(500);
}
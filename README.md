# MPU-6050 Laser Turret

A two-axis turret that follows
the tilt of an MPU-6050 accelerometer.

## Hardware
Arduino Nano, GY-521 MPU-6050, two micro servos.

| MPU-6050 | Nano |
|---|---|
| VCC | 5V |
| GND | GND |
| SCL | A5 |
| SDA | A4 |

| Servo | Pin |
|---|---|
| Pan (roll) | D9 |
| Tilt (pitch) | D10 |

Servo power and ground come from the 5V and GND rails.

## How it works
1. Wake the MPU and read raw acceleration over I2C using only the Wire library.
2. Compute roll and pitch from the gravity vector, smooth with an exponential average.
3. Map -90..90 degrees to 0..180 servo commands on two orthogonal axes.
4. Print every raw value used, plus mean and std of raw ax over a rolling window of 128 samples.

## Build and run
Arduino IDE: Board = Arduino Nano, Processor = ATmega328P (Old Bootloader),
open laser_turret/laser_turret.ino, upload, open Serial Monitor at 115200 baud.

## Testing
See docs/test-log.md and tools/servo_test.
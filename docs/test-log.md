# Test log

| Step | What I tested | Result |
|------|---------------|--------|
| 1 | Upload + serial at 115200, LED blink | PASS |
| 2 | I2C scan finds 0x68, unplugged SDA shows none found | PASS |
| 3 | WHO_AM_I=0x68, error message with SDA unplugged | PASS |
| 4 | Flat: az~16384, ax/ay~0, each edge up gives ~16384, magnitude ~16384 | PASS |
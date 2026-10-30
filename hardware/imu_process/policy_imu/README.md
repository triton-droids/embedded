# ESP32-S3 policy IMU telemetry

`policy_imu.ino` is the firmware used for the ROS 2 tracking-policy bench.
It samples at 200 Hz and emits one SI JSON line per sample over USB CDC at
460800 baud: `t_us`, `seq`, `accel_mps2`, and `gyro_rad_s`.
`t_us` is a wrapping 32-bit microsecond counter.

The same I2C bus may also contain a PCA9685 at address `0x40`. The ninth
physical PCA9685 connector is zero-based channel 8. It is configured for
50 Hz, but remains full-off after boot so flashing or resetting the ESP32 does
not move the servo. Send one of these newline-terminated commands over the
same full-duplex USB CDC port:

```text
P,1500
C,0,1500
STATUS
OFF
```

`P,<microseconds>` accepts only `1000..2000`. Start unloaded at `P,1500`, then
change in small increments. `C,<channel>,<microseconds>` is the diagnostic form
for selecting any channel `0..15`; changing channels disables the previous one.
`OFF` sets all PCA9685 channels full-off, releasing a DS3225 rather than holding
its current position. Responses begin with `#`, so the existing ROS IMU reader
ignores them. The DS3225 must use an independent servo supply connected to
PCA9685 `V+`; never power it from ESP32 USB/3V3.

The tested wiring uses SDA GPIO4, SCL GPIO5, I2C 100 kHz, address 0x68,
and a sensor reporting WHO_AM_I 0x70. The sketch accepts supported MPU-family
register IDs and uses +/-2 g and +/-250 degrees/s ranges. SDA/SCL can be
overridden using the `IMU_SDA` / `IMU_SCL` compile definitions.

Using Arduino CLI with the Espressif ESP32 board core already installed:

```bash
cd ~/Github/embedded
arduino-cli compile \
  --fqbn esp32:esp32:esp32s3:FlashSize=8M,CDCOnBoot=default \
  hardware/imu_process/policy_imu
# Stop the ROS reader before uploading to its USB port.
arduino-cli upload \
  --fqbn esp32:esp32:esp32s3:FlashSize=8M,CDCOnBoot=default \
  --port /dev/ttyACM0 hardware/imu_process/policy_imu
```

The tested board used 8 MB flash; select the board configuration matching the
actual device. Firmware is already installed on the tested bench, so ordinary
policy starts do not need another upload. Diagnostic lines begin with `#` and
are ignored by the JSON reader. This sketch does not communicate with motors.

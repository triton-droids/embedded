# ESP32-S3 policy IMU telemetry

`policy_imu.ino` is the firmware used for the ROS 2 tracking-policy bench.
It samples at 200 Hz and emits one SI JSON line per sample over USB CDC at
460800 baud: `t_us`, `seq`, `accel_mps2`, and `gyro_rad_s`.
`t_us` is a wrapping 32-bit microsecond counter.

The tested wiring uses SDA GPIO4, SCL GPIO5, I2C 100 kHz, address 0x68,
and a sensor reporting WHO_AM_I 0x70. The sketch accepts supported MPU-family
register IDs and uses +/-2 g and +/-250 degrees/s ranges. SDA/SCL can be
overridden using the `IMU_SDA` / `IMU_SCL` compile definitions.

Using Arduino CLI with the Espressif ESP32 board core already installed:

```bash
cd ~/Github/embedded
arduino-cli compile \
  --fqbn esp32:esp32:esp32s3:FlashSize=8M,CDCOnBoot=default \
  imu_process/policy_imu
# Stop the ROS reader before uploading to its USB port.
arduino-cli upload \
  --fqbn esp32:esp32:esp32s3:FlashSize=8M,CDCOnBoot=default \
  --port /dev/ttyACM0 imu_process/policy_imu
```

The tested board used 8 MB flash; select the board configuration matching the
actual device. Firmware is already installed on the tested bench, so ordinary
policy starts do not need another upload. Diagnostic lines begin with `#` and
are ignored by the JSON reader. This sketch does not communicate with motors.

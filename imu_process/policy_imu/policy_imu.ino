// ESP32-S3 MPU6050 bench telemetry. No CAN, Wi-Fi, or motor output.
#include <Arduino.h>
#include <Wire.h>

#ifndef IMU_SDA
#define IMU_SDA 4
#endif
#ifndef IMU_SCL
#define IMU_SCL 5
#endif

constexpr uint32_t PERIOD_US = 5000;  // 200 Hz IMU, independent of policy frequency.
uint8_t imuAddress = 0;
uint32_t sequenceNumber = 0;
uint32_t nextSample = 0;
bool ready = false;

bool writeRegister(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(imuAddress);
  Wire.write(reg);
  Wire.write(value);
  return Wire.endTransmission() == 0;
}

bool readRegisters(uint8_t reg, uint8_t* buffer, size_t count) {
  Wire.beginTransmission(imuAddress);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom(imuAddress, (uint8_t)count) != count) return false;
  for (size_t i = 0; i < count; ++i) buffer[i] = Wire.read();
  return true;
}

int16_t signed16(const uint8_t* p) {
  return (int16_t)((uint16_t)p[0] << 8 | p[1]);
}

void setup() {
  Serial.begin(460800);
  delay(500);
  Wire.begin(IMU_SDA, IMU_SCL);
  Wire.setClock(100000);  // Match the known working scanner wiring/bus speed.
  Wire.setTimeOut(20);
  for (uint8_t addr : {uint8_t(0x68), uint8_t(0x69)}) {
    Wire.beginTransmission(addr);
    if (Wire.endTransmission() == 0) {
      imuAddress = addr;
      break;
    }
  }
  if (!imuAddress) {
    Serial.printf("# no_IMU SDA=%d SCL=%d\n", IMU_SDA, IMU_SCL);
    return;
  }
  uint8_t who = 0;
  if (!readRegisters(0x75, &who, 1) || (who != 0x68 && who != 0x70 && who != 0x71 && who != 0x73)) {
    Serial.printf("# unsupported_IMU WHO_AM_I=0x%02X\n", who);
    return;
  }
  // Reset, select gyro PLL, all axes on, +/-250 deg/s, +/-2g, DLPF ~44Hz.
  if (!writeRegister(0x6B, 0x80)) return;
  delay(100);
  ready = writeRegister(0x6B, 0x01) && writeRegister(0x6C, 0x00) &&
          writeRegister(0x1B, 0x00) && writeRegister(0x1C, 0x00) &&
          writeRegister(0x1A, 0x03) && writeRegister(0x19, 0x04);
  if (who != 0x68) ready = ready && writeRegister(0x1D, 0x03);
  delay(100);
  Serial.printf("# policy_IMU addr=0x%02X who=0x%02X SDA=%d SCL=%d hz=200 baud=460800 units=SI\n", imuAddress, who, IMU_SDA, IMU_SCL);
  nextSample = micros() + PERIOD_US;
}

void loop() {
  if (!ready) {
    Serial.printf("# IMU_not_ready SDA=%d SCL=%d addr=0x%02X\n", IMU_SDA, IMU_SCL, imuAddress);
    delay(1000);
    return;
  }
  uint32_t now = micros();
  if ((int32_t)(now - nextSample) < 0) { delayMicroseconds(100); return; }
  nextSample += PERIOD_US;
  if ((int32_t)(now - nextSample) >= 0) nextSample = now + PERIOD_US;
  uint8_t data[14];
  if (!readRegisters(0x3B, data, sizeof(data))) {
    Serial.println("# read_fail");
    return;
  }
  float accel[3], gyro[3];
  for (int i = 0; i < 3; ++i) {
    accel[i] = signed16(data + i * 2) * (9.80665f / 16384.0f);
    gyro[i] = signed16(data + 8 + i * 2) * (PI / (180.0f * 131.0f));
  }
  Serial.printf("{\"t_us\":%lu,\"seq\":%lu,\"accel_mps2\":[%.5f,%.5f,%.5f],\"gyro_rad_s\":[%.6f,%.6f,%.6f]}\n",
                (unsigned long)now, (unsigned long)sequenceNumber++, accel[0], accel[1], accel[2], gyro[0], gyro[1], gyro[2]);
}

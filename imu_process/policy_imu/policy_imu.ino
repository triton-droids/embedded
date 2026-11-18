// ESP32-S3 IMU telemetry plus a safe PCA9685/DS3225 bench interface.
#include <Arduino.h>
#include <Wire.h>

#ifndef IMU_SDA
#define IMU_SDA 4
#endif
#ifndef IMU_SCL
#define IMU_SCL 5
#endif

constexpr uint32_t PERIOD_US = 5000;  // 200 Hz IMU, independent of policy frequency.
constexpr uint8_t PCA9685_ADDRESS = 0x40;
constexpr uint8_t SERVO_CHANNEL = 8;  // Ninth physical PCA9685 output.
constexpr float SERVO_FREQUENCY_HZ = 50.0f;
constexpr uint16_t SERVO_MIN_US = 1000;
constexpr uint16_t SERVO_MAX_US = 2000;

constexpr uint8_t PCA_MODE1 = 0x00;
constexpr uint8_t PCA_MODE2 = 0x01;
constexpr uint8_t PCA_LED0_ON_L = 0x06;
constexpr uint8_t PCA_PRESCALE = 0xFE;
constexpr uint8_t PCA_MODE1_RESTART = 0x80;
constexpr uint8_t PCA_MODE1_SLEEP = 0x10;
constexpr uint8_t PCA_MODE1_AUTO_INCREMENT = 0x20;
constexpr uint8_t PCA_FULL_OFF = 0x10;

uint8_t imuAddress = 0;
uint32_t sequenceNumber = 0;
uint32_t nextSample = 0;
bool ready = false;
bool pcaReady = false;
uint8_t pcaMode1Readback = 0xFF;
uint8_t pcaPrescaleReadback = 0xFF;
bool servoEnabled = false;
uint8_t activeServoChannel = SERVO_CHANNEL;
uint16_t servoPulseUs = 0;
String commandBuffer;

bool i2cWriteRegister(uint8_t address, uint8_t reg, uint8_t value) {
  Wire.beginTransmission(address);
  Wire.write(reg);
  Wire.write(value);
  return Wire.endTransmission() == 0;
}

bool i2cReadRegister(uint8_t address, uint8_t reg, uint8_t* value) {
  Wire.beginTransmission(address);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom(address, uint8_t(1)) != 1) return false;
  *value = Wire.read();
  return true;
}

bool imuWriteRegister(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(imuAddress);
  Wire.write(reg);
  Wire.write(value);
  return Wire.endTransmission() == 0;
}

bool imuReadRegisters(uint8_t reg, uint8_t* buffer, size_t count) {
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

void pcaSetFullOff(uint8_t channel) {
  const uint8_t base = PCA_LED0_ON_L + 4U * channel;
  Wire.beginTransmission(PCA9685_ADDRESS);
  Wire.write(base);
  Wire.write(uint8_t(0));
  Wire.write(uint8_t(0));
  Wire.write(uint8_t(0));
  Wire.write(PCA_FULL_OFF);
  Wire.endTransmission();
}

void pcaSetAllFullOff() {
  for (uint8_t channel = 0; channel < 16; ++channel) {
    pcaSetFullOff(channel);
  }
}

bool pcaSetPulseUs(uint8_t channel, uint16_t pulseUs) {
  const uint16_t ticks = uint16_t(
      lroundf(float(pulseUs) * SERVO_FREQUENCY_HZ * 4096.0f / 1000000.0f));
  const uint8_t base = PCA_LED0_ON_L + 4U * channel;

  Wire.beginTransmission(PCA9685_ADDRESS);
  Wire.write(base);
  Wire.write(uint8_t(0));
  Wire.write(uint8_t(0));
  Wire.write(uint8_t(ticks & 0xFFU));
  Wire.write(uint8_t((ticks >> 8U) & 0x0FU));
  return Wire.endTransmission() == 0;
}

bool pcaBegin() {
  Wire.beginTransmission(PCA9685_ADDRESS);
  if (Wire.endTransmission() != 0) return false;

  uint8_t oldMode = 0;
  if (!i2cReadRegister(PCA9685_ADDRESS, PCA_MODE1, &oldMode)) return false;

  const uint8_t sleepMode =
      (oldMode & ~PCA_MODE1_RESTART) | PCA_MODE1_SLEEP;
  const uint8_t wakeMode =
      (oldMode & ~PCA_MODE1_SLEEP) | PCA_MODE1_AUTO_INCREMENT;
  const uint8_t prescale = uint8_t(
      lroundf(25000000.0f / (4096.0f * SERVO_FREQUENCY_HZ)) - 1.0f);

  if (!i2cWriteRegister(PCA9685_ADDRESS, PCA_MODE1, sleepMode) ||
      !i2cWriteRegister(PCA9685_ADDRESS, PCA_PRESCALE, prescale) ||
      !i2cWriteRegister(PCA9685_ADDRESS, PCA_MODE2, 0x04) ||
      !i2cWriteRegister(PCA9685_ADDRESS, PCA_MODE1, wakeMode)) {
    return false;
  }

  delay(1);
  if (!i2cWriteRegister(PCA9685_ADDRESS, PCA_MODE1,
                        wakeMode | PCA_MODE1_RESTART)) {
    return false;
  }

  if (!i2cReadRegister(PCA9685_ADDRESS, PCA_MODE1, &pcaMode1Readback) ||
      !i2cReadRegister(PCA9685_ADDRESS, PCA_PRESCALE, &pcaPrescaleReadback) ||
      (pcaMode1Readback & PCA_MODE1_SLEEP) != 0 ||
      pcaPrescaleReadback != prescale) {
    return false;
  }

  // Safe startup: every output remains electrically released.
  pcaSetAllFullOff();
  return true;
}

bool applyServoPulse(uint8_t channel, uint16_t pulseUs) {
  if (servoEnabled && activeServoChannel != channel) {
    pcaSetFullOff(activeServoChannel);
  }
  if (!pcaSetPulseUs(channel, pulseUs)) {
    pcaSetFullOff(channel);
    servoEnabled = false;
    servoPulseUs = 0;
    return false;
  }
  activeServoChannel = channel;
  servoEnabled = true;
  servoPulseUs = pulseUs;
  return true;
}

void handleServoCommand(const String& line) {
  if (line == "OFF") {
    if (pcaReady) pcaSetAllFullOff();
    servoEnabled = false;
    servoPulseUs = 0;
    Serial.println("# servo_off all_channels");
    return;
  }

  if (line == "STATUS") {
    Serial.printf("# servo_status pca=%u channel=%u enabled=%u pulse_us=%u\n",
                  pcaReady, activeServoChannel, servoEnabled, servoPulseUs);
    return;
  }

  if (line.startsWith("C,")) {
    const int separator = line.indexOf(',', 2);
    if (separator < 0) {
      Serial.println("# command_error expected=C,<0..15>,<1000..2000>");
      return;
    }
    const long channel = line.substring(2, separator).toInt();
    const long requested = line.substring(separator + 1).toInt();
    if (!pcaReady || channel < 0 || channel > 15 ||
        requested < SERVO_MIN_US || requested > SERVO_MAX_US) {
      Serial.printf("# channel_pulse_rejected pca=%u channel=%ld pulse_us=%ld\n",
                    pcaReady, channel, requested);
      return;
    }
    if (!applyServoPulse(uint8_t(channel), uint16_t(requested))) {
      Serial.println("# servo_write_failed output_disabled");
      return;
    }
    Serial.printf("# servo_set channel=%u pulse_us=%u\n",
                  activeServoChannel, servoPulseUs);
    return;
  }

  if (!line.startsWith("P,")) {
    Serial.println("# command_error expected=P,<1000..2000>|C,<0..15>,<1000..2000>|OFF|STATUS");
    return;
  }

  const long requested = line.substring(2).toInt();
  if (!pcaReady || requested < SERVO_MIN_US || requested > SERVO_MAX_US) {
    Serial.printf("# pulse_rejected pca=%u requested=%ld range=%u..%u\n",
                  pcaReady, requested, SERVO_MIN_US, SERVO_MAX_US);
    return;
  }

  if (!applyServoPulse(SERVO_CHANNEL, uint16_t(requested))) {
    Serial.println("# servo_write_failed output_disabled");
    return;
  }

  Serial.printf("# servo_set channel=%u pulse_us=%u\n",
                activeServoChannel, servoPulseUs);
}

void readSerialCommands() {
  while (Serial.available() > 0) {
    const char c = char(Serial.read());
    if (c == '\n' || c == '\r') {
      if (!commandBuffer.isEmpty()) {
        handleServoCommand(commandBuffer);
        commandBuffer = "";
      }
      continue;
    }

    if (commandBuffer.length() >= 63) {
      commandBuffer = "";
      Serial.println("# command_error line_too_long");
      continue;
    }
    commandBuffer += c;
  }
}

void setup() {
  Serial.begin(460800);
  delay(500);
  Wire.begin(IMU_SDA, IMU_SCL);
  Wire.setClock(100000);  // Match the known working scanner wiring/bus speed.
  Wire.setTimeOut(20);
  pcaReady = pcaBegin();
  Serial.printf("# PCA9685 addr=0x%02X ready=%u mode1=0x%02X prescale=%u channel=%u startup=OFF hz=%.0f\n",
                PCA9685_ADDRESS, pcaReady, pcaMode1Readback,
                pcaPrescaleReadback, SERVO_CHANNEL, SERVO_FREQUENCY_HZ);
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
  if (!imuReadRegisters(0x75, &who, 1) || (who != 0x68 && who != 0x70 && who != 0x71 && who != 0x73)) {
    Serial.printf("# unsupported_IMU WHO_AM_I=0x%02X\n", who);
    return;
  }
  // Reset, select gyro PLL, all axes on, +/-250 deg/s, +/-2g, DLPF ~44Hz.
  if (!imuWriteRegister(0x6B, 0x80)) return;
  delay(100);
  ready = imuWriteRegister(0x6B, 0x01) && imuWriteRegister(0x6C, 0x00) &&
          imuWriteRegister(0x1B, 0x00) && imuWriteRegister(0x1C, 0x00) &&
          imuWriteRegister(0x1A, 0x03) && imuWriteRegister(0x19, 0x04);
  if (who != 0x68) ready = ready && imuWriteRegister(0x1D, 0x03);
  delay(100);
  Serial.printf("# policy_IMU addr=0x%02X who=0x%02X SDA=%d SCL=%d hz=200 baud=460800 units=SI\n", imuAddress, who, IMU_SDA, IMU_SCL);
  nextSample = micros() + PERIOD_US;
}

void loop() {
  readSerialCommands();
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
  if (!imuReadRegisters(0x3B, data, sizeof(data))) {
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

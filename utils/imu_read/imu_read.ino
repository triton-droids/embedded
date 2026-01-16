#include <Wire.h>
#include <math.h>

static const int SDA_PIN = 22;   // 你确认可用：SDA=22
static const int SCL_PIN = 21;   // 你确认可用：SCL=21

static uint8_t MPU_ADDR = 0;

bool ping(uint8_t addr) {
  Wire.beginTransmission(addr);
  return (Wire.endTransmission(true) == 0);
}

bool writeReg(uint8_t addr, uint8_t reg, uint8_t val) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  Wire.write(val);
  return (Wire.endTransmission(true) == 0);
}

bool readBytes(uint8_t addr, uint8_t reg, uint8_t *buf, size_t n) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;

  Wire.requestFrom((int)addr, (int)n, true);
  for (size_t i = 0; i < n; i++) {
    if (!Wire.available()) return false;
    buf[i] = Wire.read();
  }
  return true;
}

uint8_t readReg(uint8_t addr, uint8_t reg) {
  uint8_t v = 0xFF;
  Wire.beginTransmission(addr);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return 0xFF;
  Wire.requestFrom((int)addr, 1, true);
  if (Wire.available()) v = Wire.read();
  return v;
}

void setup() {
  Serial.begin(115200);
  delay(1500);

  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(100000);    // 先用100k稳
  Wire.setTimeOut(50);      // 防止I2C扫描卡死

  Serial.println("serial_ok");
  Serial.printf("Using SDA=%d SCL=%d\n", SDA_PIN, SCL_PIN);

  bool ok68 = ping(0x68);
  bool ok69 = ping(0x69);
  Serial.printf("ping_0x68=%d ping_0x69=%d\n", ok68 ? 1 : 0, ok69 ? 1 : 0);

  if (ok68) MPU_ADDR = 0x68;
  else if (ok69) MPU_ADDR = 0x69;

  if (!MPU_ADDR) {
    Serial.println("No MPU found. Check wiring/VCC/GND/SDA/SCL/AD0.");
    return;
  }

  uint8_t who = readReg(MPU_ADDR, 0x75); // WHO_AM_I
  Serial.printf("MPU found at 0x%02X, WHO_AM_I=0x%02X\n", MPU_ADDR, who);

  // 唤醒：PWR_MGMT_1 = 0
  if (!writeReg(MPU_ADDR, 0x6B, 0x00)) {
    Serial.println("Write PWR_MGMT_1 failed");
  }

  // 可选配置（都不写也能读）
  // Gyro ±250 dps: GYRO_CONFIG(0x1B)=0x00  => 131 LSB/(deg/s)
  writeReg(MPU_ADDR, 0x1B, 0x00);

  // Accel ±2g: ACCEL_CONFIG(0x1C)=0x00    => 16384 LSB/g
  writeReg(MPU_ADDR, 0x1C, 0x00);

  // DLPF (可选)：CONFIG(0x1A)=0x03
  writeReg(MPU_ADDR, 0x1A, 0x03);

  Serial.println("t_ms,ax_g,ay_g,az_g,gx_dps,gy_dps,gz_dps,roll_deg,pitch_deg,temp_C");
}

void loop() {
  if (!MPU_ADDR) {
    delay(500);
    return;
  }

  uint8_t b[14];
  // 从 0x3B 开始一次读 14 字节：accel(6)+temp(2)+gyro(6)
  if (!readBytes(MPU_ADDR, 0x3B, b, 14)) {
    Serial.println("read_fail");
    delay(100);
    return;
  }

  int16_t ax = (b[0] << 8) | b[1];
  int16_t ay = (b[2] << 8) | b[3];
  int16_t az = (b[4] << 8) | b[5];
  int16_t t  = (b[6] << 8) | b[7];
  int16_t gx = (b[8] << 8) | b[9];
  int16_t gy = (b[10] << 8) | b[11];
  int16_t gz = (b[12] << 8) | b[13];

  // 转单位（按上面的±2g、±250dps配置）
  const float ACC_S = 16384.0f;  // LSB/g
  const float GYR_S = 131.0f;    // LSB/(deg/s)

  float ax_g = ax / ACC_S;
  float ay_g = ay / ACC_S;
  float az_g = az / ACC_S;

  float gx_dps = gx / GYR_S;
  float gy_dps = gy / GYR_S;
  float gz_dps = gz / GYR_S;

  float tempC = (t / 340.0f) + 36.53f;

  // 仅用加速度计估计 roll/pitch（静止/慢动时有意义）
  float roll  = atan2f(ay_g, az_g) * 57.2957795f;
  float pitch = atan2f(-ax_g, sqrtf(ay_g * ay_g + az_g * az_g)) * 57.2957795f;

  Serial.printf("%lu,%.5f,%.5f,%.5f,%.3f,%.3f,%.3f,%.2f,%.2f,%.2f\n",
                (unsigned long)millis(),
                ax_g, ay_g, az_g,
                gx_dps, gy_dps, gz_dps,
                roll, pitch, tempC);

  delay(20); // 50 Hz
}

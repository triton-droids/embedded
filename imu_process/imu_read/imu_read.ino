#include <Wire.h>
extern "C" {
  #include "driver/twai.h"
}

// ========= PIN MAP (avoid conflicts!) =========
// CAN pins (to transceiver)
#define CAN_TX_GPIO 5
#define CAN_RX_GPIO 4

// I2C pins (to MPU6050)  
#define I2C_SDA_PIN 10
#define I2C_SCL_PIN 9

// ========= CAN IDs =========
static const uint32_t CAN_ID_RSP1 = 0x101; // ax ay az gx
static const uint32_t CAN_ID_RSP2 = 0x102; // gy gz (and seq)

// ========= MPU6050 =========
static uint8_t MPU_ADDR = 0x68;

// Registers
static const uint8_t REG_WHO_AM_I   = 0x75;
static const uint8_t REG_PWR_MGMT_1 = 0x6B;
static const uint8_t REG_CONFIG     = 0x1A;
static const uint8_t REG_GYRO_CFG   = 0x1B;
static const uint8_t REG_ACCEL_CFG  = 0x1C;
static const uint8_t REG_DATA_START = 0x3B; // 14 bytes

// ========= helpers =========
static inline void put_be16(uint8_t *p, int16_t v) {
  p[0] = (uint8_t)((v >> 8) & 0xFF);
  p[1] = (uint8_t)(v & 0xFF);
}

bool i2c_ping(uint8_t addr) {
  Wire.beginTransmission(addr);
  return (Wire.endTransmission(true) == 0);
}

bool i2c_write(uint8_t addr, uint8_t reg, uint8_t val) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  Wire.write(val);
  return (Wire.endTransmission(true) == 0);
}

bool i2c_read_bytes(uint8_t addr, uint8_t reg, uint8_t *buf, size_t n) {
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

uint8_t i2c_read_u8(uint8_t addr, uint8_t reg) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return 0xFF;
  Wire.requestFrom((int)addr, 1, true);
  if (!Wire.available()) return 0xFF;
  return Wire.read();
}

bool mpu_init() {
  // Some MPU modules use 0x68, some 0x69
  bool ok68 = i2c_ping(0x68);
  bool ok69 = i2c_ping(0x69);
  if (ok68) MPU_ADDR = 0x68;
  else if (ok69) MPU_ADDR = 0x69;
  else return false;

  uint8_t who = i2c_read_u8(MPU_ADDR, REG_WHO_AM_I);
  Serial.printf("MPU at 0x%02X WHO_AM_I=0x%02X\n", MPU_ADDR, who);

  // Wake up
  if (!i2c_write(MPU_ADDR, REG_PWR_MGMT_1, 0x00)) return false;

  // ±250 dps
  i2c_write(MPU_ADDR, REG_GYRO_CFG, 0x00);
  // ±2g
  i2c_write(MPU_ADDR, REG_ACCEL_CFG, 0x00);
  // DLPF
  i2c_write(MPU_ADDR, REG_CONFIG, 0x03);

  return true;
}

bool can_init() {
  twai_general_config_t g =
    TWAI_GENERAL_CONFIG_DEFAULT((gpio_num_t)CAN_TX_GPIO,
                                (gpio_num_t)CAN_RX_GPIO,
                                TWAI_MODE_NORMAL);
  twai_timing_config_t t = TWAI_TIMING_CONFIG_500KBITS();
  twai_filter_config_t f = TWAI_FILTER_CONFIG_ACCEPT_ALL();

  int install_ret = (int)twai_driver_install(&g, &t, &f);
  int start_ret   = (int)twai_start();
  Serial.printf("TWAI install=%d start=%d\n", install_ret, start_ret);

  twai_clear_transmit_queue();
  twai_clear_receive_queue();
  return (install_ret == 0 && start_ret == 0);
}

bool can_send_std(uint32_t id, const uint8_t *data, uint8_t dlc) {
  twai_message_t m = {};
  m.extd = 0;
  m.rtr  = 0;
  m.identifier = id;
  m.data_length_code = dlc;
  for (int i = 0; i < dlc; i++) m.data[i] = data[i];
  return (twai_transmit(&m, pdMS_TO_TICKS(10)) == ESP_OK);
}

// ========= main =========
static uint8_t seq = 0;

void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println("\nNode B (IMU->CAN) boot...");

  // I2C
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  Wire.setClock(100000);
  Wire.setTimeOut(50);
  Serial.printf("I2C SDA=%d SCL=%d\n", I2C_SDA_PIN, I2C_SCL_PIN);

  // CAN
  Serial.printf("CAN TX=%d RX=%d\n", CAN_TX_GPIO, CAN_RX_GPIO);
  if (!can_init()) {
    Serial.println("CAN init FAILED (need transceiver + wiring)");
  }

  // MPU
  if (!mpu_init()) {
    Serial.println("No MPU found on I2C. Check VCC/GND/SDA/SCL/AD0 and pin conflicts.");
  } else {
    Serial.println("MPU init OK.");
  }
}

void loop() {
  // Read 14 bytes: ax ay az temp gx gy gz
  uint8_t b[14];
  if (!i2c_read_bytes(MPU_ADDR, REG_DATA_START, b, 14)) {
    // If this keeps happening, wiring/I2C pins still wrong
    Serial.println("MPU read_fail");
    delay(50);
    return;
  }

  int16_t ax = (int16_t)((b[0] << 8) | b[1]);
  int16_t ay = (int16_t)((b[2] << 8) | b[3]);
  int16_t az = (int16_t)((b[4] << 8) | b[5]);
  // int16_t t  = (int16_t)((b[6] << 8) | b[7]);
  int16_t gx = (int16_t)((b[8] << 8) | b[9]);
  int16_t gy = (int16_t)((b[10] << 8) | b[11]);
  int16_t gz = (int16_t)((b[12] << 8) | b[13]);

  seq++;

  // RSP1: ax ay az gx  (8 bytes)
  uint8_t d1[8] = {0};
  put_be16(&d1[0], ax);
  put_be16(&d1[2], ay);
  put_be16(&d1[4], az);
  put_be16(&d1[6], gx);

  // RSP2: gy gz ... seq (8 bytes)  —— seq 放最后一个字节，便于拼包
  uint8_t d2[8] = {0};
  put_be16(&d2[0], gy);
  put_be16(&d2[2], gz);
  d2[7] = seq;

  bool ok1 = can_send_std(CAN_ID_RSP1, d1, 8);
  bool ok2 = can_send_std(CAN_ID_RSP2, d2, 8);

  if (!ok1 || !ok2) {
    Serial.printf("CAN tx fail ok1=%d ok2=%d\n", ok1 ? 1 : 0, ok2 ? 1 : 0);
  }

  delay(20); // ~50Hz
}

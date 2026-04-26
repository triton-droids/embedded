#include <Wire.h>
#include <math.h>

#if defined(ARDUINO_ARCH_ESP32)
#include <driver/twai.h>
#else
#error "imu_read.ino CAN support requires an ESP32-class board with TWAI support."
#endif

static const int SDA_PIN = 22;
static const int SCL_PIN = 21;

// Adjust these to match your ESP32 <-> CAN transceiver wiring.
static const int CAN_TX_PIN = 5;
static const int CAN_RX_PIN = 4;
static const uint32_t CAN_BITRATE = 1000000;

// Request/reply protocol expected by utils/stream_imu_can.py and utils/imu_read.py.
static const uint16_t CAN_REQ_ID = 0x100;
static const uint16_t CAN_RSP1_ID = 0x101;
static const uint16_t CAN_RSP2_ID = 0x102;

static const uint32_t SAMPLE_PERIOD_MS = 5;  // ~200 Hz sensor refresh

static uint8_t MPU_ADDR = 0;
static uint8_t CAN_SEQ = 0;

struct ImuSample {
  int16_t ax;
  int16_t ay;
  int16_t az;
  int16_t temp_raw;
  int16_t gx;
  int16_t gy;
  int16_t gz;
  uint32_t t_ms;
};

static ImuSample LAST_SAMPLE = {0, 0, 0, 0, 0, 0, 0, 0};
static bool HAVE_SAMPLE = false;
static uint32_t LAST_SAMPLE_MS = 0;
static uint32_t LAST_READ_FAIL_MS = 0;

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

static inline void packI16BE(int16_t value, uint8_t *dst) {
  dst[0] = (uint8_t)((value >> 8) & 0xFF);
  dst[1] = (uint8_t)(value & 0xFF);
}

twai_timing_config_t makeCanTimingConfig(uint32_t bitrate) {
  switch (bitrate) {
    case 1000000:
      return TWAI_TIMING_CONFIG_1MBITS();
    case 500000:
      return TWAI_TIMING_CONFIG_500KBITS();
    case 250000:
      return TWAI_TIMING_CONFIG_250KBITS();
    case 125000:
      return TWAI_TIMING_CONFIG_125KBITS();
    default:
      Serial.printf("Unsupported CAN bitrate %lu, falling back to 1000000.\n", (unsigned long)bitrate);
      return TWAI_TIMING_CONFIG_1MBITS();
  }
}

bool initCan() {
  twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(
      (gpio_num_t)CAN_TX_PIN,
      (gpio_num_t)CAN_RX_PIN,
      TWAI_MODE_NORMAL);
  g_config.tx_queue_len = 8;
  g_config.rx_queue_len = 32;

  twai_timing_config_t t_config = makeCanTimingConfig(CAN_BITRATE);
  twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

  esp_err_t err = twai_driver_install(&g_config, &t_config, &f_config);
  if (err != ESP_OK) {
    Serial.printf("CAN driver install failed: %d\n", (int)err);
    return false;
  }

  err = twai_start();
  if (err != ESP_OK) {
    Serial.printf("CAN start failed: %d\n", (int)err);
    twai_driver_uninstall();
    return false;
  }

  Serial.printf(
      "CAN ready TX=%d RX=%d bitrate=%lu req=0x%03X rsp1=0x%03X rsp2=0x%03X\n",
      CAN_TX_PIN,
      CAN_RX_PIN,
      (unsigned long)CAN_BITRATE,
      CAN_REQ_ID,
      CAN_RSP1_ID,
      CAN_RSP2_ID);
  return true;
}

bool readImuSample(ImuSample &sample) {
  if (!MPU_ADDR) {
    return false;
  }

  uint8_t b[14];
  if (!readBytes(MPU_ADDR, 0x3B, b, 14)) {
    return false;
  }

  sample.ax = (int16_t)((b[0] << 8) | b[1]);
  sample.ay = (int16_t)((b[2] << 8) | b[3]);
  sample.az = (int16_t)((b[4] << 8) | b[5]);
  sample.temp_raw = (int16_t)((b[6] << 8) | b[7]);
  sample.gx = (int16_t)((b[8] << 8) | b[9]);
  sample.gy = (int16_t)((b[10] << 8) | b[11]);
  sample.gz = (int16_t)((b[12] << 8) | b[13]);
  sample.t_ms = millis();
  return true;
}

void printSampleSerial(const ImuSample &sample) {
  const float ACC_S = 16384.0f;
  const float GYR_S = 131.0f;

  float ax_g = sample.ax / ACC_S;
  float ay_g = sample.ay / ACC_S;
  float az_g = sample.az / ACC_S;

  float gx_dps = sample.gx / GYR_S;
  float gy_dps = sample.gy / GYR_S;
  float gz_dps = sample.gz / GYR_S;

  float tempC = (sample.temp_raw / 340.0f) + 36.53f;
  float roll = atan2f(ay_g, az_g) * 57.2957795f;
  float pitch = atan2f(-ax_g, sqrtf(ay_g * ay_g + az_g * az_g)) * 57.2957795f;

  Serial.printf("%lu,%.5f,%.5f,%.5f,%.3f,%.3f,%.3f,%.2f,%.2f,%.2f\n",
                (unsigned long)sample.t_ms,
                ax_g, ay_g, az_g,
                gx_dps, gy_dps, gz_dps,
                roll, pitch, tempC);
}

bool sendCanSample(const ImuSample &sample) {
  twai_message_t rsp1 = {};
  rsp1.identifier = CAN_RSP1_ID;
  rsp1.extd = 0;
  rsp1.rtr = 0;
  rsp1.data_length_code = 8;
  packI16BE(sample.ax, &rsp1.data[0]);
  packI16BE(sample.ay, &rsp1.data[2]);
  packI16BE(sample.az, &rsp1.data[4]);
  packI16BE(sample.gx, &rsp1.data[6]);

  twai_message_t rsp2 = {};
  rsp2.identifier = CAN_RSP2_ID;
  rsp2.extd = 0;
  rsp2.rtr = 0;
  rsp2.data_length_code = 8;
  packI16BE(sample.gy, &rsp2.data[0]);
  packI16BE(sample.gz, &rsp2.data[2]);
  rsp2.data[4] = CAN_SEQ++;
  rsp2.data[5] = 0;
  rsp2.data[6] = 0;
  rsp2.data[7] = 0;

  esp_err_t err1 = twai_transmit(&rsp1, pdMS_TO_TICKS(2));
  esp_err_t err2 = twai_transmit(&rsp2, pdMS_TO_TICKS(2));
  if (err1 != ESP_OK || err2 != ESP_OK) {
    Serial.printf("CAN transmit failed: rsp1=%d rsp2=%d\n", (int)err1, (int)err2);
    return false;
  }
  return true;
}

void handleCanRequests() {
  twai_message_t rx_msg;
  while (twai_receive(&rx_msg, 0) == ESP_OK) {
    if (rx_msg.extd) {
      continue;
    }
    if (rx_msg.identifier != CAN_REQ_ID) {
      continue;
    }
    if (!HAVE_SAMPLE) {
      continue;
    }
    sendCanSample(LAST_SAMPLE);
  }
}

void setup() {
  Serial.begin(115200);
  delay(1500);

  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(100000);
  Wire.setTimeOut(50);

  Serial.println("serial_ok");
  Serial.printf("Using SDA=%d SCL=%d\n", SDA_PIN, SCL_PIN);

  bool ok68 = ping(0x68);
  bool ok69 = ping(0x69);
  Serial.printf("ping_0x68=%d ping_0x69=%d\n", ok68 ? 1 : 0, ok69 ? 1 : 0);

  if (ok68) MPU_ADDR = 0x68;
  else if (ok69) MPU_ADDR = 0x69;

  if (!MPU_ADDR) {
    Serial.println("No MPU found. Check wiring/VCC/GND/SDA/SCL/AD0.");
  } else {
    uint8_t who = readReg(MPU_ADDR, 0x75);
    Serial.printf("MPU found at 0x%02X, WHO_AM_I=0x%02X\n", MPU_ADDR, who);

    if (!writeReg(MPU_ADDR, 0x6B, 0x00)) {
      Serial.println("Write PWR_MGMT_1 failed");
    }
    writeReg(MPU_ADDR, 0x1B, 0x00);
    writeReg(MPU_ADDR, 0x1C, 0x00);
    writeReg(MPU_ADDR, 0x1A, 0x03);
  }

  initCan();
  Serial.println("t_ms,ax_g,ay_g,az_g,gx_dps,gy_dps,gz_dps,roll_deg,pitch_deg,temp_C");
}

void loop() {
  handleCanRequests();

  uint32_t now = millis();
  if ((uint32_t)(now - LAST_SAMPLE_MS) < SAMPLE_PERIOD_MS) {
    delay(0);
    return;
  }
  LAST_SAMPLE_MS = now;

  ImuSample sample;
  if (!readImuSample(sample)) {
    if ((uint32_t)(now - LAST_READ_FAIL_MS) >= 100) {
      LAST_READ_FAIL_MS = now;
      Serial.println("read_fail");
    }
    delay(0);
    return;
  }

  LAST_SAMPLE = sample;
  HAVE_SAMPLE = true;
  printSampleSerial(sample);
}

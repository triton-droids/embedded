#include <Wire.h>
#include <MPU6050.h>

extern "C" {
  #include "driver/twai.h"
}

// ====== 你实际的接线（按你说的） ======
#define CAN_TX_GPIO 5
#define CAN_RX_GPIO 4

#define I2C_SDA 10
#define I2C_SCL 9

static const uint32_t CAN_ID_REQ = 0x100;
static const uint32_t CAN_ID_RSP = 0x101;

// 建议两节点通信用 NORMAL（NO_ACK 适合单节点自测，会“TX ok”但不代表总线真的通）
#define CAN_MODE TWAI_MODE_NORMAL

MPU6050 imu;

// ---- I2C 扫描：用来证明“到底有没有器件” ----
void i2cScan() {
  Serial.println("I2C scan...");
  int found = 0;
  for (uint8_t addr = 1; addr < 127; addr++) {
    Wire.beginTransmission(addr);
    if (Wire.endTransmission() == 0) {
      Serial.print("  found 0x");
      Serial.println(addr, HEX);
      found++;
    }
  }
  if (!found) Serial.println("  (no I2C devices found)");
  Serial.println("I2C scan done.");
}

bool imu_init() {
  Wire.begin(I2C_SDA, I2C_SCL);
  delay(50);
  i2cScan();

  imu.initialize();
  delay(10);

  // 如果你的模块 AD0=1，地址会变成 0x69，此时 testConnection 可能失败
  return imu.testConnection();
}

bool can_init() {
  twai_general_config_t g =
    TWAI_GENERAL_CONFIG_DEFAULT((gpio_num_t)CAN_TX_GPIO,
                                (gpio_num_t)CAN_RX_GPIO,
                                CAN_MODE);

  twai_timing_config_t t = TWAI_TIMING_CONFIG_500KBITS();
  twai_filter_config_t f = TWAI_FILTER_CONFIG_ACCEPT_ALL();

  Serial.printf("Installing TWAI... install=%d\n", (int)twai_driver_install(&g, &t, &f));
  Serial.printf("Starting TWAI...  start=%d\n", (int)twai_start());

  twai_clear_transmit_queue();
  twai_clear_receive_queue();

  return true;
}

static void send_imu_reply(int16_t ax, int16_t ay, int16_t gx, int16_t gy) {
  twai_message_t m = {};
  m.extd = 0;
  m.rtr  = 0;
  m.identifier = CAN_ID_RSP;
  m.data_length_code = 8;

  m.data[0] = (ax >> 8) & 0xFF;  m.data[1] = ax & 0xFF;
  m.data[2] = (ay >> 8) & 0xFF;  m.data[3] = ay & 0xFF;
  m.data[4] = (gx >> 8) & 0xFF;  m.data[5] = gx & 0xFF;
  m.data[6] = (gy >> 8) & 0xFF;  m.data[7] = gy & 0xFF;

  esp_err_t e = twai_transmit(&m, pdMS_TO_TICKS(50));
  if (e == ESP_OK) {
    Serial.printf("RSP TX ok | ax=%d ay=%d gx=%d gy=%d\n", ax, ay, gx, gy);
  } else {
    Serial.printf("RSP TX fail err=%d\n", (int)e);
  }
}

void setup() {
  Serial.begin(74800);
  delay(300);
  Serial.println("\nNode B (IMU Responder) boot...");

  bool ok_imu = imu_init();
  Serial.println(ok_imu ? "IMU OK" : "IMU FAILED (check SDA/SCL/VCC/GND, pins, address)");

  can_init();
  Serial.println("Node B ready.");
}

void loop() {
  twai_message_t r;
  if (twai_receive(&r, pdMS_TO_TICKS(1000)) != ESP_OK) return;

  if (!r.extd && r.identifier == CAN_ID_REQ) {
    Serial.print("REQ RX | DLC=");
    Serial.println(r.data_length_code);

    int16_t ax, ay, az, gx, gy, gz;
    imu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    send_imu_reply(ax, ay, gx, gy);
  }
}
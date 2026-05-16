#include <Wire.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

extern "C" {
  #include "driver/twai.h"
}

// ===================== WiFi AP + UDP =====================
const char* AP_SSID = "ESP32S3";
const char* AP_PASS = "12345678";

WiFiUDP udp;
IPAddress UDP_BROADCAST_IP(192, 168, 4, 255);
const int UDP_PORT = 5005;

// ===================== Pins =====================
#define CAN_TX_GPIO 5
#define CAN_RX_GPIO 4

#define I2C_SDA 10
#define I2C_SCL 9

// ===================== CAN IDs =====================
static const uint32_t CAN_ID_REQ   = 0x100;
static const uint32_t CAN_ID_ACC   = 0x101;  // ax ay az
static const uint32_t CAN_ID_GYRO  = 0x102;  // gx gy gz
static const uint32_t CAN_ID_MAG   = 0x103;  // mx my mz
static const uint32_t CAN_ID_EULER = 0x104;  // heading roll pitch seq

#define CAN_MODE TWAI_MODE_NORMAL
// #define CAN_MODE TWAI_MODE_NO_ACK

// ===================== BNO055 =====================
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);
bool imu_ok = false;

// ===================== Helpers =====================
static inline void put_be16(uint8_t *p, int16_t v) {
  p[0] = (uint8_t)((v >> 8) & 0xFF);
  p[1] = (uint8_t)(v & 0xFF);
}

int16_t scale100(float x) {
  return (int16_t)(x * 100.0f);
}

void wifi_ap_init() {
  WiFi.mode(WIFI_AP);
  bool ok = WiFi.softAP(AP_SSID, AP_PASS);

  if (ok) {
    Serial.println("WiFi AP started.");
    Serial.print("AP SSID: ");
    Serial.println(AP_SSID);
    Serial.print("AP PASS: ");
    Serial.println(AP_PASS);
    Serial.print("AP IP: ");
    Serial.println(WiFi.softAPIP());

    udp.begin(UDP_PORT);
    Serial.printf("UDP broadcast to 192.168.4.255:%d\n", UDP_PORT);
  } else {
    Serial.println("WiFi AP start FAILED.");
  }
}

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
  Wire.setClock(100000);
  Wire.setTimeOut(50);

  delay(50);
  i2cScan();

  if (!bno.begin()) {
    return false;
  }

  delay(1000);
  bno.setExtCrystalUse(true);
  return true;
}

bool can_init() {
  twai_general_config_t g =
    TWAI_GENERAL_CONFIG_DEFAULT(
      (gpio_num_t)CAN_TX_GPIO,
      (gpio_num_t)CAN_RX_GPIO,
      CAN_MODE
    );

  twai_timing_config_t t = TWAI_TIMING_CONFIG_500KBITS();
  twai_filter_config_t f = TWAI_FILTER_CONFIG_ACCEPT_ALL();

  int install_ret = (int)twai_driver_install(&g, &t, &f);
  int start_ret = (int)twai_start();

  Serial.printf("TWAI install=%d start=%d\n", install_ret, start_ret);

  twai_clear_transmit_queue();
  twai_clear_receive_queue();

  return install_ret == 0 && start_ret == 0;
}

bool send_frame(uint32_t id, const uint8_t *data, uint8_t dlc) {
  twai_message_t m = {};
  m.extd = 0;
  m.rtr = 0;
  m.identifier = id;
  m.data_length_code = dlc;

  for (int i = 0; i < dlc; i++) {
    m.data[i] = data[i];
  }

  return twai_transmit(&m, pdMS_TO_TICKS(50)) == ESP_OK;
}

void send_can3(uint32_t id, int16_t x, int16_t y, int16_t z, uint8_t seq) {
  uint8_t d[8] = {0};
  put_be16(&d[0], x);
  put_be16(&d[2], y);
  put_be16(&d[4], z);
  d[6] = seq;

  bool ok = send_frame(id, d, 8);
  Serial.printf("CAN 0x%03X TX %s\n", id, ok ? "ok" : "FAIL");
}

void send_udp_bno(uint8_t seq,
                  float ax, float ay, float az,
                  float gx, float gy, float gz,
                  float mx, float my, float mz,
                  float heading, float roll, float pitch) {
  char msg[256];

  snprintf(msg, sizeof(msg),
           "seq=%u,ax=%.3f,ay=%.3f,az=%.3f,gx=%.3f,gy=%.3f,gz=%.3f,mx=%.3f,my=%.3f,mz=%.3f,heading=%.3f,roll=%.3f,pitch=%.3f",
           seq, ax, ay, az, gx, gy, gz, mx, my, mz, heading, roll, pitch);

  udp.beginPacket(UDP_BROADCAST_IP, UDP_PORT);
  udp.print(msg);
  udp.endPacket();

  Serial.print("UDP sent: ");
  Serial.println(msg);
}

// ===================== Arduino =====================
static uint8_t seq = 0;

void setup() {
  Serial.begin(115200);
  delay(300);

  Serial.println("\nNode B (BNO055 Responder + AP + UDP) boot...");
  Serial.printf("I2C SDA=%d SCL=%d | CAN TX=%d RX=%d\n",
                I2C_SDA, I2C_SCL, CAN_TX_GPIO, CAN_RX_GPIO);

  wifi_ap_init();

  imu_ok = imu_init();
  Serial.println(imu_ok ? "BNO055 OK" : "BNO055 FAIL");

  bool can_ok = can_init();
  Serial.println(can_ok ? "CAN OK" : "CAN FAIL");

  Serial.println("Node B ready.");
}

void loop() {
  twai_message_t r;

  if (twai_receive(&r, pdMS_TO_TICKS(1000)) != ESP_OK) {
    return;
  }

  if (!r.extd && r.identifier == CAN_ID_REQ) {
    seq++;

    Serial.printf("REQ RX | dlc=%d | seq=%u\n",
                  (int)r.data_length_code,
                  seq);

    if (!imu_ok) {
      Serial.println("BNO055 not ready -> skip send");
      return;
    }

    imu::Vector<3> acc = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
    imu::Vector<3> gyro = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
    imu::Vector<3> mag = bno.getVector(Adafruit_BNO055::VECTOR_MAGNETOMETER);
    imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);

    float ax = acc.x();
    float ay = acc.y();
    float az = acc.z();

    float gx = gyro.x();
    float gy = gyro.y();
    float gz = gyro.z();

    float mx = mag.x();
    float my = mag.y();
    float mz = mag.z();

    float heading = euler.x();
    float roll = euler.y();
    float pitch = euler.z();

    Serial.printf("BNO055 | ax=%.2f ay=%.2f az=%.2f | gx=%.2f gy=%.2f gz=%.2f | mx=%.2f my=%.2f mz=%.2f | heading=%.2f roll=%.2f pitch=%.2f\n",
                  ax, ay, az, gx, gy, gz, mx, my, mz, heading, roll, pitch);

    send_udp_bno(seq, ax, ay, az, gx, gy, gz, mx, my, mz, heading, roll, pitch);

    send_can3(CAN_ID_ACC, scale100(ax), scale100(ay), scale100(az), seq);
    send_can3(CAN_ID_GYRO, scale100(gx), scale100(gy), scale100(gz), seq);
    send_can3(CAN_ID_MAG, scale100(mx), scale100(my), scale100(mz), seq);
    send_can3(CAN_ID_EULER, scale100(heading), scale100(roll), scale100(pitch), seq);
  }
}
// Streams BNO085 orientation, gyro and accel over USB serial at ~100 Hz for
// imu_safety_bench.py. One CSV line per orientation update:
//   t_ms,qw,qx,qy,qz,gx,gy,gz,ax,ay,az
// q = game rotation vector, g = calibrated gyro (rad/s),
// a = accelerometer incl. gravity (m/s^2). Units match sensor_msgs/Imu.
//
// Wiring (ESP32-S3): VIN->3V3, GND->GND, SDA->GPIO 8, SCL->GPIO 9.
// Library: Adafruit BNO08x. Serial Monitor/baud: 115200.

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_BNO08x.h>

static const int SDA_PIN = 8;
static const int SCL_PIN = 9;
static const uint8_t BNO_ADDR = 0x4A;  // Try 0x4B if "BNO085 not found".
static const uint32_t REPORT_US = 10000;  // 100 Hz, same as the safety node.

Adafruit_BNO08x bno;
sh2_SensorValue_t value;

float gx = 0, gy = 0, gz = 0;
float ax = 0, ay = 0, az = 0;

void enableReports() {
    bno.enableReport(SH2_GAME_ROTATION_VECTOR, REPORT_US);
    bno.enableReport(SH2_GYROSCOPE_CALIBRATED, REPORT_US);
    bno.enableReport(SH2_ACCELEROMETER, REPORT_US);
}

void setup() {
    Serial.begin(115200);
    delay(2000);
    Wire.begin(SDA_PIN, SCL_PIN);
    Wire.setClock(400000);
    delay(100);
    if (!bno.begin_I2C(BNO_ADDR, &Wire)) {
        while (true) {
            Serial.println("# BNO085 not found, check wiring/address");
            delay(1000);
        }
    }
    enableReports();
    Serial.println("# BNO085 ready: t_ms,qw,qx,qy,qz,gx,gy,gz,ax,ay,az");
}

void loop() {
    if (bno.wasReset()) {
        Serial.println("# BNO085 reset, re-enabling reports");
        enableReports();
    }
    if (!bno.getSensorEvent(&value)) {
        return;
    }
    switch (value.sensorId) {
        case SH2_GYROSCOPE_CALIBRATED:
            gx = value.un.gyroscope.x;
            gy = value.un.gyroscope.y;
            gz = value.un.gyroscope.z;
            break;
        case SH2_ACCELEROMETER:
            ax = value.un.accelerometer.x;
            ay = value.un.accelerometer.y;
            az = value.un.accelerometer.z;
            break;
        case SH2_GAME_ROTATION_VECTOR: {
            // 6 decimals so normal sensor noise is visible to the stuck check.
            const auto &q = value.un.gameRotationVector;
            Serial.printf("%lu,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f\n",
                          (unsigned long)millis(), q.real, q.i, q.j, q.k,
                          gx, gy, gz, ax, ay, az);
            break;
        }
    }
}


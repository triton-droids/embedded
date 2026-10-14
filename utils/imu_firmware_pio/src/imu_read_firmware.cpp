#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_BNO08x.h>

Adafruit_BNO08x bno;
sh2_SensorValue_t sensorValue;

void enableReports() {
    bno.enableReport(SH2_GAME_ROTATION_VECTOR, 200000);
    bno.enableReport(SH2_LINEAR_ACCELERATION, 200000);
    bno.enableReport(SH2_GYROSCOPE_CALIBRATED, 200000);
}

// Quaternion -> Euler angles in degrees (roll about X, pitch about Y, yaw about Z)
void quatToEuler(float qw, float qx, float qy, float qz,
                 float &roll, float &pitch, float &yaw) {
    float sinr = 2.0f * (qw * qx + qy * qz);
    float cosr = 1.0f - 2.0f * (qx * qx + qy * qy);
    roll = atan2f(sinr, cosr) * RAD_TO_DEG;

    float sinp = 2.0f * (qw * qy - qz * qx);
    sinp = constrain(sinp, -1.0f, 1.0f);  // avoid NaN near +-90 deg
    pitch = asinf(sinp) * RAD_TO_DEG;

    float siny = 2.0f * (qw * qz + qx * qy);
    float cosy = 1.0f - 2.0f * (qy * qy + qz * qz);
    yaw = atan2f(siny, cosy) * RAD_TO_DEG;
}

void setup() {
    Serial.begin(115200);
    delay(3000);
    Serial.println("Starting...");

    Wire.begin(8, 9);       // explicitly init Wire first
    Wire.setClock(400000);  // 400kHz I2C
    delay(3000);             // give I2C bus time to stabilize

    Serial.println("Wire initialized");
    
    while (!bno.begin_I2C(0x4A, &Wire)) {
      Wire.begin(8, 9);  
      Wire.setClock(400000);
      Serial.println("Trying again");
    }

    Serial.println("BNO085 found");
    delay(100);
    enableReports();
    Serial.println("BNO085 ready");
}

void loop() {
    if (bno.wasReset()) {
        Serial.println("BNO085 reset, re-enabling reports");
        enableReports();
    }

    if (!bno.getSensorEvent(&sensorValue)) {
        return;
    }

    switch (sensorValue.sensorId) {
        case SH2_GAME_ROTATION_VECTOR: {
            float qw = sensorValue.un.gameRotationVector.real;
            float qx = sensorValue.un.gameRotationVector.i;
            float qy = sensorValue.un.gameRotationVector.j;
            float qz = sensorValue.un.gameRotationVector.k;
            /*
            Serial.print("Q:");
            Serial.print(qw, 4); Serial.print(",");
            Serial.print(qx, 4); Serial.print(",");
            Serial.print(qy, 4); Serial.print(",");
            Serial.println(qz, 4);
            */
            float roll, pitch, yaw;
            quatToEuler(qw, qx, qy, qz, roll, pitch, yaw);
            Serial.print("E:");
            Serial.print(roll, 2);  Serial.print(",");
            Serial.print(pitch, 2); Serial.print(",");
            Serial.println(yaw, 2);
            break;
        }
        case SH2_LINEAR_ACCELERATION: {
            float ax = sensorValue.un.linearAcceleration.x;
            float ay = sensorValue.un.linearAcceleration.y;
            float az = sensorValue.un.linearAcceleration.z;
            Serial.print("A:");
            Serial.print(ax, 4); Serial.print(",");
            Serial.print(ay, 4); Serial.print(",");
            Serial.println(az, 4);
            break;
        }
        case SH2_GYROSCOPE_CALIBRATED: {
            float gx = sensorValue.un.gyroscope.x;
            float gy = sensorValue.un.gyroscope.y;
            float gz = sensorValue.un.gyroscope.z;
            Serial.print("G:");
            Serial.print(gx, 4); Serial.print(",");
            Serial.print(gy, 4); Serial.print(",");
            Serial.println(gz, 4);
            break;
        }
    }
}


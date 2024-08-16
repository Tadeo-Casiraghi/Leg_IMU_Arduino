#include "I2Cdev.h"
#include "MPU6050.h"

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for InvenSense evaluation board)
// AD0 high = 0x69
MPU6050 accelgyro1;
#define MPU1_AD0 10

MPU6050 accelgyro2;
#define MPU2_AD0 9

MPU6050 accelgyro3;
#define MPU3_AD0 8

#define  LED_PIN 2

int16_t ax, ay, az, gx, gy, gz;
int16_t ax_offset[] = {3736, 347, 3356};
int16_t ay_offset[] = {3679, 1736, -1592};
int16_t az_offset[] = {-1552, -1576, -841};
int16_t gx_offset[] = {64, 187, 549};
int16_t gy_offset[] = {29, 194, 800};
int16_t gz_offset[] = {-41, 10, 405};
int ad0s[] = {MPU1_AD0, MPU2_AD0, MPU3_AD0};
MPU6050* boards[] = {&accelgyro1, &accelgyro2, &accelgyro3};

bool start = false;

unsigned long t0, diference;

void setup() {
    // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, HIGH);
    delay(50);
    digitalWrite(LED_PIN, LOW);
    delay(50);
    digitalWrite(LED_PIN, HIGH);
    delay(50);
    digitalWrite(LED_PIN, LOW);

    Serial.begin(115200);

    pinMode(MPU1_AD0, OUTPUT);
    digitalWrite(MPU1_AD0, HIGH);

    pinMode(MPU2_AD0, OUTPUT);
    digitalWrite(MPU2_AD0, HIGH);

    pinMode(MPU3_AD0, OUTPUT);
    digitalWrite(MPU3_AD0, HIGH);

    delay(100);

    Serial.println("Initializing I2C devices...");

    //  SETUP FIRST MPU
    digitalWrite(MPU1_AD0, LOW);
    accelgyro1.initialize();
    Serial.println("Testing MPU1 connections...");
    Serial.println(accelgyro1.testConnection() ? "MPU1 connection successful" : "MPU1 connection failed");
    accelgyro1.setXAccelOffset(0);
    accelgyro1.setYAccelOffset(0);
    accelgyro1.setZAccelOffset(0);
    accelgyro1.setXGyroOffset(0);
    accelgyro1.setYGyroOffset(0);
    accelgyro1.setZGyroOffset(0);
    accelgyro1.setFullScaleAccelRange(3);
    digitalWrite(MPU1_AD0, HIGH);

    //  SETUP SECOND MPU
    digitalWrite(MPU2_AD0, LOW);
    accelgyro2.initialize();
    Serial.println("Testing MPU2 connections...");
    Serial.println(accelgyro2.testConnection() ? "MPU2 connection successful" : "MPU2 connection failed");
    accelgyro2.setXAccelOffset(0);
    accelgyro2.setYAccelOffset(0);
    accelgyro2.setZAccelOffset(0);
    accelgyro2.setXGyroOffset(0);
    accelgyro2.setYGyroOffset(0);
    accelgyro2.setZGyroOffset(0);
    accelgyro2.setFullScaleAccelRange(3);
    digitalWrite(MPU2_AD0, HIGH);

    //  SETUP THIRD MPU
    digitalWrite(MPU3_AD0, LOW);
    accelgyro3.initialize();
    Serial.println("Testing MPU3 connections...");
    Serial.println(accelgyro3.testConnection() ? "MPU3 connection successful" : "MPU3 connection failed");
    accelgyro3.setXAccelOffset(0);
    accelgyro3.setYAccelOffset(0);
    accelgyro3.setZAccelOffset(0);
    accelgyro3.setXGyroOffset(0);
    accelgyro3.setYGyroOffset(0);
    accelgyro3.setZGyroOffset(0);
    accelgyro3.setFullScaleAccelRange(3);
    digitalWrite(MPU3_AD0, HIGH);
    Serial.println("Ready to Start");


}

void loop() {
  if (Serial.available() && !start) {
    String dump = Serial.readString();
    start = true;
    t0 = micros();
    digitalWrite(LED_PIN, HIGH);
  }
  if (Serial.available() && start) {
    String dump = Serial.readString();
    start = false;
    digitalWrite(LED_PIN, LOW);
  }
  if (start) {
    for (uint8_t j = 0; j < 3; j++) {
      digitalWrite(ad0s[j], LOW);
      boards[j]->getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
      ax -= ax_offset[j];
      ay -= ay_offset[j];
      az -= az_offset[j];
      gx -= gx_offset[j];
      gy -= gy_offset[j];
      gz -= gz_offset[j];
      digitalWrite(ad0s[j], HIGH);
      Serial.write((uint8_t)(ax >> 8)); Serial.write((uint8_t)(ax & 0xFF));
      Serial.write((uint8_t)(ay >> 8)); Serial.write((uint8_t)(ay & 0xFF));
      Serial.write((uint8_t)(az >> 8)); Serial.write((uint8_t)(az & 0xFF));
      Serial.write((uint8_t)(gx >> 8)); Serial.write((uint8_t)(gx & 0xFF));
      Serial.write((uint8_t)(gy >> 8)); Serial.write((uint8_t)(gy & 0xFF));
      Serial.write((uint8_t)(gz >> 8)); Serial.write((uint8_t)(gz & 0xFF));
    }
    diference = micros() - t0;
    Serial.write((byte)(diference >> 24));
    Serial.write((byte)((diference >> 16)&0x000000FF));
    Serial.write((byte)((diference >> 8)&0x000000FF));
    Serial.write((byte)(diference & 0x000000FF));
  }
}

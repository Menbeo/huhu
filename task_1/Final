#include <Arduino.h>
#include <Wire.h>

#define I2C_SDA 23
#define I2C_SCL 22

const int MPU6050_ADDR = 0x68;
const int PWR_MGMT_1   = 0x6B;
const int ACCEL_XOUT_H = 0x3B;
const int TEMP_OUT_H   = 0x41;
const int GYRO_XOUT_H  = 0x43;

const float ACCEL_SENSITIVITY = 16384.0;
const float GYRO_SENSITIVITY = 131.0;

int16_t rawAccX, rawAccY, rawAccZ;
int16_t rawTemp;
int16_t rawGyroX, rawGyroY, rawGyroZ;

float accX, accY, accZ;
float gyroX, gyroY, gyroZ;
float temperature;
float angleX = 0.0, angleY = 0.0;
float accelAngleX = 0.0, accelAngleY = 0.0;
unsigned long prevTime = 0;
float dt = 0.0;

void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10);

  Serial.println("ESP32 MPU6050 Raw Register Read Test");

  if (!Wire.begin(I2C_SDA, I2C_SCL)) {
    Serial.println("Failed to initialize I2C communication.");
    while (1) delay(10);
  }

  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(PWR_MGMT_1);
  Wire.write(0x00);
  if (Wire.endTransmission(true) != 0) {
    Serial.println("MPU6050 Wake Up failed.");
    while (1) delay(10);
  }

  Serial.println("MPU6050 Initialized.");
  delay(100);
}

void loop() {
  unsigned long currTime = millis();
  dt = (currTime - prevTime) / 1000.0;
  prevTime = currTime;

  // Read accelerometer
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(ACCEL_XOUT_H);
  Wire.endTransmission(false);
  if (Wire.requestFrom(MPU6050_ADDR, 6, true) == 6) {
    rawAccX = (Wire.read() << 8) | Wire.read();
    rawAccY = (Wire.read() << 8) | Wire.read();
    rawAccZ = (Wire.read() << 8) | Wire.read();
  }

  // Read temperature
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(TEMP_OUT_H);
  Wire.endTransmission(false);
  if (Wire.requestFrom(MPU6050_ADDR, 2, true) == 2) {
    rawTemp = (Wire.read() << 8) | Wire.read();
  }

  // Read gyroscope
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(GYRO_XOUT_H);
  Wire.endTransmission(false);
  if (Wire.requestFrom(MPU6050_ADDR, 6, true) == 6) {
    rawGyroX = (Wire.read() << 8) | Wire.read();
    rawGyroY = (Wire.read() << 8) | Wire.read();
    rawGyroZ = (Wire.read() << 8) | Wire.read();
  }

  // Convert raw to real units
  accX = rawAccX / ACCEL_SENSITIVITY;
  accY = rawAccY / ACCEL_SENSITIVITY;
  accZ = rawAccZ / ACCEL_SENSITIVITY;

  gyroX = rawGyroX / GYRO_SENSITIVITY;
  gyroY = rawGyroY / GYRO_SENSITIVITY;
  gyroZ = rawGyroZ / GYRO_SENSITIVITY;

  temperature = (rawTemp / 340.0) + 36.53;

  // Complementary filter
  angleX += gyroX * dt;
  angleY += gyroY * dt;

  accelAngleX =  (atan(accY/ sqrt(pow(accX,2) + pow(accZ,2))) * 180/3.14159265359) - 0.58 ; 
  accelAngleY = (atan(-1 * accX/ sqrt(pow(accY ,2) + pow(accZ,2))) * 180/3.14159265359) + 1.58;
  //The gyroscope is kinda percise so we uses this S
  angleX = 0.95 * angleX + 0.05 * accelAngleX;
  angleY = 0.95 * angleY + 0.05 * accelAngleY;

  // Print Data- test imu
  // Serial.print("Acc (g): X="); Serial.print(accX, 3);
  // Serial.print(" | Y="); Serial.print(accY, 3);
  // Serial.print(" | Z="); Serial.println(accZ, 3);

  // Serial.print("Gyro (°/s): X="); Serial.print(gyroX, 3);
  // Serial.print(" | Y="); Serial.print(gyroY, 3);
  // Serial.print(" | Z="); Serial.println(gyroZ, 3);

  // Serial.print("Temp (*C): ");
  // Serial.println(temperature, 2);
  
  Serial.print(angleX, 2);
  Serial.print("\t");
  Serial.println(angleY, 2);

  // Serial.print("Tilt Angle Y (Pitch) (°): ");
  Serial.println(angleY, 2);


  // Serial.println("--------------------------------------");

  delay(500);
}

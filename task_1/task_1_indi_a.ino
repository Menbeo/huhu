#include <Arduino.h> // Include the main Arduino header
#include <Wire.h>

// Define the I2C pins specifically for ESP32
#define I2C_SDA 23
#define I2C_SCL 22

// MPU6050 I2C Address (assuming AD0 is LOW)
const int MPU6050_ADDR = 0x68;

// MPU6050 Register Addresses
const int PWR_MGMT_1   = 0x6B; // Power Management 1
const int ACCEL_XOUT_H = 0x3B; // Accelerometer measurements (High byte first)
const int ACCEL_YOUT_H = 0x3D;
const int ACCEL_ZOUT_H = 0x3F;
const int TEMP_OUT_H   = 0x41; // Temperature measurement
const int GYRO_XOUT_H  = 0x43; // Gyroscope measurements (High byte first)
const int GYRO_YOUT_H  = 0x45;
const int GYRO_ZOUT_H  = 0x47;

// Variables to store raw sensor data (signed 16-bit)
int16_t rawAccX, rawAccY, rawAccZ;
int16_t rawTemp;
int16_t rawGyroX, rawGyroY, rawGyroZ;

// Variables for scaled sensor data
float accX, accY, accZ;
float gyroX, gyroY, gyroZ;
float temperature;

// Sensitivity Scale Factor
const float ACCEL_SENSITIVITY = 16384.0;
const float GYRO_SENSITIVITY = 131.0;

// Variable to store the calculated angle
float angleY; // Tilt around y-axis (pitch)

void setup() {
  Serial.begin(115200);
  while (!Serial) {
    delay(10);
  }
  Serial.println("ESP32 MPU6050 Raw Register Read Test");

  Serial.println("Initializing I2C...");
  if (!Wire.begin(I2C_SDA, I2C_SCL)) {
    Serial.println("Failed to initialize I2C communication.");
    while (1) delay(10);
  }
  Serial.println("I2C Initialized.");

  Serial.println("Waking up MPU6050...");
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(PWR_MGMT_1);
  Wire.write(0x00);
  if (Wire.endTransmission(true) == 0) {
    Serial.println("MPU6050 Wake Up successful.");
  } else {
    Serial.println("MPU6050 Wake Up failed. Check connections & address.");
    while (1) delay(10);
  }

  Serial.println("\nStarting sensor readings...");
  delay(100);
}

void loop() {
  // --- Read Accelerometer Data ---
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(ACCEL_XOUT_H);
  Wire.endTransmission(false);
  if (Wire.requestFrom(MPU6050_ADDR, 6, true) == 6) {
    rawAccX = (Wire.read() << 8) | Wire.read();
    rawAccY = (Wire.read() << 8) | Wire.read();
    rawAccZ = (Wire.read() << 8) | Wire.read();
  } else {
    Serial.println("Failed to read Accel data");
    rawAccX = rawAccY = rawAccZ = 0;
  }

  // --- Read Temperature Data ---
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(TEMP_OUT_H);
  Wire.endTransmission(false);
  if (Wire.requestFrom(MPU6050_ADDR, 2, true) == 2) {
    rawTemp = (Wire.read() << 8) | Wire.read();
  } else {
    Serial.println("Failed to read Temp data");
    rawTemp = 0;
  }

  // --- Read Gyroscope Data ---
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(GYRO_XOUT_H);
  Wire.endTransmission(false);
  if (Wire.requestFrom(MPU6050_ADDR, 6, true) == 6) {
    rawGyroX = (Wire.read() << 8) | Wire.read();
    rawGyroY = (Wire.read() << 8) | Wire.read();
    rawGyroZ = (Wire.read() << 8) | Wire.read();
  } else {
    Serial.println("Failed to read Gyro data");
    rawGyroX = rawGyroY = rawGyroZ = 0;
  }

  // --- Convert Raw Data to Meaningful Units ---
  accX = rawAccX / ACCEL_SENSITIVITY;
  accY = rawAccY / ACCEL_SENSITIVITY;
  accZ = rawAccZ / ACCEL_SENSITIVITY;

  gyroX = rawGyroX / GYRO_SENSITIVITY;
  gyroY = rawGyroY / GYRO_SENSITIVITY;
  gyroZ = rawGyroZ / GYRO_SENSITIVITY;

  temperature = (rawTemp / 340.0) + 36.53;
  //Combine angle 
  // accelAngleX = atan2(accY, accZ) * 180/M_PI;
  // accelAngleY = atan2(-accX, sqrt(accY*accY + accZ*accZ)) * 180/3.14159265359;

  // --- Calculate Tilt Angle around Y axis (Pitch) ---
  angleY = (atan(-1 * accX/ sqrt(pow(accY,2) + pow(accZ, 2))) * 180/3.14159265359) + 1.58; 

  // --- Print Scaled Data ---
  // Serial.print("Acc (g): ");
  // Serial.print("X="); Serial.print(accX, 3);
  // Serial.print(" | Y="); Serial.print(accY, 3);
  // Serial.print(" | Z="); Serial.println(accZ, 3);

  // Serial.print("Gyro (°/s): ");
  // Serial.print("X="); Serial.print(gyroX, 3);
  // Serial.print(" | Y="); Serial.print(gyroY, 3);
  // Serial.print(" | Z="); Serial.println(gyroZ, 3);

  // Serial.print("Temp (*C): ");
  // Serial.println(temperature, 2);

  // --- Print Tilt Angle ---
  // Serial.print("Tilt Angle Y (Pitch) (°): ");
  Serial.println(angleY, 2);

  // Serial.println("--------------------------------------");

  delay(500);
}

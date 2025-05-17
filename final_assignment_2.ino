

#include <Arduino.h> 

#include <Wire.h> 

#include <BLEDevice.h> 

#include <BLEServer.h> 

#include <BLEUtils.h> 

#include <BLE2902.h> 

#include <String.h> 

 

// Define I2C pins 

#define I2C_SDA 23 

#define I2C_SCL 22 

 

// Motor pins for L298N 

#define ENA 16 

#define IN1 17 

#define IN2 5 

#define IN3 19 

#define IN4 18 

#define ENB 21 

 

// MPU6050 settings 

const int MPU6050_ADDR = 0x68; 

const int PWR_MGMT_1 = 0x6B; 

const int ACCEL_XOUT_H = 0x3B; 

const int ACCEL_YOUT_H = 0x3D; 

const int ACCEL_ZOUT_H = 0x3F; 

const int GYRO_XOUT_H = 0x43; 

const int GYRO_YOUT_H = 0x45; 

const int GYRO_ZOUT_H = 0x47; 

 

// Sensor data 

int16_t rawAccX, rawAccY, rawAccZ; 

int16_t rawGyroX, rawGyroY, rawGyroZ; 

const float ACCEL_SENSITIVITY = 16384.0; 

const float GYRO_SENSITIVITY = 131.0; 

float accX, accY, accZ; 

float gyroX, gyroY, gyroZ; 

float angle; 

 

// Complementary filter 

const float ALPHA = 0.98; 

float lastTime; 

float dt; 

 

// PID variables 

float KP = 60; 

float KI = 100; 

float KD = 2.2; 

float setpoint = -7; 

float defaultSetpoint = 0; 

float error, lastError; 

float integral, derivative; 

float pidOutput; 

 

// Motor speed constraints 

const int MAX_PWM = 160; 

const int MIN_PWM = 50; 

 

// Mode control 

int mode = 0; 

float turnFactor = 0.0; 

 

// BLE Configuration 

#define SERVICE_UUID "4fafc201-1fb5-459e-8fcc-c5c9c331914b" 

#define CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8" 

BLEServer* pServer = NULL; 

BLECharacteristic* pCharacteristic = NULL; 

bool deviceConnected = false; 

 

class MyServerCallbacks : public BLEServerCallbacks { 

void onConnect(BLEServer* pServer) { 

deviceConnected = true; 

Serial.println("Thiết bị đã kết nối"); 

} 

void onDisconnect(BLEServer* pServer) { 

deviceConnected = false; 

Serial.println("Thiết bị ngắt kết nối"); 

BLEDevice::startAdvertising(); 

} 

 

}; 

 

class MyCharacteristicCallbacks : public BLECharacteristicCallbacks { 

void onWrite(BLECharacteristic *pCharacteristic) { 

String value = pCharacteristic->getValue().c_str(); 

if (value.length() == 0) return; 

Serial.print("Nhận dữ liệu: "); Serial.println(value); 

if (value.startsWith("PID:")) { 

String pidData = value.substring(4); 

int firstComma = pidData.indexOf(','); 

int secondComma = pidData.indexOf(',', firstComma + 1); 

int thirdComma = pidData.indexOf(',', secondComma + 1); 

if (firstComma != -1 && secondComma != -1 && thirdComma != -1) { 

String kpStr = pidData.substring(0, firstComma); 

String kiStr = pidData.substring(firstComma + 1, secondComma); 

String kdStr = pidData.substring(secondComma + 1, thirdComma); 

String spStr = pidData.substring(thirdComma + 1); 

if (kpStr.length() > 0 && kiStr.length() > 0 && kdStr.length() > 0 && spStr.length() > 0) { 

float newKP = kpStr.toFloat(); 

float newKI = kiStr.toFloat(); 

float newKD = kdStr.toFloat(); 

float newSetpoint = spStr.toFloat(); 

KP = constrain(newKP, 0.0, 3000.0); 

KI = constrain(newKI, 0.0, 3000.0); 

KD = constrain(newKD, 0.0, 3000.0); 

setpoint = constrain(newSetpoint, -10.0, 10.0); 

error = 0; 

lastError = 0; 

integral = 0; 

Serial.print("PID mới - KP: "); Serial.print(KP); 

Serial.print(", KI: "); Serial.print(KI); 

Serial.print(", KD: "); Serial.print(KD); 

Serial.print(", Setpoint: "); Serial.println(setpoint); 

} 

} 

} else if (value.startsWith("MODE:")) { 

int newMode = value.substring(5).toInt(); 

if (newMode == 0 || newMode == 1) { 

mode = newMode; 

setpoint = defaultSetpoint; 

turnFactor = 0.0; 

error = 0; 

lastError = 0; 

integral = 0; 

Serial.print("Chuyển chế độ: "); Serial.println(mode == 0 ? "Tự cân bằng" : "Điều khiển"); 

} 

} else if (value.startsWith("CMD:")) { 

String cmd = value.substring(4); 

if (mode == 1) { 

if (cmd == "FWD") { 

setpoint = defaultSetpoint + 0.8; 

turnFactor = 0.0; 

Serial.println("Lệnh: Tiến"); 

} else if (cmd == "BWD") { 

setpoint = defaultSetpoint - 0.8; 

turnFactor = 0.0; 

Serial.println("Lệnh: Lùi"); 

} else if (cmd == "LEFT") { 

setpoint = defaultSetpoint; 

turnFactor = -0.5; 

Serial.println("Lệnh: Rẽ trái"); 

} else if (cmd == "RIGHT") { 

setpoint = defaultSetpoint; 

turnFactor = 0.5; 

Serial.println("Lệnh: Rẽ phải"); 

} else if (cmd == "STOP") { 

setpoint = defaultSetpoint; 

turnFactor = 0.0; 

Serial.println("Lệnh: Dừng"); 

} 

} 

} 

} 

}; 

 

void setup() { 

Serial.begin(115200); 

pinMode(ENA, OUTPUT); 

pinMode(IN1, OUTPUT); 

pinMode(IN2, OUTPUT); 

pinMode(IN3, OUTPUT); 

pinMode(IN4, OUTPUT); 

pinMode(ENB, OUTPUT); 

stopMotors(); 

Wire.begin(I2C_SDA, I2C_SCL); 

Wire.beginTransmission(MPU6050_ADDR); 

Wire.write(PWR_MGMT_1); 

Wire.write(0x00); 

Wire.endTransmission(true); 

BLEDevice::init("1000diem"); 

pServer = BLEDevice::createServer(); 

pServer->setCallbacks(new MyServerCallbacks()); 

BLEService *pService = pServer->createService(SERVICE_UUID); 

pCharacteristic = pService->createCharacteristic( 

CHARACTERISTIC_UUID, 

BLECharacteristic::PROPERTY_READ | 

BLECharacteristic::PROPERTY_WRITE | 

BLECharacteristic::PROPERTY_NOTIFY | 

BLECharacteristic::PROPERTY_INDICATE 

); 

pCharacteristic->addDescriptor(new BLE2902()); 

pCharacteristic->setCallbacks(new MyCharacteristicCallbacks()); 

pService->start(); 

BLEAdvertising *pAdvertising = BLEDevice::getAdvertising(); 

pAdvertising->addServiceUUID(SERVICE_UUID); 

pAdvertising->setScanResponse(true); 

pAdvertising->setMinPreferred(0x06); 

BLEDevice::startAdvertising(); 

Serial.println("Bắt đầu quảng bá BLE"); 

lastTime = micros() / 1000000.0; 

} 

 

void controlMotor1(int speed, bool forward) { 

if (speed == 0) { 

digitalWrite(IN1, LOW); 

digitalWrite(IN2, LOW); 

analogWrite(ENA, 0); 

return; 

} 

digitalWrite(IN1, forward ? HIGH : LOW); 

digitalWrite(IN2, forward ? LOW : HIGH); 

analogWrite(ENA, constrain(speed, 0, MAX_PWM)); 

} 

 

void controlMotor2(int speed, bool forward) { 

if (speed == 0) { 

digitalWrite(IN3, LOW); 

digitalWrite(IN4, LOW); 

analogWrite(ENB, 0); 

return; 

} 

digitalWrite(IN3, forward ? HIGH : LOW); 

digitalWrite(IN4, forward ? LOW : HIGH); 

analogWrite(ENB, constrain(speed, 0, MAX_PWM)); 

} 

 

void stopMotors() { 

digitalWrite(IN1, LOW); 

digitalWrite(IN2, LOW); 

digitalWrite(IN3, LOW); 

digitalWrite(IN4, LOW); 

analogWrite(ENA, 0); 

analogWrite(ENB, 0); 

} 

 

bool readMPU6050() { 

int retries = 3; 

while (retries > 0) { 

Wire.beginTransmission(MPU6050_ADDR); 

Wire.write(ACCEL_XOUT_H); 

if (Wire.endTransmission(false) == 0 && Wire.requestFrom(MPU6050_ADDR, 6, true) == 6) { 

rawAccX = (Wire.read() << 8) | Wire.read(); 

rawAccY = (Wire.read() << 8) | Wire.read(); 

rawAccZ = (Wire.read() << 8) | Wire.read(); 

Wire.beginTransmission(MPU6050_ADDR); 

Wire.write(GYRO_XOUT_H); 

if (Wire.endTransmission(false) == 0 && Wire.requestFrom(MPU6050_ADDR, 6, true) == 6) { 

rawGyroX = (Wire.read() << 8) | Wire.read(); 

rawGyroY = (Wire.read() << 8) | Wire.read(); 

rawGyroZ = (Wire.read() << 8) | Wire.read(); 

accX = rawAccX / ACCEL_SENSITIVITY; 

accY = rawAccY / ACCEL_SENSITIVITY; 

accZ = rawAccZ / ACCEL_SENSITIVITY; 

gyroX = rawGyroX / GYRO_SENSITIVITY; 

gyroY = rawGyroY / GYRO_SENSITIVITY; 

gyroZ = rawGyroZ / GYRO_SENSITIVITY; 

return true; 

} 

} 

retries--; 

delay(10); 

} 

Serial.println("Lỗi đọc MPU6050 sau nhiều lần thử"); 

return false; 

} 

 

void calculateAngle() { 

float currentTime = micros() / 1000000.0; 

dt = currentTime - lastTime; 

lastTime = currentTime; 

float accAngle = atan2(accY, accZ) * 180.0 / PI; 

static float gyroAngle = 0.0; 

gyroAngle += gyroX * dt; 

angle = ALPHA * (angle + gyroX * dt) + (1.0 - ALPHA) * accAngle; 

} 

 

void loop() { 

if (!deviceConnected && mode == 1) { 

Serial.println("BLE ngắt kết nối, chuyển về tự cân bằng"); 

mode = 0; 

setpoint = defaultSetpoint; 

turnFactor = 0.0; 

error = 0; 

lastError = 0; 

integral = 0; 

} 

if (!readMPU6050()) { 

stopMotors(); 

Serial.println("Lỗi đọc MPU6050, thử lại..."); 

delay(100); 

return; 

} 

 

calculateAngle(); 

if (abs(angle) > 45) { 

stopMotors(); 

Serial.println("Góc nghiêng quá lớn, dừng động cơ"); 

delay(100); 

return; 

} 

error = -setpoint + angle; 

integral += error * dt; 

integral = constrain(integral, -1000, 1000); 

derivative = (error - lastError) / dt; 

pidOutput = KP * error + KI * integral + KD * derivative; 

pidOutput = constrain(pidOutput, -MAX_PWM, MAX_PWM); 

lastError = error; 

int baseMotorSpeed = abs(pidOutput); 

baseMotorSpeed = constrain(baseMotorSpeed, 0, MAX_PWM); 

if (baseMotorSpeed < MIN_PWM && baseMotorSpeed > 0) { 

baseMotorSpeed = MIN_PWM; 

} 

int motor1Speed = baseMotorSpeed; 

int motor2Speed = baseMotorSpeed; 

if (mode == 1 && turnFactor != 0.0) { 

motor1Speed = baseMotorSpeed * (1.0 - turnFactor); 

motor2Speed = baseMotorSpeed * (1.0 + turnFactor); 

motor1Speed = constrain(motor1Speed, 0, MAX_PWM); 

motor2Speed = constrain(motor2Speed, 0, MAX_PWM); 

} 

bool forward = pidOutput >= 0; 

controlMotor1(motor1Speed, forward); 

controlMotor2(motor2Speed, forward); 

if (deviceConnected) { 

String data = 

String(accX, 2) + "," + 

String(accY, 2) + "," + 

String(accZ, 2) + "," + 

String(gyroX, 2) + "," + 

String(gyroY, 2) + "," + 

String(gyroZ, 2) + "," + 

String(angle, 2) + "," + 

String(baseMotorSpeed) + "," + 

String(mode); 

pCharacteristic->setValue(data.c_str()); 

pCharacteristic->notify(); 

} 

Serial.print("Chế độ: "); Serial.print(mode == 0 ? "Tự cân bằng" : "Điều khiển"); 

Serial.print(" | Góc: "); Serial.print(angle, 2); 

Serial.print(" | Sai số: "); Serial.print(error, 2); 

Serial.print(" | PID: "); Serial.print(pidOutput, 2); 

Serial.print(" | Tốc độ 1: "); Serial.print(motor1Speed); 

Serial.print(" | Tốc độ 2: "); Serial.print(motor2Speed); 

Serial.print(" | Hướng: "); Serial.println(forward ? "Tiến" : "Lùi"); 

delay(20); 

} 

 

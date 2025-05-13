#include <Arduino.h>
#include <Wire.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>

// BLE UUIDs (must match your web interface)
#define SERVICE_UUID        "19b10000-e8f2-537e-4f6c-d104768a1214"
#define SENSOR_CHARACTERISTIC_UUID "19b10001-e8f2-537e-4f6c-d104768a1214"
#define LED_CHARACTERISTIC_UUID "19b10002-e8f2-537e-4f6c-d104768a1214"

// Pin setup for Motors
const int IN1 = 17, IN2 = 5, ENA = 16;    // Motor A
const int IN3 = 19, IN4 = 18, ENB = 21;   // Motor B

// I2C Pins for ESP32
#define I2C_SDA 23
#define I2C_SCL 22

// MPU6050 I2C Address (assuming AD0 is LOW)
const int MPU6050_ADDR = 0x68;

// MPU6050 Register Addresses
const int PWR_MGMT_1   = 0x6B;
const int ACCEL_XOUT_H = 0x3B;
const int ACCEL_YOUT_H = 0x3D;
const int ACCEL_ZOUT_H = 0x3F;
const int GYRO_XOUT_H  = 0x43;
const int GYRO_YOUT_H  = 0x45;
const int GYRO_ZOUT_H  = 0x47;

// Variables to store raw sensor data
int16_t rawAccX, rawAccY, rawAccZ;
int16_t rawGyroX, rawGyroY, rawGyroZ;

// Processed sensor data
float accX, accY, accZ;
float gyroX, gyroY, gyroZ;

// PID control variables
float KP = 60;     
float KI = 270;     
float KD = 0;     
float setpoint = 3.8;
float error, lastError;
float integral, derivative;
float pidOutput;

// Angle calculation
float angleY = 0;
float gyroAngleY = 0;
const float ALPHA = 0.98;

// Sensitivity Scale Factor
const float ACCEL_SENSITIVITY = 16384.0;
const float GYRO_SENSITIVITY = 131.0;

// Timing variables
unsigned long lastUpdate = 0;
const long dt_us = 5000;

// Motor constraints
const int MAX_PWM = 255;
const int MIN_PWM = 50;

// BLE Variables
BLEServer *pServer;
BLEService *pService;
BLECharacteristic *pTxCharacteristic;
BLECharacteristic *pRxCharacteristic;
bool deviceConnected = false;
bool oldDeviceConnected = false;
bool manualMode = false;
int manualSpeed = 0;

void initMotors() {
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENB, OUTPUT);
}

void stopMotors() {
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
  digitalWrite(IN1, LOW); digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW); digitalWrite(IN4, LOW);
}

void setMotors(int pwmVal) {
  if (manualMode) {
    pwmVal = manualSpeed;
  }
  
  pwmVal = constrain(pwmVal, -MAX_PWM, MAX_PWM);
  
  if (pwmVal != 0 && abs(pwmVal) < MIN_PWM) {
    pwmVal = (pwmVal > 0) ? MIN_PWM : -MIN_PWM;
  }
  
  if(pwmVal < 0) {    
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
    pwmVal = abs(pwmVal);
  } else {    
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
  }
  
  analogWrite(ENA, pwmVal);
  analogWrite(ENB, pwmVal);
}

void calculateAngle(float dt) {
  // Calculate angle from accelerometer
  float accAngleY = (atan(-1 * accX / sqrt(pow(accY, 2) + pow(accZ, 2)))) * 180.0 / PI;
  
  // Integrate gyroscope data
  gyroAngleY += gyroY * dt;
  
  // Complementary filter
  angleY = ALPHA * gyroAngleY + (1.0 - ALPHA) * accAngleY;
}

class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
      deviceConnected = true;
    };

    void onDisconnect(BLEServer* pServer) {
      deviceConnected = false;
    }
};

class MyCallbacks: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pCharacteristic) {
      std::string rxValue = pCharacteristic->getValue();
      if (rxValue.length() > 0) {
        String command = String(rxValue.c_str());
        processCommand(command);
      }
    }
};

void processCommand(String command) {
  Serial.println("Received command: " + command);

  // Check for PID command
  if (command.startsWith("PID:")) {
    command.remove(0, 4); // Remove "PID:"
    int comma1 = command.indexOf(',');
    int comma2 = command.indexOf(',', comma1 + 1);
    int comma3 = command.indexOf(',', comma2 + 1);
    
    if (comma1 != -1 && comma2 != -1 && comma3 != -1) {
      KP = command.substring(0, comma1).toFloat();
      KI = command.substring(comma1 + 1, comma2).toFloat();
      KD = command.substring(comma2 + 1, comma3).toFloat();
      setpoint = command.substring(comma3 + 1).toFloat();
      
      Serial.printf("PID Updated: Kp=%.2f, Ki=%.2f, Kd=%.2f, Setpoint=%.2f\n", KP, KI, KD, setpoint);
    }
  }
  // Handle manual control commands
  else if (command == "forward") {
    manualMode = true;
    manualSpeed = 150;
  }
  else if (command == "backward") {
    manualMode = true;
    manualSpeed = -150;
  }
  else if (command == "stop") {
    manualMode = false;
    manualSpeed = 0;
  }
  else if (command == "left") {
    // Implement left turn logic
  }
  else if (command == "right") {
    // Implement right turn logic
  }
  // Handle direct speed commands (from slider)
  else if (command.toInt() != 0 || command == "0") {
    manualMode = true;
    manualSpeed = command.toInt();
  }
}

void setupBLE() {
  BLEDevice::init("Huhu");
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  pService = pServer->createService(SERVICE_UUID);

  // Create a BLE Characteristic for sending data
  pTxCharacteristic = pService->createCharacteristic(
                      CHARACTERISTIC_UUID_TX,
                      BLECharacteristic::PROPERTY_NOTIFY
                    );
  pTxCharacteristic->addDescriptor(new BLE2902());

  // Create a BLE Characteristic for receiving data
  pRxCharacteristic = pService->createCharacteristic(
                      CHARACTERISTIC_UUID_RX,
                      BLECharacteristic::PROPERTY_WRITE
                    );
  pRxCharacteristic->setCallbacks(new MyCallbacks());

  pService->start();
  pServer->getAdvertising()->start();
  Serial.println("Waiting for a client connection to notify...");
}

void setup() {
  Serial.begin(115200);
  while (!Serial) {
    delay(10);
  }
  
  Wire.begin(I2C_SDA, I2C_SCL);
  
  // Wake up MPU6050
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(PWR_MGMT_1);
  Wire.write(0x00);
  Wire.endTransmission(true);
  
  initMotors();
  lastUpdate = micros();
  
  setupBLE();
}

void loop() {
  unsigned long now = micros();
  if (now - lastUpdate < dt_us) return;
  float dt = (now - lastUpdate) / 1000000.0;
  lastUpdate = now;

  // Read Accelerometer Data
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(ACCEL_XOUT_H);
  Wire.endTransmission(false);
  if (Wire.requestFrom(MPU6050_ADDR, 6, true) == 6) {
    rawAccX = (Wire.read() << 8) | Wire.read();
    rawAccY = (Wire.read() << 8) | Wire.read();
    rawAccZ = (Wire.read() << 8) | Wire.read();
  }

  // Read Gyroscope Data
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(GYRO_YOUT_H);
  Wire.endTransmission(false);
  if (Wire.requestFrom(MPU6050_ADDR, 6, true) == 6) {
    rawGyroX = (Wire.read() << 8) | Wire.read();
    rawGyroY = (Wire.read() << 8) | Wire.read();
    rawGyroZ = (Wire.read() << 8) | Wire.read();
  }

  // Convert to proper units
  accX = rawAccX / ACCEL_SENSITIVITY;
  accY = rawAccY / ACCEL_SENSITIVITY;
  accZ = rawAccZ / ACCEL_SENSITIVITY;
  gyroY = rawGyroY / GYRO_SENSITIVITY;

  // Calculate angle
  calculateAngle(dt);

  // Safety stop
  if (fabs(angleY) > 30.0) {
    stopMotors();
    integral = 0;
    delay(1000);
    return;
  }

  // Handle BLE connection status
  if (!deviceConnected && oldDeviceConnected) {
    delay(500); // give the bluetooth stack the chance to get things ready
    pServer->startAdvertising(); // restart advertising
    Serial.println("start advertising");
    oldDeviceConnected = deviceConnected;
  }
  if (deviceConnected && !oldDeviceConnected) {
    oldDeviceConnected = deviceConnected;
  }

  // Only run PID if not in manual mode
  if (!manualMode) {
    // PID calculation
    error = angleY - setpoint;
    integral += error * dt;
    derivative = (error - lastError) / dt;
    lastError = error;
    
    integral = constrain(integral, -50, 255);
    
    pidOutput = KP * error + KI * integral + KD * derivative;
    
    // Motor control
    setMotors(pidOutput);
  }

  // Send data to BLE client
  if (deviceConnected) {
    char txString[50];
    snprintf(txString, sizeof(txString), "Angle:%.2f,PID:%.2f,%.2f,%.2f,Set:%.2f", 
             angleY, KP, KI, KD, setpoint);
    pTxCharacteristic->setValue(txString);
    pTxCharacteristic->notify();
  }

  // Debug output
  Serial.print("Angle: "); Serial.print(angleY);
  Serial.print(" Error: "); Serial.print(error);
  Serial.print(" P: "); Serial.print(KP * error);
  Serial.print(" I: "); Serial.print(KI * integral);
  Serial.print(" D: "); Serial.print(KD * derivative);
  Serial.print(" PWM: "); Serial.println(manualMode ? manualSpeed : pidOutput);
}
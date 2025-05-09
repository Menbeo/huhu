// Motor A pins
int enA = 15;
int in1 = 25;
int in2 = 33;
int EncoderA1 = 12;
int EncoderA2 = 13;

volatile int pos = 0; // current position (pulse count)

long prevT = 0;
float eprev = 0;
float eintegral = 0;

const int pulses_per_rev = 263;
const int target_degrees = 360;
const int target_pos = (pulses_per_rev * target_degrees)/360;

bool motorStopped = false;

void setup() {
  Serial.begin(9600);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(enA, OUTPUT);
  pinMode(EncoderA1, INPUT);
  pinMode(EncoderA2, INPUT);
  attachInterrupt(digitalPinToInterrupt(EncoderA1), readencoder, RISING);
  Serial.println("Rotating motor to 90 degrees...");
}

void loop() {
  // Print position continuously
  Serial.print("Target: ");
  Serial.print(target_pos);
  Serial.print(" | Pos: ");
  Serial.print(pos);
  Serial.print(" | Error: ");
  Serial.println(target_pos - pos);

  // Stop if target is reached
  if (!motorStopped && abs(pos) >= target_pos) {
    analogWrite(enA, 0);
    Serial.println("Rotation complete!");
    motorStopped = true;
  }

  // Skip PID if motor is stopped
  if (motorStopped) {
    delay(100); // just update position
    return;
  }

  // PID control
  float Kp = 80;
  float Ki = 0.001;
  float Kd = 4;

  long currentT = millis();
  float dt = (currentT - prevT) / 1000.0;
  prevT = currentT;

  float error = target_pos - pos;
  eintegral += error * dt;
  float dedt = (error - eprev) / dt;
  float u = Kp * error + Kd * dedt + Ki * eintegral;
  eprev = error;

  float pwr = fabs(u);
  if (pwr > 255) pwr = 255;

  int dir = (u > 0) ? 1 : -1;

  digitalWrite(in1, dir == 1 ? HIGH : LOW);
  digitalWrite(in2, dir == 1 ? LOW : HIGH);

  analogWrite(enA, (int)pwr);

  delay(100); // print and update every 100 ms
}

void readencoder() {
  int A2 = digitalRead(EncoderA2);
  if (A2 == HIGH) {
    pos++;
  } else {
    pos--;
  }
}

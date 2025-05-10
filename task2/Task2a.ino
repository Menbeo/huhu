// ESP32 code for manual wheel rotation to count encoder pulses

// Motor A pins
const int IN1 = 17;
const int IN2 = 5;
const int ENA = 16; // PWM pin

// Encoder pins
const int ENCODER_A = 12;
const int ENCODER_B = 13;

// Encoder Parameters
#define PPR 100       // Pulses per revolution per channel (adjust based on encoder)
#define PULSES_PER_REV (PPR * 4)  // Total pulses in CHANGE mode (4x resolution)
#define DEBOUNCE_TIME 2000  // Debounce time in microseconds

// Volatile variables for interrupt handling
volatile long pulseCount = 0;  // Total pulses counted
volatile unsigned long lastInterruptTime = 0;

// Interrupt Service Routine for Encoder
void IRAM_ATTR handleEncoder() {
  // Debounce: ignore interrupts within DEBOUNCE_TIME
  unsigned long currentTime = micros();
  if (currentTime - lastInterruptTime < DEBOUNCE_TIME) {
    return;
  }
  lastInterruptTime = currentTime;

  // Read encoder states
  static int lastA = LOW;
  int a = digitalRead(ENCODER_A);
  int b = digitalRead(ENCODER_B);

  // Quadrature decoding table
  if (lastA != a) {  // Only process changes on ENCODER_A
    if (a == HIGH) {
      pulseCount += (b == LOW) ? 1 : -1;  // A rising: B low -> forward, B high -> reverse
    } else {
      pulseCount += (b == HIGH) ? 1 : -1;  // A falling: B high -> forward, B low -> reverse
    }
  }
  lastA = a;

  // Debug: print encoder states (uncomment for troubleshooting)
  // Serial.print("A: "); Serial.print(a); Serial.print(" B: "); Serial.print(b);
  // Serial.print(" Count: "); Serial.println(pulseCount);
}

void setup() {
  // Initialize Serial
  Serial.begin(115200);

  // Set up L298N pins
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENA, OUTPUT);

  // Explicitly stop motor
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  analogWrite(ENA, 0);
  Serial.println("Motor stopped (IN1=LOW, IN2=LOW, ENA=0). Ready for manual rotation.");

  // Set up encoder pins with internal pull-ups
  pinMode(ENCODER_A, INPUT_PULLUP);
  pinMode(ENCODER_B, INPUT_PULLUP);

  // Attach interrupts for both channels in CHANGE mode
  attachInterrupt(digitalPinToInterrupt(ENCODER_A), handleEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_B), handleEncoder, CHANGE);

  // Print encoder settings
  Serial.print("Encoder PPR (per channel): ");
  Serial.println(PPR);
  Serial.print("Total pulses per revolution (CHANGE mode): ");
  Serial.println(PULSES_PER_REV);
  Serial.println("Rotate wheel manually 360 degrees to count pulses.");
}

void loop() {
  // Static variables to track pulses for one revolution
  static long startPulseCount = 0;
  static bool countingRevolution = false;

  // Start counting for a new revolution
  if (!countingRevolution) {
    startPulseCount = pulseCount;
    countingRevolution = true;
  }

  // Check if one revolution is completed
  if (abs(pulseCount - startPulseCount) >= PULSES_PER_REV) {
    long pulsesInRevolution = abs(pulseCount - startPulseCount);
    Serial.print("Revolution completed! Pulses counted: ");
    Serial.println(pulsesInRevolution);
    countingRevolution = false;  // Reset for next revolution
  }

  // Print current pulse count periodically, ignore small noise
  if (millis() % 1000 == 0) {
    if (abs(pulseCount) > 5) {  // Ignore counts <= 5 (likely noise)
      Serial.print("Current pulse count: ");
      Serial.println(pulseCount);
    } else {
      Serial.println("Pulse count stable (no significant movement)");
    }
    delay(1);  // Prevent multiple prints
  }
}

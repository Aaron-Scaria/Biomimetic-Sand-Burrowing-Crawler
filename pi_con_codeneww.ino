#include <PID_v1.h>

// SAFE PINS for ESP32-S3 (Avoids 11, 12, 13)
#define ENC1_A 4
#define ENC1_B 5
#define ENC2_A 18 
#define ENC2_B 19 

// TB6612FNG Driver Pins
#define PWMA 9
#define AIN1 6
#define AIN2 7
#define PWMB 16
#define BIN1 17
#define BIN2 15

#define CPR 4200
#define CONTROL_INTERVAL 100 

double targetRPM = 20; 
double rpm1, rpm2;
double pwm1, pwm2;

// PID Gains: High Ki is essential for sand to overcome resistance
double Kp = 3.5, Ki = 4.0, Kd = 0.1;

PID motor1PID(&rpm1, &pwm1, &targetRPM, Kp, Ki, Kd, DIRECT);
PID motor2PID(&rpm2, &pwm2, &targetRPM, Kp, Ki, Kd, DIRECT);

volatile long encoder1Count = 0;
volatile long encoder2Count = 0;
unsigned long lastTime = 0;

void IRAM_ATTR encoder1ISR() { encoder1Count++; }
void IRAM_ATTR encoder2ISR() { encoder2Count++; }

void setup() {
  Serial.begin(115200);
  // S3 Native USB Handshake
  while (!Serial && millis() < 3000); 

  pinMode(AIN1, OUTPUT); pinMode(AIN2, OUTPUT);
  pinMode(BIN1, OUTPUT); pinMode(BIN2, OUTPUT);
  pinMode(ENC1_A, INPUT_PULLUP); pinMode(ENC2_A, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(ENC1_A), encoder1ISR, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC2_A), encoder2ISR, RISING);

  // Initialize ESP32-S3 PWM
  ledcAttach(PWMA, 5000, 8);
  ledcAttach(PWMB, 5000, 8);

  // Set Motor Directions
  digitalWrite(AIN1, HIGH); digitalWrite(AIN2, LOW);
  digitalWrite(BIN1, HIGH); digitalWrite(BIN2,LOW);

  motor1PID.SetMode(AUTOMATIC);
  motor2PID.SetMode(AUTOMATIC);
  motor1PID.SetOutputLimits(0, 255);
  motor2PID.SetOutputLimits(0, 255);

  lastTime = millis();
}

void loop() {
  if (millis() - lastTime >= CONTROL_INTERVAL) {
    unsigned long duration = millis() - lastTime;
    lastTime = millis();

    // Reset counts atomically
    noInterrupts();
    long c1 = encoder1Count;
    long c2 = encoder2Count;
    encoder1Count = 0;
    encoder2Count = 0;
    interrupts();

    // Convert pulses to RPM
    rpm1 = (c1 * 60000.0) / (double(CPR) * duration);
    rpm2 = (c2 * 60000.0) / (double(CPR) * duration);

    // Run standard PID
    motor1PID.Compute();
    motor2PID.Compute();

    // --- SYNCHRONIZATION LOGIC ---
    // If M1 is faster than M2, syncError is positive.
    double syncError = rpm1 - rpm2;
    double Ksync = 2.0; // The "Sync Factor" you asked for

    // Apply correction: Motor 2 tries to catch up to Motor 1
    double finalPWM1 = pwm1;
    double finalPWM2 = pwm2 + (syncError * Ksync);

    // --- FLUIDIZATION TORQUE MINIMUM ---
    // N20s won't turn in sand below a certain PWM. We force a floor of 80.
    if (targetRPM > 0) {
      if (finalPWM1 > 0 && finalPWM1 < 80) finalPWM1 = 80;
      if (finalPWM2 > 0 && finalPWM2 < 80) finalPWM2 = 80;
    }

    // Hardware output
    ledcWrite(PWMA, (int)constrain(finalPWM1, 0, 255));
    ledcWrite(PWMB, (int)constrain(finalPWM2, 0, 255));

    // Monitor for tuning
    Serial.printf("T:%d | R1:%.1f | R2:%.1f | P1:%.0f | P2:%.0f\n", 
                  (int)targetRPM, rpm1, rpm2, finalPWM1, finalPWM2);
  }
}
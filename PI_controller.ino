// ================= MOTOR A =================
#define ENC_A 13
#define PWMA 17
#define AIN1 14
#define AIN2 16

// ================= MOTOR B =================
#define ENC_B 18
#define PWMB 19
#define BIN1 21
#define BIN2 22

// ================= VARIABLES =================
volatile long pulseA = 0;
volatile long pulseB = 0;

volatile int countA = 0;
volatile int countB = 0;

// PI Gains
#define KP 0.5
#define KI 0.05

// Anti-windup
#define INTEGRAL_LIMIT 200

// Base speed
#define BASE_PWM 80

// Max correction
#define MAX_CORRECTION 10

// Same speed limits for both motors
#define MAX_PWM 120
#define MIN_PWM 50

// B runs 1.04x A — 2 extra pulses over 50
// 52/50 = 1.04
#define B_FACTOR 1.125

// Reset only when A hits 50
#define REV_PULSES 50

#define PULSES_PER_REV 50

long lastPulseA = 0;
long lastPulseB = 0;
unsigned long lastTime = 0;

float rpmA = 0;
float rpmB = 0;

float integralSync = 0;

// ================= ISR =================
void IRAM_ATTR isrA() {
  countA++;
  if (countA >= 41) {
    pulseA++;
    countA = 0;
  }
}

void IRAM_ATTR isrB() {
  countB++;
  if (countB >= 41) {
    pulseB++;
    countB = 0;
  }
}

// ================= MOTOR CONTROL =================
void motorA(float pwm) {
  pwm = constrain(pwm, MIN_PWM, MAX_PWM);
  digitalWrite(AIN1, HIGH);
  digitalWrite(AIN2, LOW);
  ledcWrite(PWMA, (int)pwm);
}

void motorB(float pwm) {
  pwm = constrain(pwm, MIN_PWM, MAX_PWM);
  digitalWrite(BIN1, HIGH);
  digitalWrite(BIN2, LOW);
  ledcWrite(PWMB, (int)pwm);
}

void stopMotors() {
  digitalWrite(AIN1, HIGH);
  digitalWrite(AIN2, HIGH);
  digitalWrite(BIN1, HIGH);
  digitalWrite(BIN2, HIGH);
  ledcWrite(PWMA, 0);
  ledcWrite(PWMB, 0);
}

// ================= SETUP =================
void setup() {
  Serial.begin(115200);

  pinMode(ENC_A, INPUT_PULLUP);
  pinMode(ENC_B, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(ENC_A), isrA, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_B), isrB, RISING);

  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);

  ledcAttach(PWMA, 5000, 8);
  ledcAttach(PWMB, 5000, 8);

  lastTime = millis();
  Serial.println("Dual Motor Sync - B Factor 1.04");
}

// ================= LOOP =================
void loop() {

  // Reset only when A hits 50 — B will be at ~52 at this point
  if (pulseA >= REV_PULSES) {
    noInterrupts();
    pulseA = 0;
    pulseB = 0;
    interrupts();

    integralSync = 0;
    lastPulseA = 0;
    lastPulseB = 0;

    Serial.println("Revolution complete — pulses reset");
  }

  // B expected = A * 1.04 at every pulse
  // A=10 → B=10.4, A=25 → B=26, A=50 → B=52
  float syncError = ((float)pulseA * B_FACTOR) - (float)pulseB;

  // PI on sync error
  integralSync += syncError;
  integralSync = constrain(integralSync, -INTEGRAL_LIMIT, INTEGRAL_LIMIT);

  float correction = (KP * syncError) + (KI * integralSync);
  correction = constrain(correction, -MAX_CORRECTION, MAX_CORRECTION);

  // Base speed with correction applied symmetrically
  float finalA = BASE_PWM - correction;
  float finalB = BASE_PWM + correction;

  // ================= RPM CALCULATION =================
  if (millis() - lastTime >= 100) {
    long deltaA = pulseA - lastPulseA;
    long deltaB = pulseB - lastPulseB;

    rpmA = (deltaA * 600.0) / PULSES_PER_REV;
    rpmB = (deltaB * 600.0) / PULSES_PER_REV;

    lastPulseA = pulseA;
    lastPulseB = pulseB;
    lastTime = millis();
  }

  // ================= DEBUG =================
  Serial.print("A: "); Serial.print(pulseA);
  Serial.print(" B: "); Serial.print(pulseB);
  Serial.print(" Expected B: "); Serial.print((float)pulseA * B_FACTOR);
  Serial.print(" SyncErr: "); Serial.print(syncError);
  Serial.print(" Correction: "); Serial.print(correction);
  Serial.print(" pwmA: "); Serial.print(finalA);
  Serial.print(" pwmB: "); Serial.print(finalB);
  Serial.print(" RPM_A: "); Serial.print(rpmA);
  Serial.print(" RPM_B: "); Serial.println(rpmB);

  // Always drive motors
  motorA(finalA);
  motorB(finalB);

  delay(20);
}
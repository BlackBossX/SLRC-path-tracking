// Motor A (Left) Pins
const int PWM_A = 3;
const int AIN1 = 11;
const int AIN2 = 2;

// Motor B (Right) Pins
const int PWM_B = 5;
const int BIN1 = 12;
const int BIN2 = 4;

// Encoder A (Left) Pins
const int ENC_A_1 = 8;
const int ENC_A_2 = 9;

// Encoder B (Right) Pins
const int ENC_B_1 = 6;
const int ENC_B_2 = 7;

// Variables to keep track of encoder counts
volatile long encoderACount = 0;
volatile long encoderBCount = 0;

// Interrupt Service Routines for Encoders
void isr_encoderA() {
  // Read the other pin to determine direction
  if (digitalRead(ENC_A_2) == HIGH) {
    encoderACount++;
  } else {
    encoderACount--;
  }
}

void isr_encoderB() {
  if (digitalRead(ENC_B_2) == HIGH) {
    encoderBCount++;
  } else {
    encoderBCount--;
  }
}

void setup() {
  // Start Serial Monitor
  Serial.begin(115200);

  // Note: If using an Arduino Uno/Nano, pins 0 and 1 are the USB Serial TX/RX pins! 
  // Using them for Motor controls might conflict with the Serial Monitor.
  // If you are using a Raspberry Pi Pico using the Arduino IDE, pins 0 & 1 are fine.
  
  // Motor Pins Setup
  pinMode(PWM_A, OUTPUT);
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  
  pinMode(PWM_B, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);

  // Encoder Pins Setup
  pinMode(ENC_A_1, INPUT_PULLUP);
  pinMode(ENC_A_2, INPUT_PULLUP);
  pinMode(ENC_B_1, INPUT_PULLUP);
  pinMode(ENC_B_2, INPUT_PULLUP);

  // Attach interrupts for Encoders
  // Note: on Uno, standard interrupt pins are only 2 and 3. 
  // On Pico/ESP32, all pins support interrupts.
  attachInterrupt(digitalPinToInterrupt(ENC_A_1), isr_encoderA, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_B_1), isr_encoderB, RISING);

  // NOTE FOR TB6612FNG:
  // Don't forget that the "STBY" (Standby) pin on the TB6612 driver 
  // MUST be connected to 3.3V / 5V (or a digital pin set to HIGH) to work!

  Serial.println("Robot Motor and Encoder Test Starting...");
  delay(2000);
}

void loop() {
  Serial.println("=== TEST 1: Left Motor FORWARD ===");
  moveMotors(150, 0); // Only Left
  monitorEncoders(2000);

  Serial.println("=== TEST 2: Left Motor BACKWARD ===");
  moveMotors(-150, 0);
  monitorEncoders(2000);

  Serial.println("=== TEST 3: Right Motor FORWARD ===");
  moveMotors(0, 150); // Only Right
  monitorEncoders(2000);

  Serial.println("=== TEST 4: Right Motor BACKWARD ===");
  moveMotors(0, -150); // This is where the right motor fails for you.
  monitorEncoders(2000);
  
  Serial.println("=== STOPPING ===");
  moveMotors(0, 0);
  monitorEncoders(2000);
}

// Function to set Motor Speeds (-255 to 255)
void moveMotors(int speedLeft, int speedRight) {
  // Left Motor (Motor A)
  if (speedLeft > 0) {
    digitalWrite(AIN1, HIGH);
    digitalWrite(AIN2, LOW);
    analogWrite(PWM_A, speedLeft);
  } else if (speedLeft < 0) {
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, HIGH);
    analogWrite(PWM_A, -speedLeft);
  } else {
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, LOW);
    analogWrite(PWM_A, 0);
  }

  // Right Motor (Motor B)
  if (speedRight > 0) {
    digitalWrite(BIN1, HIGH);
    digitalWrite(BIN2, LOW);
    analogWrite(PWM_B, speedRight);
  } else if (speedRight < 0) {
    digitalWrite(BIN1, LOW);
    digitalWrite(BIN2, HIGH);
    analogWrite(PWM_B, -speedRight);
  } else {
    digitalWrite(BIN1, LOW);
    digitalWrite(BIN2, LOW);
    analogWrite(PWM_B, 0);
  }
}

// Function to print encoder counts to Serial Monitor for a set duration
void monitorEncoders(int durationMs) {
  long startTime = millis();
  while (millis() - startTime < durationMs) {
    Serial.print("Left Encoder: ");
    Serial.print(encoderACount);
    Serial.print(" | Right Encoder: ");
    Serial.println(encoderBCount);
    delay(100); // 10 readings per second
  }
}

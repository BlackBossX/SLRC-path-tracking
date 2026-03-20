// --- LEFT ENCODER --- (Uses Hardware Interrupt)
const int ENC_LEFT_A = 2; // UNO Interrupt 0
const int ENC_LEFT_B = 3; // Standard digital pin

// --- RIGHT ENCODER --- (Uses Hardware Interrupt)
const int ENC_RIGHT_A = 4; // UNO Interrupt 1
const int ENC_RIGHT_B = 5; // Standard digital pin

// Variables to keep track of encoder counts
volatile long leftEncoderCount = 0;
volatile long rightEncoderCount = 0;

void isr_left_encoder() {
  // Triggered on RISING edge of ENC_LEFT_A (Pin 2)
  if (digitalRead(ENC_LEFT_B) == HIGH) {
    leftEncoderCount++; 
  } else {
    leftEncoderCount--; 
  }
}

void isr_right_encoder() {
  // Triggered on RISING edge of ENC_RIGHT_A (Pin 3)
  if (digitalRead(ENC_RIGHT_B) == HIGH) {
    rightEncoderCount++; 
  } else {
    rightEncoderCount--; 
  }
}

void setup() {
  Serial.begin(115200);

  // Setup Left Encoder Pins
  pinMode(ENC_LEFT_A, INPUT_PULLUP);
  pinMode(ENC_LEFT_B, INPUT_PULLUP);

  // Setup Right Encoder Pins
  pinMode(ENC_RIGHT_A, INPUT_PULLUP);
  pinMode(ENC_RIGHT_B, INPUT_PULLUP);

  // Attach interrupts for BOTH encoders
  // The 'A' channels MUST be on pins 2 and 3 for the Uno.
  // The 'B' channels can be on any other standard digital pins (we chose 4 and 5)
  attachInterrupt(digitalPinToInterrupt(ENC_LEFT_A), isr_left_encoder, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_RIGHT_A), isr_right_encoder, RISING);

  Serial.println("Arduino UNO Dual Encoder Test Started!");
  Serial.println("Left Enc (Pins 2 & 4) | Right Enc (Pins 3 & 5)");
}

void loop() {
  Serial.print("Left Encoder: ");
  Serial.print(leftEncoderCount);
  Serial.print("\t|\tRight Encoder: ");
  Serial.println(rightEncoderCount);
  
  delay(100);
}

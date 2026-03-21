// --- LEFT ENCODER --- (Uses Hardware Interrupt)
const int ENC_LEFT_A = 2; // UNO Interrupt 0
const int ENC_LEFT_B = 4; // Standard digital pin

// --- RIGHT ENCODER --- (Uses Hardware Interrupt)
const int ENC_RIGHT_A = 3; // UNO Interrupt 1
const int ENC_RIGHT_B = 5; // Standard digital pin

// Variables to keep track of encoder counts
volatile long leftEncoderCount = 0;
volatile long rightEncoderCount = 0;

// Variables for debounce to ignore electrical noise
volatile unsigned long lastLeftInterrupt = 0;
volatile unsigned long lastRightInterrupt = 0;

void isr_left_encoder() {
  unsigned long currentTime = micros();
  // Ignore pulses that happen closer than 50 microseconds together (electrical noise)
  if (currentTime - lastLeftInterrupt > 50) {
    if (digitalRead(ENC_LEFT_B) == HIGH) {
      leftEncoderCount++; 
    } else {
      leftEncoderCount--; 
    }
    lastLeftInterrupt = currentTime;
  }
}

void isr_right_encoder() {
  unsigned long currentTime = micros();
  if (currentTime - lastRightInterrupt > 50) {
    if (digitalRead(ENC_RIGHT_B) == HIGH) {
      rightEncoderCount++; 
    } else {
      rightEncoderCount--; 
    }
    lastRightInterrupt = currentTime;
  }
}

void setup() {
  // 115200 Baud Rate. This automatically enables Hardware TX (Pin 1) and RX (Pin 0)
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
  long tempLeft = 0;
  long tempRight = 0;

  // IMPORTANT: The Arduino Uno is an 8-bit microcontroller. 
  // It takes 4 steps to read a 32-bit 'long' variable. If an interrupt 
  // fires right in the middle of reading it, you will get massive garbage/corrupted values.
  // We must pause interrupts for a split microsecond to copy them safely.
  noInterrupts();
  tempLeft = leftEncoderCount;
  tempRight = rightEncoderCount;
  interrupts();

  // Format the output exactly as "LeftValue,RightValue" for the Pi to parse easily
  Serial.print(tempLeft);
  Serial.print(",");
  Serial.println(tempRight);
  
  // 20ms delay is 50 times a second, fast enough for robot response, but doesn't overwhelm serial
  delay(20);
}

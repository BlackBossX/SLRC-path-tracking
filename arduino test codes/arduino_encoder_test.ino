// Encoder Test Code for Arduino

// Define Encoder Pins
// Adjust these to match the pins you are using on your Arduino
const int ENC_LEFT_A = 2; // Usually an interrupt pin (2 or 3 on Uno)
const int ENC_LEFT_B = 4;

const int ENC_RIGHT_A = 3; // Usually an interrupt pin (2 or 3 on Uno)
const int ENC_RIGHT_B = 5;

// Variables to hold the encoder counts. 
// Use volatile because they are changed inside interrupt service routines (ISRs)
volatile long leftEncoderCount = 0;
volatile long rightEncoderCount = 0;

void setup() {
  Serial.begin(115200);

  // Set encoder pins as inputs with internal pull-up resistors enabled
  pinMode(ENC_LEFT_A, INPUT_PULLUP);
  pinMode(ENC_LEFT_B, INPUT_PULLUP);
  
  pinMode(ENC_RIGHT_A, INPUT_PULLUP);
  pinMode(ENC_RIGHT_B, INPUT_PULLUP);

  // Attach interrupts to the A pins
  // RISING means the interrupt will trigger when the pin goes from LOW to HIGH
  attachInterrupt(digitalPinToInterrupt(ENC_LEFT_A), leftEncoderISR, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_RIGHT_A), rightEncoderISR, RISING);

  Serial.println("Encoder Test Started");
  Serial.println("Spin the wheels manually!");
}

void loop() {
  // Print the current counts every 100ms
  Serial.print("Left Enc: ");
  Serial.print(leftEncoderCount);
  Serial.print("  |  Right Enc: ");
  Serial.println(rightEncoderCount);
  
  delay(100);
}

// Interrupt Service Routine for Left Encoder
void leftEncoderISR() {
  // Read the state of the B pin to determine direction
  if (digitalRead(ENC_LEFT_B) == HIGH) {
    leftEncoderCount++;
  } else {
    leftEncoderCount--;
  }
}

// Interrupt Service Routine for Right Encoder
void rightEncoderISR() {
  // Read the state of the B pin to determine direction
  if (digitalRead(ENC_RIGHT_B) == HIGH) {
    rightEncoderCount++;
  } else {
    rightEncoderCount--;
  }
}

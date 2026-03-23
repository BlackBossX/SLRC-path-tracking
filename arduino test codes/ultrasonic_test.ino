// Ultrasonic Sensor Test for Arduino

// Define Pins
const int TRIG_PIN = 9;
const int ECHO_PIN = 10;

// Variables for calculation
long duration;
int distance_cm;

void setup() {
  Serial.begin(115200);
  
  // Setup Sensor Pins
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  // Initialize Trigger LOW
  digitalWrite(TRIG_PIN, LOW);
  
  Serial.println("Ultrasonic Sensor Test Started!");
}

void loop() {
  // 1. Send out a 10-microsecond ping pulse
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  // 2. Read the echo return time in microseconds
  // pulseIn() blocks until the ECHO pin goes HIGH, then waits for it to go LOW
  duration = pulseIn(ECHO_PIN, HIGH);

  // 3. Calculate Distance
  // Distance = (Time passed * Speed of Sound) / 2
  // Speed of sound is ~0.034 cm/microsecond
  distance_cm = duration * 0.034 / 2;

  // 4. Print the result
  Serial.print("Distance: ");
  Serial.print(distance_cm);
  Serial.println(" cm");

  // Delay before the next reading
  delay(100);
}

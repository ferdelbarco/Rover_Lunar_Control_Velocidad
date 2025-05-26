int hallSensorPin = 8;    // Pin connected to the Hall sensor
int motorPin1 = 2;        // Motor connected to pin 2
int motorPin2 = 3;        // Motor connected to pin 3

int lastSensorState = 0;  // Last sensor state (to detect changes)
volatile int pulses = 0;  // Pulse counter
unsigned long timeold = 0; // Time of the previous pulse
float rpm = 0;             // Variable to store RPM
float rps = 0;             // Variable to store RPS

void setup() {
  pinMode(hallSensorPin, INPUT);   // Set the Hall sensor pin as input
  pinMode(motorPin1, OUTPUT);      // Set the motor pins as outputs
  pinMode(motorPin2, OUTPUT);

  Serial.begin(9600);  // Start the serial monitor
}

void loop() {
  // Activate the motor (turn it on)
  digitalWrite(motorPin1, LOW);  // Motor on
  digitalWrite(motorPin2, HIGH);

  // Read the current state of the Hall sensor
  int sensorState = digitalRead(hallSensorPin);

  // Check for a change in the sensor state (from LOW to HIGH)
  if (sensorState == HIGH && lastSensorState == LOW) {
    pulses++;  // Increment the pulse count when the magnet is detected
  }

  // Update the last sensor state
  lastSensorState = sensorState;

  if (pulses >= 1) {  // When at least one pulse has been counted
    // Calculate RPM and RPS
    rpm = 60000.0 / (millis() - timeold) * pulses;  // RPM = 60000ms / time between pulses * pulse count
    rps = rpm / 60.0;  // RPS = RPM / 60

    // Print RPM and RPS to the Serial Monitor
    Serial.print("RPM: ");
    Serial.println(rpm);
    Serial.print("RPS: ");
    Serial.println(rps);

    // Reset for next measurement
    timeold = millis();  // Update the time for the next pulse
    pulses = 0;          // Reset pulse counter
  }
}


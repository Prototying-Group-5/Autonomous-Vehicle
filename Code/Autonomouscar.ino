// Pin definitions for motor control
#define enA 10 // Enable1 L293 Pin enA 
#define in1 6  // Motor1 L293 Pin in1 
#define in2 7  // Motor1 L293 Pin in2 
#define in3 12 // Motor2 L293 Pin in3 
#define in4 8  // Motor2 L293 Pin in4 
#define enB 9  // Enable2 L293 Pin enB 

// Pin definitions for color sensor
#define S0_PIN A0 // Color sensor S0 pin
#define S1_PIN A1 // Color sensor S1 pin
#define S2_PIN A2 // Color sensor S2 pin
#define S3_PIN A3 // Color sensor S3 pin
#define OUT_PIN A4 // Color sensor OUT pin

// Pin definitions for IR sensors
#define R_S 5 // IR sensor Right
#define L_S 4 // IR sensor Left

// Pin definitions for ultrasonic sensor
int trigPin = 2;    // TRIG pin
int echoPin = 3;    // ECHO pin

// Variables for ultrasonic sensor
float duration, distance;

// Motor speed constants
const int maxSpeed = 90; // Maximum motor speed
const int minSpeed = 82; // Minimum motor speed
const int accelRate = 1;  // Speed increment per step
const int decelRate = 3;  // Speed decrement per step

// Current motor speed
int motorSpeed = 80;  // Initial motor speed

// Enumeration for Vehicle states
enum State { FOLLOW_LINE, AVOID_OBSTACLE, PARK, STOPPED };
State currentState = FOLLOW_LINE; // Initial state

void setup() {
  Serial.begin(9600); // Initialize serial communication at 9600 baud

  // Set the S0, S1, S2, S3 Pins as Output for the color sensor
  pinMode(S0_PIN, OUTPUT);
  pinMode(S1_PIN, OUTPUT);
  pinMode(S2_PIN, OUTPUT);
  pinMode(S3_PIN, OUTPUT);

  // Set OUT_PIN as Input for the color sensor
  pinMode(OUT_PIN, INPUT);

  // Set Pulse Width scaling to 20% for the color sensor
  digitalWrite(S0_PIN, HIGH);
  digitalWrite(S1_PIN, LOW);

  // Initialize motor and sensor pins
  pinMode(R_S, INPUT); // Set right IR sensor as input
  pinMode(L_S, INPUT); // Set left IR sensor as input
  pinMode(enA, OUTPUT); // Set Enable1 pin as output
  pinMode(in1, OUTPUT); // Set Motor1 in1 pin as output
  pinMode(in2, OUTPUT); // Set Motor1 in2 pin as output
  pinMode(in3, OUTPUT); // Set Motor2 in3 pin as output
  pinMode(in4, OUTPUT); // Set Motor2 in4 pin as output
  pinMode(enB, OUTPUT); // Set Enable2 pin as output

  // Initialize ultrasonic sensor pins
  pinMode(trigPin, OUTPUT); // Set TRIG pin as output
  pinMode(echoPin, INPUT);  // Set ECHO pin as input
}

void loop() {
  ultrasonic(); // Measure distance using ultrasonic sensor

  // Process color sensor values
  int b = process_blue_value();
  int g = process_green_value();
  int r = process_red_value();

  // Check if an obstacle is within 7 cm
  if (distance < 7) {
    Stop(); // Stop the car
    delay(1000); // Wait for 1 second

    // Determine the current state based on color sensor values
    if (r < g && r < b && g > b) {
      Serial.println("Colour Red");
      currentState = AVOID_OBSTACLE; // Avoid obstacle if red is detected
    } else if (r > g && r > b && g > b) {
      currentState = PARK; // Park if blue is detected
      Serial.println("Colour Blue");
    } else if (g < r && g < b && r > b) {
      currentState = STOPPED; // Stop if green is detected
      Serial.println("Colour Green");
    } else {
      currentState = FOLLOW_LINE; // Follow line otherwise
    }
  }

  // Read IR sensor values
  int rightSensor = digitalRead(R_S);
  int leftSensor = digitalRead(L_S);

  // Execute behavior based on current state
  switch (currentState) {
    case FOLLOW_LINE:
      followLine(rightSensor, leftSensor); // Follow the line
      break;
    case AVOID_OBSTACLE:
      avoidObstacle(); // Avoid obstacle
      currentState = FOLLOW_LINE; // Return to line following after avoiding obstacle
      break;
    case PARK:
      Park(); // Park the car
      break;
    case STOPPED:
      Stop(); // Stop the Car
      break;
  }
}

// Function to follow the line based on IR sensor values
void followLine(int rightSensor, int leftSensor) {
  // Determine direction based on sensor readings
  switch (rightSensor << 1 | leftSensor) {
    case 0b11: // Both sensors on the line
      Stop();
      delay(1000);
      break;
    case 0b10: // Right sensor on the line, left sensor off
      turnRight();
      decelerateMotor();
      break;
    case 0b01: // Left sensor on the line, right sensor off
      turnLeft();
      decelerateMotor();
      break;
    case 0b00: // Both sensors off the line
      forward();
      accelerateMotor();
      break;
    default:
      forward();
      decelerateMotor();
      break;
  }
}

// Function to move the Car forward
void forward() { 
  digitalWrite(in1, LOW); // Set right motor forward pin LOW
  digitalWrite(in2, HIGH); // Set right motor backward pin HIGH
  digitalWrite(in3, HIGH); // Set left motor forward pin HIGH (reversed logic)
  digitalWrite(in4, LOW); // Set left motor backward pin LOW (reversed logic)
  analogWrite(enA, motorSpeed); // Set right motor speed
  analogWrite(enB, motorSpeed); // Set left motor speed
}

// Function to move the Car backward
void backward() { 
  digitalWrite(in1, HIGH); // Set right motor forward pin HIGH
  digitalWrite(in2, LOW); // Set right motor backward pin LOW
  digitalWrite(in3, LOW); // Set left motor forward pin LOW (reversed logic)
  digitalWrite(in4, HIGH); // Set left motor backward pin HIGH (reversed logic)
  analogWrite(enA, 96); // Set right motor speed
  analogWrite(enB, 95); // Set left motor speed
}

// Function to turn the Car right
void turnRight() { 
  digitalWrite(in1, LOW); // Set right motor forward pin LOW
  digitalWrite(in2, HIGH); // Set right motor backward pin HIGH
  digitalWrite(in3, LOW); // Set left motor backward pin LOW (reversed logic)
  digitalWrite(in4, LOW); // Set left motor forward pin LOW (reversed logic)
  analogWrite(enA, motorSpeed); // Set right motor speed
  analogWrite(enB, motorSpeed); // Set left motor speed
}

// Function to turn the Car right with adjusted speed
void aturnRight() { 
  digitalWrite(in1, LOW); // Set right motor forward pin LOW
  digitalWrite(in2, HIGH); // Set right motor backward pin HIGH
  digitalWrite(in3, LOW); // Set left motor backward pin LOW (reversed logic)
  digitalWrite(in4, LOW); // Set left motor forward pin LOW (reversed logic)
  analogWrite(enA, 115); // Set right motor speed to 115
  analogWrite(enB, 0); // Set left motor speed to 0
}

// Function to turn the Car left
void turnLeft() { 
  digitalWrite(in1, LOW); // Set right motor forward pin LOW
  digitalWrite(in2, LOW); // Set right motor backward pin LOW
  digitalWrite(in3, HIGH); // Set left motor forward pin HIGH (reversed logic)
  digitalWrite(in4, LOW); // Set left motor backward pin LOW (reversed logic)
  analogWrite(enA, motorSpeed); // Set right motor speed
  analogWrite(enB, motorSpeed); // Set left motor speed
}

// Function to turn the Car left with adjusted speed
void aturnLeft() { 
  digitalWrite(in1, LOW); // Set right motor forward pin LOW
  digitalWrite(in2, LOW); // Set right motor backward pin LOW
  digitalWrite(in3, HIGH); // Set left motor forward pin HIGH (reversed logic)
  digitalWrite(in4, LOW); // Set left motor backward pin LOW (reversed logic)
  analogWrite(enB, 120); // Set left motor speed to 120
  analogWrite(enA, 0); // Set right motor speed to 0
}

// Function to stop the Car
void Stop() { 
  digitalWrite(in1, LOW); // Set right motor forward pin LOW
  digitalWrite(in2, LOW); // Set right motor backward pin LOW
  digitalWrite(in3, LOW); // Set left motor forward pin LOW
  digitalWrite(in4, LOW); // Set left motor backward pin LOW
}

// Function to measure distance using ultrasonic sensor
void ultrasonic() {
  digitalWrite(trigPin, LOW); // Set TRIG pin LOW
  delayMicroseconds(2); // Wait for 2 microseconds
  digitalWrite(trigPin, HIGH); // Set TRIG pin HIGH
  delayMicroseconds(10); // Wait for 10 microseconds
  digitalWrite(trigPin, LOW); // Set TRIG pin LOW

  // Measure duration of echo pulse
  duration = pulseIn(echoPin, HIGH);
  // Calculate distance in cm
  distance = duration * 0.034 / 2;
}

// Function to avoid obstacles
void avoidObstacle() {
  // Step 1: Stop and move backward to avoid the obstacle
  backward(); // Move backward
  delay(1000); // Wait for 1 second (adjust based on distance)
  Stop(); // Stop the Car
  delay(500); // Wait for 0.5 seconds

  // Step 2: Turn left to start avoiding the obstacle
  aturnLeft(); // Turn left with adjusted speed
  delay(1200); // Wait for 1.2 seconds (adjust for turn angle)
  currentState = FOLLOW_LINE; // Set state to follow line

  // Step 3: Move forward past the obstacle
  forward(); // Move forward
  delay(700); // Wait for 0.7 seconds (adjust for obstacle size)
  currentState = FOLLOW_LINE; // Set state to follow line

  // Step 4: Turn right to realign with the path
  aturnRight(); // Turn right with adjusted speed
  delay(1700); // Wait for 1.7 seconds (adjust for turn angle)
  currentState = FOLLOW_LINE; // Set state to follow line
  forward(); // Move forward
  delay(300); // Wait for 0.3 seconds
  currentState = FOLLOW_LINE; // Set state to follow line
}

// Function to park the Car
void Park() {
  int b = process_blue_value(); // Process blue color value
  int g = process_green_value(); // Process green color value
  int r = process_red_value(); // Process red color value

  backward(); // Move backward
  delay(1000); // Wait for 1 second
  Stop(); // Stop the Car
  delay(500); // Wait for 0.5 seconds
  currentState = FOLLOW_LINE; // Set state to follow line

  // Step 3: Align perfectly
  aturnLeft(); // Turn left with adjusted speed
  delay(1500); // Wait for 1.5 seconds (adjust for alignment)
  Stop(); // Stop the Car
  delay(500); // Wait for 0.5 seconds
  currentState = FOLLOW_LINE; // Set state to follow line

  forward(); // Move forward
  delay(1000); // Wait for 1 second (adjust for parking)
  Stop(); // Stop the Car
  delay(500); // Wait for 0.5 seconds
  currentState = FOLLOW_LINE; // Set state to follow line

  // Ensure the vehicle stays stopped
  Stop(); // Stop the Car
  delay(100000); // Wait indefinitely

  // Check if the red color value is greater than green and blue
  if (r > g && r > b && g > b) {
    avoidObstacle(); // Avoid obstacle if red is detected
  }
}

// Function to process red color value from sensor
int process_red_value() {
  digitalWrite(S2_PIN, LOW); // Set S2 pin LOW
  digitalWrite(S3_PIN, LOW); // Set S3 pin LOW
  delay(100); // Allow sensor to stabilize for 100 milliseconds
  int pulse_length = pulseIn(OUT_PIN, LOW); // Measure pulse length while OUT_PIN is LOW
  return pulse_length; // Return the measured pulse length
}

// Function to process green color value from sensor
int process_green_value() {
  digitalWrite(S2_PIN, HIGH); // Set S2 pin HIGH
  digitalWrite(S3_PIN, HIGH); // Set S3 pin HIGH
  delay(100); // Allow sensor to stabilize for 100 milliseconds
  int pulse_length = pulseIn(OUT_PIN, LOW); // Measure pulse length while OUT_PIN is LOW
  return pulse_length; // Return the measured pulse length
}

// Function to process blue color value from sensor
int process_blue_value() {
  digitalWrite(S2_PIN, LOW); // Set S2 pin LOW
  digitalWrite(S3_PIN, HIGH); // Set S3 pin HIGH
  delay(100); // Allow sensor to stabilize for 100 milliseconds
  int pulse_length = pulseIn(OUT_PIN, LOW); // Measure pulse length while OUT_PIN is LOW
  return pulse_length; // Return the measured pulse length
}

// Function to accelerate the motor
void accelerateMotor() {
  if (motorSpeed < maxSpeed) { // If motor speed is less than the maximum speed
    motorSpeed += accelRate; // Increase motor speed by acceleration rate
    if (motorSpeed > maxSpeed) { // If motor speed exceeds the maximum speed
      motorSpeed = maxSpeed; // Set motor speed to maximum speed
    }
    delay(100); // Wait for 100 milliseconds
  }
}

// Function to decelerate the motor
void decelerateMotor() {
  if (motorSpeed > minSpeed) { // If motor speed is greater than the minimum speed
    motorSpeed -= decelRate; // Decrease motor speed by deceleration rate
    if (motorSpeed < minSpeed) { // If motor speed is less than the minimum speed
      motorSpeed = minSpeed; // Set motor speed to minimum speed
    }
    delay(10); // Wait for 10 milliseconds
  }
}

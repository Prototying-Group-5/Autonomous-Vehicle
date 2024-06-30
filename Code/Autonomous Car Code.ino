#define enA 10 // Enable1 L293 Pin enA 
#define in1 6 // Motor1 L293 Pin in1 
#define in2 7 // Motor1 L293 Pin in2 
#define in3 12 // Motor2 L293 Pin in3 
#define in4 8 // Motor2 L293 Pin in4 
#define enB 9 // Enable2 L293 Pin enB 
#define S0_PIN A0
#define S1_PIN A1
#define S2_PIN A2
#define S3_PIN A3
#define OUT_PIN A4
#define R_S 5 // IR sensor Right
#define L_S 4 // IR sensor Left

int trigPin = 2;    // TRIG pin
int echoPin = 3;    // ECHO pin
float duration, distance;

const int maxSpeed = 90; // Maximum motor speed
const int minSpeed = 82;  // Minimum motor speed
const int accelRate = 1;  // Speed increment per step
const int decelRate = 3;  // Speed decrement per step

int motorSpeed = 80;  // Current motor speed
enum State { FOLLOW_LINE, AVOID_OBSTACLE, PARK, STOPPED };
State currentState = FOLLOW_LINE;

void setup() { 
  Serial.begin(9600);
  // Set the S0, S1, S2, S3 Pins as Output
  pinMode(S0_PIN, OUTPUT);
  pinMode(S1_PIN, OUTPUT);
  pinMode(S2_PIN, OUTPUT);
  pinMode(S3_PIN, OUTPUT);
  // Set OUT_PIN as Input
  pinMode(OUT_PIN, INPUT);
  // Set Pulse Width scaling to 20%
  digitalWrite(S0_PIN, HIGH);
  digitalWrite(S1_PIN, LOW);
  // Enable UART for Debugging
  pinMode(R_S, INPUT); 
  pinMode(L_S, INPUT); 
  pinMode(enA, OUTPUT); 
  pinMode(in1, OUTPUT); 
  pinMode(in2, OUTPUT); 
  pinMode(in3, OUTPUT); 
  pinMode(in4, OUTPUT); 
  pinMode(enB, OUTPUT);
  pinMode(trigPin, OUTPUT); // configure the trigger pin to output mode
  pinMode(echoPin, INPUT); // configure the echo pin to input mode
}

void loop() { 

  ultrasonic();
  int b=process_blue_value();
  int g=process_green_value();
  int r = process_red_value();
 
  if (distance <7)  // 7 cm threshold for obstacle detection
   { 
    Stop(); 
    delay(1000);
  if(r < g && r < b && g > b){
    Serial.println("Colour Red");
    currentState = AVOID_OBSTACLE;
    
  } else if ( r > g && r > b && g > b){
    currentState =  PARK;
    Serial.println("Colour Blue");
  }
  else if ( g < r&&g < b&& r>b){
    currentState =  STOPPED;
    Serial.println("Colour Green");
  }
  else  {
    currentState = FOLLOW_LINE;  
  }
  }

  int rightSensor = digitalRead(R_S);
  int leftSensor = digitalRead(L_S);

  switch (currentState) {
    case FOLLOW_LINE:
      followLine(rightSensor, leftSensor);
      break;
    case AVOID_OBSTACLE:
      avoidObstacle();
      currentState = FOLLOW_LINE; // Return to line following after avoiding obstacle
      break;
    case PARK:
      Park();
      break;
    case STOPPED:
      Stop();
      break;
  }
}

void followLine(int rightSensor, int leftSensor) {
  switch (rightSensor << 1 | leftSensor) {
    case 0b11: // Both sensors on the line
      Stop();
      delay(1000);
      //turnRight();
      //delay(500);
     // turnLeft();
     // delay(1500);
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

void forward() { 
  digitalWrite(in1, LOW); // Right Motor forward Pin 
  digitalWrite(in2, HIGH);  // Right Motor backward Pin 
  digitalWrite(in3, HIGH);  // Left Motor forward Pin (Reversed logic) 
  digitalWrite(in4, LOW); // Left Motor backward Pin (Reversed logic)
   analogWrite(enA, motorSpeed);
  analogWrite(enB, motorSpeed);
}

void backward() { 
  digitalWrite(in1, HIGH); // Right Motor forward Pin 
  digitalWrite(in2, LOW);  // Right Motor backward Pin 
  digitalWrite(in3, LOW);  // Left Motor forward Pin (Reversed logic) 
  digitalWrite(in4, HIGH); // Left Motor backward Pin (Reversed logic)
  analogWrite(enA, 96);
  analogWrite(enB, 95);
}

void turnRight() { 
  digitalWrite(in1, LOW); // Right Motor forward Pin 
  digitalWrite(in2, HIGH);  // Right Motor backward Pin  
  digitalWrite(in3, LOW);  // Left Motor backward Pin (Reversed logic)
  digitalWrite(in4, LOW);  // Left Motor forward Pin (Reversed logic)
  analogWrite(enA, motorSpeed);
  analogWrite(enB, motorSpeed);
}
void aturnRight() { 
  digitalWrite(in1, LOW); // Right Motor forward Pin 
  digitalWrite(in2, HIGH);  // Right Motor backward Pin  
  digitalWrite(in3, LOW);  // Left Motor backward Pin (Reversed logic)
  digitalWrite(in4, LOW);  // Left Motor forward Pin (Reversed logic)
   analogWrite(enA, 115);
    analogWrite(enB, 0);
}

void turnLeft() { 
  digitalWrite(in1, LOW);  // Right Motor forward Pin 
  digitalWrite(in2, LOW);  // Right Motor backward Pin 
  digitalWrite(in3, HIGH);  // Left Motor forward Pin (Reversed logic)
  digitalWrite(in4, LOW); // Left Motor backward Pin (Reversed logic)
 analogWrite(enA, motorSpeed);
  analogWrite(enB, motorSpeed);
}
void aturnLeft() { 
  digitalWrite(in1, LOW);  // Right Motor forward Pin 
  digitalWrite(in2, LOW);  // Right Motor backward Pin 
  digitalWrite(in3, HIGH);  // Left Motor forward Pin (Reversed logic)
  digitalWrite(in4, LOW); // Left Motor backward Pin (Reversed logic)
  analogWrite(enB, 120);
   analogWrite(enA, 0);
}

void Stop() { 
  digitalWrite(in1, LOW);  // Right Motor forward Pin 
  digitalWrite(in2, LOW);  // Right Motor backward Pin 
  digitalWrite(in3, LOW);  // Left Motor forward Pin 
  digitalWrite(in4, LOW);  // Left Motor backward Pin 
}

void ultrasonic() {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  duration = pulseIn(echoPin, HIGH);
  distance = duration * 0.034 / 2;
}

void avoidObstacle() {
  // Step 1: Stop and move backward to avoid the obstacle
  
  backward();
  delay(1000); // Adjust based on the distance to move backward
  Stop();   
  delay(500);

  // Step 2: Turn left to start avoiding the obstacle
  aturnLeft();
  delay(1200); // Adjust this delay to control the turn angle
 currentState = FOLLOW_LINE;

  // Step 3: Move forward past the obstacle
  forward();
  delay(700); // Adjust this delay based on obstacle size
  
currentState = FOLLOW_LINE;
  // Step 4: Turn right to realign with the path
  aturnRight();
  delay(1700);
  currentState = FOLLOW_LINE;
  forward();
  delay(300);
currentState = FOLLOW_LINE;
  //aturnRight();
  delay(300); // Adjust this delay to control the turn angle
  
}

void Park() {
   int b=process_blue_value();
  int g=process_green_value();
  int r = process_red_value();
  backward();
  delay(1000);  // Short delay to move a bit back
  Stop();
  delay(500);
currentState = FOLLOW_LINE;  
  // Step 3: Align perfectly
  aturnLeft();
  delay(1500);  // Adjust the delay to ensure correct alignment
  Stop();
  delay(500);
currentState = FOLLOW_LINE;  
  forward();
  delay(1000);  // Adjust the delay to move into the perfect spot
  Stop();
  delay(500);
currentState = FOLLOW_LINE;  
  // Ensure the vehicle stays stopped
  Stop();
  delay(100000);
  
  if (r > r > g && r > b && g > b) {
    avoidObstacle();
  }
}

int process_red_value() {
  digitalWrite(S2_PIN, LOW);
  digitalWrite(S3_PIN, LOW);
  delay(100); // Allow sensor to stabilize
  int pulse_length = pulseIn(OUT_PIN, LOW);
  return pulse_length;
}

int process_green_value()
{
  digitalWrite(S2_PIN, HIGH);
  digitalWrite(S3_PIN, HIGH);
  delay(100); // Allow sensor to stabilize
  int pulse_length = pulseIn(OUT_PIN, LOW);
  return pulse_length;
}

int process_blue_value()
{
  digitalWrite(S2_PIN, LOW);
  digitalWrite(S3_PIN, HIGH);
  delay(100); // Allow sensor to stabilize
  int pulse_length = pulseIn(OUT_PIN, LOW);
  return pulse_length;
}
void accelerateMotor() {
  if (motorSpeed < maxSpeed) {
    motorSpeed += accelRate;
    if (motorSpeed > maxSpeed) {
      motorSpeed = maxSpeed;
    }
      delay(100);
  }
}

void decelerateMotor() {
  if (motorSpeed > minSpeed) {
    motorSpeed -= decelRate;
    if (motorSpeed < minSpeed) {
      motorSpeed = minSpeed;
    }
      delay(10);
  }
}


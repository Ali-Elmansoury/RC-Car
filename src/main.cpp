#include <Arduino.h>
#define BLYNK_PRINT Serial
#define BLYNK_TEMPLATE_ID ""           //Replace with your Blynk Template ID
#define BLYNK_TEMPLATE_NAME "RC Car"   //Replace with your Blynk Template Name
#define BLYNK_AUTH_TOKEN ""            //Replace with your Blynk Auth Token
#include <ESP8266WiFi.h>
#include <BlynkSimpleEsp8266.h>
#include <Servo.h>

// Your Wi-Fi credentials
char ssid[] = "";        // Replace with your Wi-Fi SSID
char pass[] = "";  // Replace with your Wi-Fi password

void stopMotors();

int turnSpeedObstacle = 100;
int turnSpeedLine = 80;

// Motor control pins
#define IN1 D0  // Left motor control
#define IN2 D1
#define ENA D6  // Left motor speed (PWM)
#define IN3 D3  // Right motor control
#define IN4 D4
#define ENB D5  // Right motor speed (PWM)

// Ultrasonic sensor pins
#define TRIG_F D7  // Front sensor trigger pin
#define ECHO_F D8  // Front sensor echo pin

int mode = 0; // Variable for the selected mode

//define sound velocity in cm/uS
#define SOUND_VELOCITY 0.034

long duration;
float distanceCm;
float distanceInch;

#define SERVO_PIN D2
Servo frontServo;
int angle = 0;

#define LineFollower_R D9
#define LineFollower_L D10

void setup() {
  // Initialize Serial Monitor
  Serial.begin(9600);

  frontServo.attach(SERVO_PIN,500,2400);  // Attach servo
  frontServo.write(90);  // Set initial position to center
  
  // Set motor pins as output
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENB, OUTPUT);

  // Set ultrasonic pins as output/input
  pinMode(TRIG_F, OUTPUT);
  pinMode(ECHO_F, INPUT);

  // Set line follower pins as input
  pinMode(LineFollower_R, INPUT);
  pinMode(LineFollower_L, INPUT);

  // Stop motors at startup
  stopMotors();

  // Connect to Blynk
  Blynk.begin(BLYNK_AUTH_TOKEN, ssid, pass);
  Blynk.virtualWrite(V5, "Connected to Wif");
  Blynk.virtualWrite(V6, "i");
}

// Function to stop the motors
void stopMotors() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(ENA, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  digitalWrite(ENB, LOW);
}

// Function to move forward
void moveForward() {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(ENA, HIGH);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  digitalWrite(ENB, HIGH);
}

// Function to move backward
void moveBackward() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(ENA, HIGH);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  digitalWrite(ENB, HIGH);
}

// Function to turn right
void turnRight() {
  digitalWrite(IN1, HIGH);  // Left wheels forward
  digitalWrite(IN2, LOW);
  digitalWrite(ENA, HIGH);
  digitalWrite(IN3, LOW);   // Right wheels backward
  digitalWrite(IN4, HIGH);
  digitalWrite(ENB, HIGH);
}

// Function to turn left
void turnLeft() {
  digitalWrite(IN1, LOW);   // Left wheels backward
  digitalWrite(IN2, HIGH);
  digitalWrite(ENA, HIGH);
  digitalWrite(IN3, HIGH);  // Right wheels forward
  digitalWrite(IN4, LOW);
  digitalWrite(ENB, HIGH);
}

void moveForwardWithPWM(int speed) {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  analogWrite(ENA, speed);  // Control speed of left motor
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  analogWrite(ENB, speed);  // Control speed of right motor
}

// Function to move backward with PWM
void moveBackwardWithPWM(int speed) {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  analogWrite(ENA, speed);  // Control speed of left motor
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  analogWrite(ENB, speed);  // Control speed of right motor
}

// Function to turn right with PWM
void turnRightWithPWM(int speed) {
  digitalWrite(IN1, HIGH);  // Left wheels forward
  digitalWrite(IN2, LOW);
  analogWrite(ENA, speed);  // Control speed of left motor
  digitalWrite(IN3, LOW);   // Right wheels backward
  digitalWrite(IN4, HIGH);
  analogWrite(ENB, speed);  // Control speed of right motor
}

// Function to turn left with PWM
void turnLeftWithPWM(int speed) {
  digitalWrite(IN1, LOW);   // Left wheels backward
  digitalWrite(IN2, HIGH);
  analogWrite(ENA, speed);  // Control speed of left motor
  digitalWrite(IN3, HIGH);  // Right wheels forward
  digitalWrite(IN4, LOW);
  analogWrite(ENB, speed);  // Control speed of right motor
}

// Function to measure distance from an ultrasonic sensor
long measureDistance(int trigPin, int echoPin) {

  // Clears the trigPin
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  // Reads the echoPin, returns the sound wave travel time in microseconds
  duration = pulseIn(echoPin, HIGH);
  
  // Calculate the distance
  distanceCm = duration * SOUND_VELOCITY/2;

  return distanceCm;
}

// Obstacle evasion logic
void obstacleEvasion() {

  // Check distances in three directions by rotating the servo
  frontServo.write(45);  // Move servo to the left
  delay(500);  // Wait for the sensor to stabilize
  long distLeft = measureDistance(TRIG_F, ECHO_F);

  frontServo.write(90);  // Move servo to the center
  delay(500);
  long distFront = measureDistance(TRIG_F, ECHO_F);

  frontServo.write(135);  // Move servo to the right
  delay(500);
  long distRight = measureDistance(TRIG_F, ECHO_F);

  frontServo.write(90);  // Return servo to center

  // Obstacle avoidance logic based on readings
  if (distFront < 35) {
    if (distLeft > 35 && distRight > 35) {
      if (distLeft > distRight) {
        turnLeftWithPWM(turnSpeedObstacle);
      } else {
        turnRightWithPWM(turnSpeedObstacle);
      }
    } else if (distLeft > 35) {
      turnLeftWithPWM(turnSpeedObstacle);
    } else if (distRight > 35) {
      turnRightWithPWM(turnSpeedObstacle);
    } else {
      moveBackwardWithPWM(turnSpeedObstacle);
    }
    delay(500);  // Adjust as needed
  } else {
    moveForwardWithPWM(turnSpeedObstacle);
  }

  // Periodically check for mode change
  Blynk.run();  // Allow Blynk to process any updates (including mode change)
  if (mode != 1) {
    stopMotors();  // Stop motors if mode changes
    return;  // Exit the function if the mode changes
  }
}

// Line following logic
void lineFollowing() {
  // Line following logic based on sensor readings

  // Read line follower sensors
  int left = digitalRead(LineFollower_L);
  int right = digitalRead(LineFollower_R);
  if (left == LOW && right == LOW) {
    moveForwardWithPWM(turnSpeedLine);
  } else if (left == HIGH && right == LOW) {
    turnRightWithPWM(turnSpeedLine);
  } else if (left == LOW && right == HIGH) {
    turnLeftWithPWM(turnSpeedLine);
  } else {
    stopMotors();
  }
  // Periodically check for mode change
  Blynk.run();  // Allow Blynk to process any updates (including mode change)
  if (mode != 2) {
    stopMotors();  // Stop motors if mode changes
    return;  // Exit the function if the mode changes
  }
}

// Maze solving logic (right-wall following)
void mazeSolving() {
  // Rotate the servo to check distances in three directions
  frontServo.write(45);  // Move servo to the left
  delay(500);
  long distLeft = measureDistance(TRIG_F, ECHO_F);

  frontServo.write(90);  // Move servo to the front
  delay(500);
  long distFront = measureDistance(TRIG_F, ECHO_F);

  frontServo.write(135);  // Move servo to the right
  delay(500);
  long distRight = measureDistance(TRIG_F, ECHO_F);

  frontServo.write(90);  // Return servo to the center position

  // Right-wall following logic
  if (distRight > 35 && distFront > 35) {
    // If there's no wall on the right and front, turn right
    turnRightWithPWM(turnSpeedObstacle);
    delay(300);  // Small delay for smooth turning
  } else if (distFront > 35) {
    // If only the front is clear, move forward
    moveForwardWithPWM(turnSpeedObstacle);
  } else if (distLeft > 35) {
    // If there's a wall in front but no wall on the left, turn left
    turnLeftWithPWM(turnSpeedObstacle);
    delay(300);  // Small delay for smooth turning
  } else {
    // If walls are on all sides, move backward slightly and re-evaluate
    moveBackwardWithPWM(turnSpeedObstacle);
    delay(300);
  }

  // Periodically check for mode change
  Blynk.run();  // Allow Blynk to process any updates (including mode change)
  if (mode != 3) {
    stopMotors();  // Stop motors if mode changes
    return;  // Exit the function if the mode changes
  }
}


// Blynk button V0 to control forward movement
BLYNK_WRITE(V0) {
  int value = param.asInt(); // Get button state
  
  if (mode == 0 && value == 1) {  // Button pressed, only works in mobile control mode (mode 0)
    moveForward();
    Serial.println("Moving Forward");
    Blynk.virtualWrite(V5, " ");
    Blynk.virtualWrite(V6, " ");
    Blynk.virtualWrite(V5, "Moving Forward");
  } else {  // Button released
    stopMotors();
    Serial.println("Stopped");
    Blynk.virtualWrite(V5, " ");
    Blynk.virtualWrite(V6, " ");
    Blynk.virtualWrite(V5, "Stopped");
  }
}

// Blynk button V1 to control backward movement
BLYNK_WRITE(V1) {
  int value = param.asInt(); // Get button state
  
  if (mode == 0 && value == 1) {  // Button pressed, only works in mobile control mode (mode 0)
    moveBackward();
    Serial.println("Moving Backward");
    Blynk.virtualWrite(V5, " ");
    Blynk.virtualWrite(V6, " ");
    Blynk.virtualWrite(V5, "Moving Backward");
  } else {  // Button released
    stopMotors();
    Serial.println("Stopped");
    Blynk.virtualWrite(V5, " ");
    Blynk.virtualWrite(V6, " ");
    Blynk.virtualWrite(V5, "Stopped");
  }
}

// Blynk button V2 to control right turn
BLYNK_WRITE(V2) {
  int value = param.asInt(); // Get button state
  
  if (mode == 0 && value == 1) {  // Button pressed, only works in mobile control mode (mode 0)
    turnRight();
    Serial.println("Turning Right");
    Blynk.virtualWrite(V5, " ");
    Blynk.virtualWrite(V6, " ");
    Blynk.virtualWrite(V5, "Turning Right");
  } else {  // Button released
    stopMotors();
    Serial.println("Stopped");
    Blynk.virtualWrite(V5, " ");
    Blynk.virtualWrite(V6, " ");
    Blynk.virtualWrite(V5, "Stopped");
  }
}

// Blynk button V3 to control left turn
BLYNK_WRITE(V3) {
  int value = param.asInt(); // Get button state
  
  if (mode == 0 && value == 1) {  // Button pressed, only works in mobile control mode (mode 0)
    turnLeft();
    Serial.println("Turning Left");
    Blynk.virtualWrite(V5, " ");
    Blynk.virtualWrite(V6, " ");
    Blynk.virtualWrite(V5, "Turning Left");
  } else {  // Button released
    stopMotors();
    Serial.println("Stopped");
    Blynk.virtualWrite(V5, " ");
    Blynk.virtualWrite(V6, " ");
    Blynk.virtualWrite(V5, "Stopped");
  }
}

// Function to switch modes based on slider value
BLYNK_WRITE(V4) {
  mode = param.asInt(); // Get slider value (0 to 3)
  
  if (mode == 0) {
    Serial.println("Mode 0: Mobile Control");
    Blynk.virtualWrite(V5, " ");
    Blynk.virtualWrite(V6, " ");
    Blynk.virtualWrite(V5, "Mode 0: Mobile C");
    Blynk.virtualWrite(V6, "ontrol");
  } else if (mode == 1) {
    Serial.println("Mode 1: Obstacle Evasion");
    Blynk.virtualWrite(V5, " ");
    Blynk.virtualWrite(V6, " ");
    Blynk.virtualWrite(V5, "Mode 1: Obstacle");
    Blynk.virtualWrite(V6, "Evasion");
  } else if (mode == 3) {
    Serial.println("Mode 3: Shortest Path in Maze");
    Blynk.virtualWrite(V5, " ");
    Blynk.virtualWrite(V6, " ");
    Blynk.virtualWrite(V5, "Mode 3: Shortest");
    Blynk.virtualWrite(V6, "Path in Maze");
  } else if (mode == 2) {
    Serial.println("Mode 2: Line Following");
    Blynk.virtualWrite(V5, " ");
    Blynk.virtualWrite(V6, " ");
    Blynk.virtualWrite(V5, "Mode 2: Line Fol");
    Blynk.virtualWrite(V6, "lowing");
  }
}

// Function to read the numeric input from Blynk on V7
BLYNK_WRITE(V7) {
  angle = param.asInt(); // Get the value from the Numeric Input
  Serial.print("Received Servo Angle: ");
  Serial.println(angle);

  // Use this value as needed in your program, e.g., adjust speed or distance
}

// Function to read the numeric input from Blynk on V8
BLYNK_WRITE(V8) {
  turnSpeedObstacle = param.asInt(); // Get the value from the Numeric Input
  Serial.print("Received Obstacle TurnSpeed: ");
  Serial.println(turnSpeedObstacle);

  // Use this value as needed in your program, e.g., adjust speed or distance
}

// Function to read the numeric input from Blynk on V9
BLYNK_WRITE(V9) {
  turnSpeedLine = param.asInt(); // Get the value from the Numeric Input
  Serial.print("Received LineFollower TurnSpeed: ");
  Serial.println(turnSpeedLine);

  // Use this value as needed in your program, e.g., adjust speed or distance
}

void loop() {
  Blynk.run();  // Runs the Blynk interface
  if (mode == 1) {  // If in obstacle evasion mode
    obstacleEvasion();  // Continuously check for obstacles
  }
  if (mode == 2) {  // If in line following mode
    lineFollowing();  // Continuously follow the line
  }
  if (mode == 3) {  // If in maze solving mode
    mazeSolving();  // Continuously solve the maze
                    // (right-wall following), Not tested
  }
  frontServo.write(angle);  // Move servo to the specified angle
}


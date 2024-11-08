
# RC Car Project - ITI 9 Months Professional Training (Embedded Systems Track)

This project demonstrates the development of an RC Car with multiple operation modes, developed as part of the ITI 9 Months Professional Training in the Embedded Systems Track. The RC Car can be remotely controlled via a mobile app (Blynk) or operate autonomously in various modes such as obstacle evasion, line following, and maze solving.

## Project Description

The RC Car supports the following modes:
1. **Mobile Control**: Control the car's movement via Blynk app buttons.
2. **Obstacle Evasion**: Uses ultrasonic sensors to detect and avoid obstacles.
3. **Line Following**: Follows a line on the ground using IR sensors.
4. **Maze Solving**: Navigates a maze using right-wall following algorithm.

Mode selection is done via a Blynk slider, where each mode is represented by a value:
- **0**: Mobile Control
- **1**: Obstacle Evasion
- **2**: Line Following
- **3**: Maze Solving

## Components Used

- **NodeMCU (ESP8266)**: Main controller to interface with the Blynk app and handle car operations.
- **L298N Motor Driver**: Controls the DC motors for car movement.
- **DC Motors**: Four motors (each pair connected in series per side) for driving the car.
- **Servo Motor**: Used to control the front ultrasonic sensor's rotation for obstacle detection.
- **Ultrasonic Sensor**: One sensor mounted on Servo Motor to detect obstacles for autonomous navigation.
- **TCRT5000 IR Sensors**: Two sensors to enable line following functionality.
- **Blynk App**: Mobile app for remote control and mode selection, providing a user-friendly interface.

## Setup and Usage

1. Connect the components as per the wiring diagram.
2. Replace the `BLYNK_AUTH_TOKEN`, `ssid`, and `pass` with your Blynk authentication token, Wi-Fi SSID, and password respectively.
3. Upload the code to the NodeMCU using Arduino IDE.
4. Open the Blynk app and use the provided controls to operate the RC Car.
   - **Slider** on Virtual Pin V4 for mode selection.
   - **Buttons** on Virtual Pins V0, V1, V2, and V3 for forward, backward, left, and right movement (only in Mobile Control mode).

## Code Summary

The code initializes the hardware components, establishes a connection with the Blynk app, and controls the RC Car based on the selected mode. Key functionalities include:
- **Motor Control**: Functions to move forward, backward, and turn using PWM control.
- **Distance Measurement**: Ultrasonic sensor measures the distance to obstacles.
- **Obstacle Avoidance**: Logic to evade obstacles by comparing distances from left, right, and front.
- **Line Following**: Uses IR sensors to stay on a line.
- **Maze Solving**: Right-wall following algorithm to navigate through a maze.

## Important Pins

- **Motor Pins**: IN1, IN2, ENA, IN3, IN4, ENB (Control left and right motor directions and speeds).
- **Ultrasonic Sensor Pins**: TRIG_F (trigger) and ECHO_F (echo) for the front sensor.
- **Servo Pin**: Controls servo motor for ultrasonic sensor rotation.
- **Line Follower Pins**: LineFollower_R and LineFollower_L for right and left IR sensors.

## License

This project is for educational purposes only and is not intended for commercial use.

---

*Developed by Ali Mohamed Ali Elmansoury as part of the ITI Embedded Systems Track.*

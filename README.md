# UIURC_ADVANCE_LFR
## Code and instruction!!! Read every line

# Line Following Robot with PID Control

This project controls a line-following robot using PID (Proportional-Integral-Derivative) control. It reads data from IR sensors and adjusts the speed of two motors to keep the robot on the track.

## Components:
- Arduino
- Motor Driver (TB6612FNG or L298N)
- IR Sensors (5 total)
- Two Motors
- Two Push Buttons (connected to pin 11 and 12)
- Built-in LED (pin 13)

## Pin Connections:
- **Motor Pins:**
  - IN1: Pin 6
  - IN2: Pin 5
  - IN3: Pin 4
  - IN4: Pin 2
  - enA: Pin 9 (PWM for Motor A)
  - enB: Pin 3 (PWM for Motor B)
- **Sensor Pins:**
  - Analog inputs (A0 to A4 for 5 IR sensors)
- **Push Buttons:**
  - Button 1: Pin 11 (start motor)
  - Button 2: Pin 12 (stop motor)
- **LED Pin:**
  - Built-in Arduino LED: Pin 13

## How to Adjust `kp` and `kd` Values:
- `kp` (Proportional Gain) and `kd` (Derivative Gain) determine how sensitive your robot is to sensor input and how quickly it reacts to errors.
- **Higher `kp` values**: Increase responsiveness, but the robot might become unstable (oscillate).
- **Lower `kp` values**: Make the robot slower but more stable.
- **Higher `kd` values**: Reduce overshooting and make movements smoother, but too high values can cause sluggishness.
  
### Steps to adjust `kp` and `kd`:
1. **Start with a low `kp` value** (e.g., 40) and a higher `kd` value (e.g., 1000).
2. Run the robot and observe how it follows the line.
   - If it’s **too slow or sluggish**, increase `kp`.
   - If it’s **oscillating too much**, reduce `kp` or increase `kd`.
3. Tweak these values until the robot follows the line smoothly and efficiently.

## Usage:
1. **Power on the robot.**
2. Press **Button 1** (pin 11) to start the motors and begin line following.
3. The built-in LED will blink rapidly while the robot is in line-following mode.
4. Press **Button 2** (pin 12) to stop the motors and pause line following.

### Code Overview:
- The code uses 5 IR sensors to read the position of the line and calculate an average sensor position.
- It then uses the **PID control algorithm** to adjust the speed of the left and right motors, ensuring the robot stays on the line.

Function: Stops the line-following operation when pressed.

# Built-in LED:
LED → Built-in LED (usually Pin 13 on the Arduino)
Function: Rapid blinking during the line-following setup, solid when the line-following is active.

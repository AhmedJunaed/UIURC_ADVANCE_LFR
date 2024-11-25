# Line Following Robot with PID Control (5 IR Sensors)

This repository contains three versions of code for controlling a line-following robot using **PID control**. Each version offers a different level of complexity and optimization.

- **Version 1**: Beginner-friendly with simple explanations and basic PID control.
- **Version 2**: Intermediate with optimized PID control and improved performance.
- **Version 3**: Advanced with additional features such as line-loss recovery, optimized PID control, and better stability.

---

## Table of Contents

- [Code Overview](#code-overview)
- [Kp and Kd Tuning](#kp-and-kd-tuning)
- [How to Use](#how-to-use)
  - [Hardware Setup](#hardware-setup)
  - [Upload Code to Arduino](#upload-code-to-arduino)
  - [Operating the Robot](#operating-the-robot)
  - [Maintenance](#maintenance)
- [Troubleshooting Tips](#troubleshooting-tips)
- [Contributing](#contributing)
- [License](#license)

---

## Code Overview

### **Version 1 (Beginner Friendly)**
- Designed with simple explanations and comments, making it easier for beginners to understand.
- Implements basic PID control for line-following.
- Ideal for users just starting with robotics and PID control.

### **Version 2 (Intermediate)**
- Optimized for performance with improved PID control.
- Includes additional logic for better line-following performance.
- Recommended for users who are comfortable with PID control and want to experiment with robotics features.

### **Version 3 (Advanced)**
- Includes advanced features like line-loss recovery and more refined PID control.
- Offers better performance and additional stability improvements.
- Recommended for users seeking maximum performance with line-following robots.

---

## Kp and Kd Tuning

**Kp** (Proportional) and **Kd** (Derivative) are two critical parameters in the PID control algorithm that influence how your robot follows the line.

### **What is Kp?**
- **Kp (Proportional Gain)**: Determines how aggressively the robot reacts to deviations from the line. 
  - Higher **Kp** values make the robot respond more quickly but may cause overshooting.
  - Lower **Kp** values make the robot respond more slowly but more steadily.

### **What is Kd?**
- **Kd (Derivative Gain)**: Helps smooth out the robot's motion by predicting future errors based on the rate of change of the error.
  - Higher **Kd** values reduce oscillation and smoothen the robot's movement.
  - Lower **Kd** values may cause the robot to be jerky or oscillate excessively.

### **How to Adjust Kp and Kd**
You will need to manually adjust the values of **Kp** and **Kd** to get the best performance for your line-following robot. The default values are set in the code, but you may need to modify them based on your environment.

#### **Steps to Tune Kp and Kd**:
1. **Start with default values**:
   - Kp = 50
   - Kd = 500
2. **Test the robot on the line**:
   - Upload the code and observe how the robot behaves on the track.
3. **Adjust Kp**:
   - If the robot is slow to respond or drifts off the line, increase **Kp**.
   - If the robot overshoots or becomes unstable, decrease **Kp**.
4. **Adjust Kd**:
   - If the robot oscillates or makes jerky movements, increase **Kd** to smoothen it out.
   - If the robot is too slow to correct itself, decrease **Kd** slightly.
5. **Iterate and test**:
   - Continue adjusting these values incrementally until the robot follows the line smoothly.

### **Important:**
The tuning values of **Kp** and **Kd** depend on various factors such as:
- The speed of the motors
- The surface on which the robot is running
- The lighting conditions affecting the IR sensors
- The distance between sensors and the surface

Regularly test and adjust **Kp** and **Kd** whenever there is a change in any of these conditions.

---

## How to Use

### **Hardware Setup**

1. **Pinout Diagram**:

   | Component          | Arduino Pin  |
   |--------------------|--------------|
   | Motor A PWM        | 3            |
   | Motor A Direction  | 4 (AIN1), 5 (AIN2) |
   | Motor B PWM        | 6            |
   | Motor B Direction  | 7 (BIN1), 8 (BIN2) |
   | IR Sensor (QTR-8A) | A0 to A7     |
   | Button 1 (Start)   | 11           |
   | Button 2 (Stop)    | 12           |
   | Built-in LED       | 13           |

2. Connect the motors, IR sensors, and buttons to the appropriate pins on the Arduino board.
3. Ensure the motors and sensors are powered correctly.

### **Upload Code to Arduino**

1. Choose either **Code Version 1** (Beginner Friendly), **Code Version 2** (Intermediate), or **Code Version 3** (Advanced) in the Arduino IDE.
2. Connect the Arduino board to your computer and upload the selected code.

### **Operating the Robot**

- **Button 1**: Start the line-following mode. When pressed, the robot will start following the line using the PID control algorithm.
- **Button 2**: Stop the robot. When pressed, the robot will stop following the line and halt its motors.
- **Built-in LED**: The LED will blink rapidly while the robot is following the line. When the line-following stops, the LED will turn on solid.

### **Maintenance**
- Ensure the IR sensors are clean and free from obstructions.
- Regularly check the motors and wheels for wear and tear.
- If the robot starts veering off track or not responding well, adjust the **Kp** and **Kd** values, as environmental factors like surface color or sensor positioning can affect the robot’s performance.

---

## Troubleshooting Tips

- **Robot not following the line properly**: Try adjusting the **Kp** and **Kd** values.
- **Robot moves too jerky**: Lower the **Kp** or increase **Kd** to smoothen out the movement.
- **Robot veers off track**: Check the sensor alignment and clean the IR sensors if needed.

---

## Contributing

Feel free to contribute by submitting issues or pull requests. Improvements to the code, such as better PID tuning or sensor calibration, are always welcome!

---

## License

This project is open-source and licensed under the MIT License.


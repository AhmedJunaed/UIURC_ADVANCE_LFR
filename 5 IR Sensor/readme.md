# Read Every line to uderstand the code



# Line Following Robot with PID Control with 5 IR Sensor

This repository contains two versions of the code for controlling a line-following robot. Both codes implement PID control for line-following, but **Code 1** is more beginner-friendly, while **Code 2** is more advanced with additional features and optimizations.

---

## Code Overview

### **Code 1 (Beginner Friendly)**
- This code is designed with simple explanations and comments, making it easier for beginners to understand.
- It uses basic PID control for line-following with a clear structure.
- It is ideal for beginners who are just starting with robotics and PID control.

### **Code 2 (Advanced)**
- This version is optimized for performance and includes more advanced features.
- It may be more complex due to the use of advanced logic and additional functionalities.
- It is recommended for those who are comfortable with PID control and want to experiment with more advanced robotics features.

---

## Kp and Kd Tuning

**Kp** (Proportional) and **Kd** (Derivative) are two critical parameters in the PID control algorithm that influence how your robot follows the line. They determine how fast and smoothly the robot reacts to the line's position.

### **What is Kp?**
- **Kp (Proportional Gain)**: This determines how aggressively the robot reacts to being off the line. 
  - A higher **Kp** value makes the robot respond more quickly to deviations from the line but may cause it to overshoot.
  - A lower **Kp** value makes the robot respond more slowly but more steadily.
  
### **What is Kd?**
- **Kd (Derivative Gain)**: This helps smooth out the robot's motion by predicting future errors based on the rate of change of the error. 
  - A higher **Kd** value helps reduce oscillation and smoothens the robot's movement.
  - A lower **Kd** value may cause the robot to be jerky or oscillate too much.

### **How to Adjust Kp and Kd**
You will need to manually adjust the values of **Kp** and **Kd** to get the best performance for your line-following robot. The default values are set in the code, but you may need to modify them based on your environment.

#### **Steps to Tune Kp and Kd:**
1. **Start with default values**:
   - Kp = 50
   - Kd = 500
2. **Test the robot on the line**:
   - Upload the code and observe how the robot behaves on the line.
3. **Adjust Kp**:
   - If the robot is slow to respond or drifts off the line, increase **Kp**.
   - If the robot overshoots or becomes unstable, decrease **Kp**.
4. **Adjust Kd**:
   - If the robot oscillates or makes jerky movements, increase **Kd** to smoothen it out.
   - If the robot is too slow to correct itself, decrease **Kd** slightly.
5. **Iterate and test**:
   - Continue adjusting these values incrementally until the robot moves smoothly and follows the line accurately.

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

1. Connect the IR sensors, motors, and motor driver to the appropriate pins on the Arduino board as per the following pinout:

   **Pinout Diagram:**
   
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

2. Ensure that the motors and sensors are properly powered.

### **Upload Code to Arduino**
1. Choose either **Code 1 (beginner-friendly)** or **Code 2 (advanced)** in the Arduino IDE.
2. Connect the Arduino board to your computer and upload the chosen code.

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

### License
This project is open-source and licensed under the MIT License.

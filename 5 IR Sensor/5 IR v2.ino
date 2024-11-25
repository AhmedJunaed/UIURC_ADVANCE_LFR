// Define motor control pins
#define IN1 6  // Motor B (Right) forward
#define IN2 5  // Motor B (Right) backward
#define IN3 4  // Motor A (Left) forward
#define IN4 2  // Motor A (Left) backward
#define enA 9  // Motor A (Left) speed control (PWM)
#define enB 3  // Motor B (Right) speed control (PWM)

// Sensor variables for line-following
int s[6];                // Array to store sensor readings
int total, sensor_position;  // Variables to calculate sensor values
int threshold = 531;      // Threshold value to determine if sensor detects line
float avg;                // Average sensor value
int position[6] = {1, 2, 3, 4, 5};  // Positions for each sensor
int set_point = 3;        // Set point for the robot to follow the center of the line

// Push buttons and built-in LED
int start_button_pin = 11;  // Pin for the start button
int stop_button_pin = 12;   // Pin for the stop button
int led = 13;               // Arduino built-in LED
bool is_running = false;    // Flag to track if robot is running or stopped

void setup() {
  // Initialize motor control pins as OUTPUT
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  // Initialize buttons and LED pins
  pinMode(start_button_pin, INPUT_PULLUP);  // Start button with pull-up resistor
  pinMode(stop_button_pin, INPUT_PULLUP);   // Stop button with pull-up resistor
  pinMode(led, OUTPUT);  // Built-in LED for status

  // Start the Serial Monitor for debugging
  Serial.begin(9600);
}

void loop() {
  // Read the state of the start and stop buttons
  bool start_button_state = digitalRead(start_button_pin);
  bool stop_button_state = digitalRead(stop_button_pin);

  // If the start button is pressed, turn on the robot
  if (start_button_state == LOW) {
    is_running = true;  // Robot starts running
    digitalWrite(led, HIGH);  // Turn on the LED to indicate running
  }

  // If the stop button is pressed, turn off the robot
  if (stop_button_state == LOW) {
    is_running = false;  // Robot stops running
    digitalWrite(led, LOW);   // Turn off the LED to indicate stopped
    motor(0, 0);  // Stop both motors
  }

  // If the robot is running, perform line-following
  if (is_running) {
    display_sensor_values();  // Show sensor values in the Serial Monitor for debugging
    line_following_with_PID();  // Follow the line using PID control
  }
}

// Function to read the sensors and calculate average position
void read_sensors() {
  sensor_position = 0;
  total = 0;

  // Read values from 5 sensors (A0, A1, A2, A6, A7)
  for (byte i = 0; i < 5; i++) {
    if (i > 2) {
      s[i] = analogRead(i + 3);  // Read sensors A6 and A7
    } else {
      s[i] = analogRead(i);      // Read sensors A0, A1, A2
    }
    
    // Convert the sensor reading to either 1 (line detected) or 0 (no line)
    if (s[i] > threshold) s[i] = 1;
    else s[i] = 0;

    // Calculate the total position and number of sensors detecting the line
    sensor_position += s[i] * position[i];
    total += s[i];
  }

  // Calculate the average position of the detected line
  if (total) avg = sensor_position / total;
}

// Function to follow the line using PID (Proportional, Integral, Derivative) control
void line_following_with_PID() {
  // PID constants: adjust these for fine-tuning
  int kp = 50;  // Proportional gain (how much correction is applied based on current error)
  int kd = 500; // Derivative gain (how much correction is applied based on change in error)
  float error, previous_error = 0;  // Error tracking for line position
  int base_speed = 200;  // Base speed for motors
  int left_motor_speed, right_motor_speed;  // Speed for left and right motors

  // Read sensor data and calculate error to adjust motor speeds
  read_sensors();  

  // Calculate the error between the set point (center of the line) and the average sensor position
  error = set_point - avg;

  // Proportional term: adjust motor speed based on the current error
  int P = error * kp;

  // Derivative term: adjust motor speed based on the change in error
  int D = kd * (error - previous_error);

  // Calculate the PID value by combining P and D terms
  int PID_value = P + D;

  // Update the previous error for the next calculation
  previous_error = error;

  // Adjust motor speeds based on the PID value
  right_motor_speed = base_speed - PID_value;  // Right motor speed
  left_motor_speed = base_speed + PID_value;   // Left motor speed

  // Move the motors based on the calculated speeds
  motor(left_motor_speed, right_motor_speed);

  // Check if the robot is off the line and handle turns
  if (total == 0) {
    motor(0, 0);  // Stop motors if no sensors detect the line
  }
}

// Function to display sensor values in the Serial Monitor for debugging
void display_sensor_values() {
  for (byte i = 0; i < 5; i++) {
    if (i > 2) {
      s[i] = analogRead(i + 3);  // Read sensors A6 and A7
    } else {
      s[i] = analogRead(i);      // Read sensors A0, A1, A2
    }
    Serial.print(String(s[i]) + " ");  // Print the sensor value
  }
  Serial.println();
  delay(50);  // Delay for readability
}

// Function to control the motors based on speed
void motor(int a, int b) {
  // Motor A (Left) control
  if (a > 0) {
    digitalWrite(IN3, 1);
    digitalWrite(IN4, 0);
  } else {
    a = -(a);  // Make speed negative for reverse motion
    digitalWrite(IN3, 0);
    digitalWrite(IN4, 1);
  }

  // Motor B (Right) control
  if (b > 0) {
    digitalWrite(IN1, 1);
    digitalWrite(IN2, 0);
  } else {
    b = -(b);  // Make speed negative for reverse motion
    digitalWrite(IN1, 0);
    digitalWrite(IN2, 1);
  }

  // Limit motor speed to a maximum value to prevent overheating or erratic behavior
  if (a > 200) a = 200;
  if (b > 200) b = 200;

  // Set PWM speed for both motors (motor speed control)
  analogWrite(enA, a);
  analogWrite(enB, b);
}

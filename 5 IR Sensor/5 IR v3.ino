// Motor pin defines - Assigning Arduino pins to control the motor driver
#define IN1 6  // Motor B control pin 1
#define IN2 5  // Motor B control pin 2
#define IN3 4  // Motor A control pin 1
#define IN4 2  // Motor A control pin 2
#define enA 9  // Motor A PWM speed control pin
#define enB 3  // Motor B PWM speed control pin

// Variables for sensors and line-following settings
int s[6];                     // Array to store sensor readings
int total, sensor_position;    // Variables to store total sensor values and their position
int threshold = 531;           // Threshold to determine if the sensor detects a line
float avg;                     // Variable to store the average position of the detected line
int position[6] = {1, 2, 3, 4, 5};  // Array representing sensor positions (e.g., 1 to 5)
int set_point = 3;             // The target sensor position (center of the line)

// Pins for push buttons and LED
int start_button_pin = 11;  // Start button pin
int stop_button_pin = 12;   // Stop button pin
int led = 13;               // LED pin (usually built-in to Arduino)
bool is_running = false;    // Boolean to track if the robot is running or not

// PID control constants
int kp = 50;                // Proportional gain for PID control
int kd = 500;               // Derivative gain for PID control
int base_speed = 200;       // Base speed for motors
int turn_speed = 100;       // Speed for turning

void setup() {
  // Set motor driver pins as output
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  // Configure button pins as input with pull-up resistors to avoid floating values
  pinMode(start_button_pin, INPUT_PULLUP);
  pinMode(stop_button_pin, INPUT_PULLUP);

  // Set LED pin as output
  pinMode(led, OUTPUT);

  // Start the Serial Monitor for debugging
  Serial.begin(9600);
}

void loop() {
  // Read the state of the start and stop buttons
  bool start_button_state = digitalRead(start_button_pin);
  bool stop_button_state = digitalRead(stop_button_pin);

  // If the start button is pressed, begin robot operation
  if (start_button_state == LOW) {
    is_running = true;
    digitalWrite(led, HIGH);  // Turn on LED to indicate the robot is running
  }

  // If the stop button is pressed, stop the robot
  if (stop_button_state == LOW) {
    is_running = false;
    digitalWrite(led, LOW);   // Turn off LED to indicate the robot is stopped
    motor(0, 0);              // Stop both motors
  }

  // If the robot is running, execute the line-following code
  if (is_running) {
    display_value();  // Display sensor values in the Serial Monitor for debugging
    PID_LINE_FOLLOW();  // Use PID control for line-following
  }
}

// Function to read sensor values and calculate the sensor position on the line
void Sensor_reading() {
  sensor_position = 0;  // Reset sensor position
  total = 0;            // Reset total number of sensors detecting the line

  // Loop through the sensors (A0, A1, A2, A6, A7)
  for (byte i = 0; i < 5; i++) {
    if (i > 2) {
      s[i] = analogRead(i + 3);  // Read sensors A6 and A7
    } else {
      s[i] = analogRead(i);      // Read sensors A0, A1, A2
    }

    // Convert sensor reading to binary (1 for detecting the line, 0 for no line)
    if (s[i] > threshold) s[i] = 1;
    else s[i] = 0;

    // Calculate sensor position based on active sensors
    sensor_position += s[i] * position[i];
    total += s[i];  // Count how many sensors are detecting the line
  }

  // If any sensor is detecting the line, calculate the average position
  if (total) avg = sensor_position / total;
}

// Function to implement PID-based line-following control
void PID_LINE_FOLLOW() {
  int PID_Value, P, D;        // Variables to store the PID values
  float error, previous_error = 0;  // Variables to store the error and previous error
  int left_motor_speed, right_motor_speed;  // Variables for motor speeds
  char t;                     // Variable to store the turn direction

  // Continue running while the robot is active
  while (is_running) {
    Sensor_reading();  // Read sensor values
    error = set_point - avg;  // Calculate error (difference between set point and average position)

    // Calculate the derivative (change in error)
    D = kd * (error - previous_error);

    // Calculate the proportional term
    P = error * kp;

    // Sum the P and D terms to get the final PID value
    PID_Value = P + D;

    // Update the previous error
    previous_error = error;

    // Adjust motor speeds based on the PID value
    right_motor_speed = base_speed - PID_Value;
    left_motor_speed = base_speed + PID_Value;

    // Call motor function to move the robot
    motor(left_motor_speed, right_motor_speed);

    // If no sensors detect the line, handle lost line situation
    if (total == 0) {  // No sensors detecting the line
      digitalWrite(led, HIGH);  // Turn on LED to indicate lost line

      // Handle turns if the robot is off the line
      if (t != 's') {
        if (t == 'r') motor(turn_speed, -turn_speed);  // Turn right if needed
        else motor(-turn_speed, turn_speed);           // Turn left if needed

        // Wait until the center sensor detects the line again
        while (!s[2]) Sensor_reading();
        digitalWrite(led, LOW);  // Turn off LED when back on line
      }
    }

    // Detect if a left or right turn is needed
    if (s[4] == 1 && s[0] == 0) t = 'l';  // Left turn detected
    if (s[4] == 0 && s[0] == 1) t = 'r';  // Right turn detected

    // Handle stop signals (all sensors detect the line)
    else if (total == 5) {
      Sensor_reading();  // Check again to confirm stop signal
      if (total == 5) {
        motor(0, 0);  // Stop the robot
        while (total == 5) Sensor_reading();  // Wait until it's off the stop signal
      }
    }
  }
}

// Function to display sensor values in the Serial Monitor for debugging
void display_value() {
  for (byte i = 0; i < 5; i++) {
    Serial.print(s[i]);  // Print sensor state (0 or 1)
    Serial.print(" ");
  }
  Serial.println();  // Print a newline
  delay(100);  // Delay for readability
}

// Function to control the motors
void motor(int a, int b) {
  // Motor A (Left) control
  if (a > 0) {
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
  } else {
    a = -(a);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
  }

  // Motor B (Right) control
  if (b > 0) {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
  } else {
    b = -(b);
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
  }

  // Limit motor speed to a maximum value (PWM control)
  if (a > 200) a = 200;
  if (b > 200) b = 200;

  // Apply PWM speed control
  analogWrite(enA, a);  // Apply PWM to Motor A
  analogWrite(enB, b);  // Apply PWM to Motor B
}

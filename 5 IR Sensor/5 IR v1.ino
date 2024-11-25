// Motor pin defines
#define IN1 6  // Motor B (Right) forward
#define IN2 5  // Motor B (Right) backward
#define IN3 4  // Motor A (Left) forward
#define IN4 2  // Motor A (Left) backward
#define enA 9  // Motor A (Left) speed control (PWM)
#define enB 3  // Motor B (Right) speed control (PWM)

// Sensor variables
int s[6];                  // Array to store sensor readings
int threshold = 531;        // Threshold to detect the line (Adjust as needed)

// Push buttons and LED
int start_button_pin = 11;  // Start button pin
int stop_button_pin = 12;   // Stop button pin
int led = 13;               // Built-in LED for status indication
bool is_running = false;    // Flag to track whether the robot is running or stopped

void setup() {
  // Motor pins as OUTPUT
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  // Buttons and LED setup
  pinMode(start_button_pin, INPUT_PULLUP);  // Pull-up resistor to avoid floating values
  pinMode(stop_button_pin, INPUT_PULLUP);
  pinMode(led, OUTPUT);  // Built-in LED for status

  // Start Serial Monitor for debugging
  Serial.begin(9600);
}

void loop() {
  // Read the state of start and stop buttons
  bool start_button_state = digitalRead(start_button_pin);
  bool stop_button_state = digitalRead(stop_button_pin);

  // Start the robot if the start button is pressed
  if (start_button_state == LOW) {
    is_running = true;
    digitalWrite(led, HIGH);  // Turn on LED to indicate the robot is running
  }

  // Stop the robot if the stop button is pressed
  if (stop_button_state == LOW) {
    is_running = false;
    digitalWrite(led, LOW);  // Turn off LED to indicate the robot is stopped
    motor(0, 0);  // Stop both motors
  }

  // If the robot is running, perform line following
  if (is_running) {
    basic_line_following();
  }
}

// Basic line-following function
void basic_line_following() {
  int left_motor_speed = 200;  // Constant speed for motors
  int right_motor_speed = 200;

  // Read the sensor values
  read_sensors();

  // Check sensor readings and adjust motor speeds accordingly
  if (s[0] == 1) {
    // Leftmost sensor detects the line, turn left
    motor(left_motor_speed - 50, right_motor_speed);
  } else if (s[4] == 1) {
    // Rightmost sensor detects the line, turn right
    motor(left_motor_speed, right_motor_speed - 50);
  } else {
    // If neither extreme sensor detects the line, move straight
    motor(left_motor_speed, right_motor_speed);
  }
}

// Function to read the sensor values
void read_sensors() {
  for (byte i = 0; i < 5; i++) {
    // Read sensors A0, A1, A2, A6, A7
    if (i > 2) {
      s[i] = analogRead(i + 3);  // Read sensors A6 and A7
    } else {
      s[i] = analogRead(i);      // Read sensors A0, A1, A2
    }
    // Convert the analog value to a digital value based on the threshold
    if (s[i] > threshold) s[i] = 1;  // Line detected
    else s[i] = 0;  // No line detected
  }
}

// Function to control the motors
void motor(int left_speed, int right_speed) {
  // Motor A (Left) control
  if (left_speed > 0) {
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
  } else {
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
    left_speed = -left_speed;  // Reverse speed if going backward
  }

  // Motor B (Right) control
  if (right_speed > 0) {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
  } else {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    right_speed = -right_speed;  // Reverse speed if going backward
  }

  // Limit motor speed to a maximum value (0-255 for PWM)
  if (left_speed > 255) left_speed = 255;
  if (right_speed > 255) right_speed = 255;

  // Set motor speeds using PWM
  analogWrite(enA, left_speed);
  analogWrite(enB, right_speed);
}

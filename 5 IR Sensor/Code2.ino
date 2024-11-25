// Motor pin defines
#define IN1 6
#define IN2 5
#define IN3 4
#define IN4 2
#define enA 9
#define enB 3

// Sensor variables and line-following settings
int s[6], total, sensor_position;
int threshold = 531;  // Adjust this based on your setup
float avg;
int position[6] = {1, 2, 3, 4, 5};
int set_point = 3;

// Push buttons and LED
int start_button_pin = 11;  // Start button
int stop_button_pin = 12;   // Stop button
int led = 13;               // Built-in Arduino LED
bool is_running = false;    // Tracks if the robot is running

void setup() {
  // Motor driver pins as output
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  
  // Button pins as input with pull-up resistors
  pinMode(start_button_pin, INPUT_PULLUP);
  pinMode(stop_button_pin, INPUT_PULLUP);
  
  // LED pin as output
  pinMode(led, OUTPUT);
  
  // Initialize Serial Monitor
  Serial.begin(9600);
}

void loop() {
  // Check the state of the start and stop buttons
  bool start_button_state = digitalRead(start_button_pin);
  bool stop_button_state = digitalRead(stop_button_pin);

  // Start the robot when the start button is pressed
  if (start_button_state == LOW) {
    is_running = true;
    digitalWrite(led, HIGH);  // Solid LED to indicate running state
  }

  // Stop the robot when the stop button is pressed
  if (stop_button_state == LOW) {
    is_running = false;
    digitalWrite(led, LOW);   // Turn off the LED when stopped
    motor(0, 0);              // Stop both motors
  }

  // Run the line-following code if the robot is running
  if (is_running) {
    display_value();  // Display sensor values in the Serial Monitor

    // Line-following using PID control
    PID_LINE_FOLLOW();
  }
}

void Sensor_reading() {
  sensor_position = 0;
  total = 0;
  for (byte i = 0; i < 5; i++) {
    if (i > 2) {
      s[i] = analogRead(i + 3);  // Read sensors A6 and A7
    } else {
      s[i] = analogRead(i);      // Read sensors A0, A1, A2
    }
    // Convert analog value to digital based on threshold
    if (s[i] > threshold) s[i] = 1;
    else s[i] = 0;
    sensor_position += s[i] * position[i];
    total += s[i];
  }
  if (total) avg = sensor_position / total;  // Calculate average position
}

void PID_LINE_FOLLOW() {
  int kp = 50, kd = 500, PID_Value, P, D;
  float error, previous_error = 0;
  int base_speed = 200, left_motor_speed, right_motor_speed, turn_speed = 100;
  char t;

  while (is_running) {
    Sensor_reading();
    error = set_point - avg;
    D = kd * (error - previous_error);
    P = error * kp;
    PID_Value = P + D;
    previous_error = error;

    // Adjust motor speed based on PID value
    right_motor_speed = base_speed - PID_Value;
    left_motor_speed = base_speed + PID_Value;
    motor(left_motor_speed, right_motor_speed);  // Move motors

    Sensor_reading();
    if (total == 0) {
      digitalWrite(led, HIGH);  // Turn on LED when out of line
      if (t != 's') {
        if (t == 'r') motor(turn_speed, -turn_speed);  // Right turn
        else motor(-turn_speed, turn_speed);           // Left turn
        while (!s[2]) Sensor_reading();
        digitalWrite(led, LOW);  // Turn off LED when back on line
      }
    }

    if (s[4] == 1 && s[0] == 0) t = 'l';  // Left turn detected
    if (s[4] == 0 && s[0] == 1) t = 'r';  // Right turn detected

    else if (total == 5) {
      Sensor_reading();
      if (total == 5) {
        motor(0, 0);  // Stop motors if the robot is at a stop signal
        while (total == 5) Sensor_reading();  // Wait until it's off the stop signal
      } else if (total == 0) t = 's';  // Go straight
    }
  }
}

void display_value() {
  // Display sensor values on the Serial Monitor
  for (byte i = 0; i < 5; i++) {
    if (i > 2) {
      s[i] = analogRead(i + 3);  // Read sensors A6 and A7
    } else {
      s[i] = analogRead(i);      // Read sensors A0, A1, A2
    }
    Serial.print(String(s[i]) + " ");
  }
  Serial.println();
  delay(50);
}

// Function to control motor movement
void motor(int a, int b) {
  // Motor A control (left)
  if (a > 0) {
    digitalWrite(IN3, 1);
    digitalWrite(IN4, 0);
  } else {
    a = -(a);
    digitalWrite(IN3, 0);
    digitalWrite(IN4, 1);
  }
  // Motor B control (right)
  if (b > 0) {
    digitalWrite(IN1, 1);
    digitalWrite(IN2, 0);
  } else {
    b = -(b);
    digitalWrite(IN1, 0);
    digitalWrite(IN2, 1);
  }

  // Limit motor speed
  if (a > 200) a = 200;
  if (b > 200) b = 200;

  // Set motor speed using PWM
  analogWrite(enB, a);
  analogWrite(enA, b);
}

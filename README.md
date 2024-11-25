# UIURC_ADVANCE_LFR
Code for the ongoing workshop for advance line follower robot



# Motor Driver (TB6612FNG Motor Driver):


# Motor A Control Pins:

- IN1 (Motor A direction) → Pin 6 (Arduino) #
- IN2 (Motor A direction) → Pin 5 (Arduino)
- enA (Motor A speed - PWM) → Pin 9 (Arduino)
- IN3 (Motor B direction) → Pin 4 (Arduino)
- IN4 (Motor B direction) → Pin 2 (Arduino)
- enB (Motor B speed - PWM) → Pin 3 (Arduino)


#  Sensors (5 IR Sensors):

- Sensor 1 (Leftmost sensor) → A0 (Arduino)
- Sensor 2 → A1 (Arduino)
- Sensor 3 (Middle sensor) → A2 (Arduino)
- Sensor 4 → A6 (Arduino)
- Sensor 5 (Rightmost sensor) → A7 (Arduino)

# Push Buttons:
- Start Button → Pin 11 (Arduino)
- - Function: Starts the line-following operation when pressed.
- Stop Button → Pin 12 (Arduino)

Function: Stops the line-following operation when pressed.

# Built-in LED:
LED → Built-in LED (usually Pin 13 on the Arduino)
Function: Rapid blinking during the line-following setup, solid when the line-following is active.

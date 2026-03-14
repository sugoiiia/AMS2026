///////////////////////////////////////////////////////////////////
//      Albertaloop Mechatronics Showdown
//      AMS Robot Car Starter Code
//      Version: 2.0
//      Date:  March 10, 2026
//      Author: Albertaloop
//      
//      Description: 
//      This starter code provides basic functionality for 
//      controlling the AMS Robot Car. For detailed usage 
//      instructions and additional features, refer to the 
//      project GitHub
//
//      Note:
//      - This code is purely for reference. 
//      - There may be some mistakes in the code.
//      - For support, reach out to anyone wearing an AlbertaLoop T-shirt.
//
//      © 2026 AlbertaLoop. All rights reserved.
///////////////////////////////////////////////////////////////////

// Pin assignments motors (matches the provided schematic)
/*
 * Note: The motor enable pins can be connected directly to 5V if speed control is not needed.
 * In that case, PWM control in this code is unnecessary and the motors will run at full speed.
 */

#define back_left_enable 7
#define back_left_1 6
#define back_left_2 5
#define back_right_enable 2
#define back_right_1 4
#define back_right_2 3

#define front_right_enable 8
#define front_right_1 9
#define front_right_2 10
#define front_left_enable 13
#define front_left_1 11
#define front_left_2 12

// Set initial values
int speed_back_left = 0;
int speed_back_right = 0;
int speed_front_right = 0;
int speed_front_left = 0;

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void setup() {
  // the setup only runs once!
  
  // Set pin modes to output for the back motors
  pinMode(back_left_enable, OUTPUT);
  pinMode(back_right_enable, OUTPUT);
  pinMode(back_left_1, OUTPUT);
  pinMode(back_left_2, OUTPUT);
  pinMode(back_right_1, OUTPUT);
  pinMode(back_right_2, OUTPUT);

  // Set pin modes to output for front motors
  pinMode(front_right_enable, OUTPUT);
  pinMode(front_left_enable, OUTPUT);
  pinMode(front_right_1, OUTPUT);
  pinMode(front_right_2, OUTPUT);
  pinMode(front_left_1, OUTPUT);
  pinMode(front_left_2, OUTPUT);
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void loop() {
  // Reads analog value of A0 and A1 for joystick
  int xAxis = analogRead(A0); // Read Joysticks X-axis (left and right)
  int yAxis = analogRead(A1); // Read Joysticks Y-axis (forward and backward)

  // Y-axis used for forward and backward control
  if (yAxis < 470) {
    // Case where joystick is pushed backwards --> Robot is moving backwards

    // Set back motors backward
    // Note: Setting pin 1 to HIGH and pin 2 to LOW will spin the motors in one direction.
    //       Setting pin 1 to LOW and pin 2 to HIGH will spin the motors in the opposite direction.
    // Note that the orientation of the motor changes the direction of rotation so you may have to swap these around in code or on hardware. 
    digitalWrite(back_left_1, LOW);  // Set back left pin 1 to LOW
    digitalWrite(back_left_2, HIGH);  // Set back left pin 2 to HIGH
    digitalWrite(back_right_1, LOW);
    digitalWrite(back_right_2, HIGH);

    // Set front motors backwards
    digitalWrite(front_right_1, HIGH);
    digitalWrite(front_right_2, LOW);
    digitalWrite(front_left_1, HIGH);
    digitalWrite(front_left_2, LOW);

    // Convert the declining Y-axis readings for going backward from 470 to 0 into 0 to 255 value for the PWM signal for increasing the motor speed
    speed_back_left = map(yAxis, 470, 0, 0, 255);
    speed_back_right = map(yAxis, 470, 0, 0, 255);
    speed_front_right = map(yAxis, 470, 0, 0, 255);
    speed_front_left = map(yAxis, 470, 0, 0, 255);
  }
  else if (yAxis > 550) {
    // Case where joystick is pushed fowards --> Robot moves forwards

    // Set back motors forward

    // Note: Setting pin 1 to HIGH and pin 2 to LOW will spin the motors in one direction.
    //       Setting pin 1 to LOW and pin 2 to HIGH will spin the motors in the opposite direction.
    // Note that the orientation of the motor changes the direction of rotation
    digitalWrite(back_left_1, HIGH);
    digitalWrite(back_left_2, LOW);
    digitalWrite(back_right_1, HIGH);
    digitalWrite(back_right_2, LOW);

    // Set front motors forward
    digitalWrite(front_right_1, LOW);
    digitalWrite(front_right_2, HIGH);
    digitalWrite(front_left_1, LOW);
    digitalWrite(front_left_2, HIGH);

    // Convert the increasing Y-axis readings for going forward from 550 to 1023 into 0 to 255 value for the PWM signal for increasing the motor speed
    speed_back_left = map(yAxis, 550, 1023, 0, 255);
    speed_back_right = map(yAxis, 550, 1023, 0, 255);
    speed_front_right = map(yAxis, 550, 1023, 0, 255);
    speed_front_left = map(yAxis, 550, 1023, 0, 255);
  }
  else {
    // Case where joystick is in the middle --> Motors not moving
    speed_back_left = 0;
    speed_back_right = 0;
    speed_front_right = 0;
    speed_front_left = 0;
  }
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  // X-axis used for left and right control
  if (xAxis > 550) { 
    int xMapped = map(xAxis, 550, 1023, 0, 255);
    
    // Right motors backward
    digitalWrite(back_right_1, LOW);
    digitalWrite(back_right_2, HIGH);
    digitalWrite(front_right_1, HIGH);
    digitalWrite(front_right_2, LOW);
    speed_back_right += xMapped;
    speed_front_right += xMapped;
  
    // Left motors forward
    digitalWrite(back_left_1, HIGH);
    digitalWrite(back_left_2, LOW);
    digitalWrite(front_left_1, LOW);
    digitalWrite(front_left_2, HIGH);
    speed_back_left += xMapped;
    speed_front_left += xMapped;
  
    if (speed_back_left > 255) speed_back_left = 255;
    if (speed_back_right > 255) speed_back_right = 255;
    if (speed_front_right > 255) speed_front_right = 255;
    if (speed_front_left > 255) speed_front_left = 255;
  }
  

  else if (xAxis < 470) {
    int xMapped = map(xAxis, 470, 0, 0, 255);
  
    // Left motors backward
    digitalWrite(back_left_1, LOW);
    digitalWrite(back_left_2, HIGH);
    digitalWrite(front_left_1, HIGH);
    digitalWrite(front_left_2, LOW);
    speed_back_left += xMapped;
    speed_front_left += xMapped;
  
    // Right motors forward
    digitalWrite(back_right_1, HIGH);
    digitalWrite(back_right_2, LOW);
    digitalWrite(front_right_1, LOW);
    digitalWrite(front_right_2, HIGH);
    speed_back_right += xMapped;
    speed_front_right += xMapped;
  
    if (speed_back_left > 255) speed_back_left = 255;
    if (speed_back_right > 255) speed_back_right = 255;
    if (speed_front_right > 255) speed_front_right = 255;
    if (speed_front_left > 255) speed_front_left = 255;
  }

  // Prevent buzzing at low speeds (Adjust according to your motors. My motors couldn't start moving if PWM value was below value of 70)
  if (speed_back_left < 72) {
    speed_back_left = 0;
  }
  if (speed_back_right < 72) {
    speed_back_right = 0;
  }
  if (speed_front_right < 72) {
    speed_front_right = 0;
  }
  if (speed_front_left < 72) {
    speed_front_left = 0;
  }
  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

  // Sends PWM signal to motors
  analogWrite(back_left_enable, speed_back_left); // Send analog signal to back left motor
  analogWrite(back_right_enable, speed_back_right); // Send analog signal to back right motor
  analogWrite(front_right_enable, speed_front_right); // Send analog signal to front right
  analogWrite(front_left_enable, speed_front_left); // Send analog signal to front left
}

///////////////////////////////////////////////////////////////////
//      Albertaloop Mechatronics Showdown
//      AMS Robot Car - Mega 2560 + MPU6050 PID + Clamped PWM
///////////////////////////////////////////////////////////////////

#include <Wire.h>
#include <MPU6050_light.h>

MPU6050 mpu(Wire);

// --- Hardware Pins ---
// Motor Driver Pins
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

// --- Tuning Variables ---
// 1. Hardware Limits
int maxPWM = 130;          // Caps the 11V power to prevent violent movements (Max is 255)
int motorDeadband = 72;    // Minimum PWM required to make the motors physically spin

// 2. Calibration
float setpoint = 0.0;           // The target angle (0 = perfectly balanced)
float mechanicalOffset = 1;   // Tweak this if the robot's physical center of gravity isn't at exactly 0 degrees

// 3. PID Gains
float Kp = 30;  // Proportional gain (Tune this first!)
float Ki = 0.01;   // Integral gain
float Kd = 3;   // Derivative gain

// --- System Variables ---
int speed_back_left = 0, speed_back_right = 0;
int speed_front_right = 0, speed_front_left = 0;
float cumError = 0, lastError = 0;
unsigned long previousTime;
unsigned long lastPrintTime = 0; 

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void setup() {
  Serial.begin(9600);
  Wire.begin();
  
  Serial.println("Initializing MPU6050...");
  
  // Initialize MPU6050
  byte status = mpu.begin();
  
  while(status != 0){ 
    Serial.print("MPU6050 Connection Error! Status Code: ");
    Serial.println(status);
    Serial.println("Check SDA (Pin 20) and SCL (Pin 21) on Mega 2560.");
    delay(1000); 
  } 
  
  Serial.println("MPU6050 connected! Calculating offsets...");
  Serial.println("DO NOT MOVE THE ROBOT!");
  delay(1000);
  mpu.calcOffsets(true,true); 
  Serial.println("Offsets calculated. Ready to balance.");
  
  // Set pin modes to output
  pinMode(back_left_enable, OUTPUT);
  pinMode(back_right_enable, OUTPUT);
  pinMode(back_left_1, OUTPUT);
  pinMode(back_left_2, OUTPUT);
  pinMode(back_right_1, OUTPUT);
  pinMode(back_right_2, OUTPUT);

  pinMode(front_right_enable, OUTPUT);
  pinMode(front_left_enable, OUTPUT);
  pinMode(front_right_1, OUTPUT);
  pinMode(front_right_2, OUTPUT);
  pinMode(front_left_1, OUTPUT);
  pinMode(front_left_2, OUTPUT);
  
  previousTime = millis();
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void loop() {
  mpu.update(); 
  
  // Calculate elapsed time in seconds for PID math
  unsigned long currentTime = millis();
  float elapsedTime = (currentTime - previousTime) / 1000.0;
  if (elapsedTime <= 0.0) elapsedTime = 0.001; // Prevent divide-by-zero
  
  // 1. Get current angle and apply your physical offset
  float rawAngle = mpu.getAngleY();
  float currentAngle = rawAngle + mechanicalOffset;
  
  // 2. Calculate PID
  float error = currentAngle - setpoint;
  cumError += error * elapsedTime;
  cumError = constrain(cumError, -100, 100); // Anti-windup
  float rateError = (error - lastError) / elapsedTime;
  
  float output = (Kp * error) + (Ki * cumError) + (Kd * rateError);
  
  lastError = error;
  previousTime = currentTime;

  // 3. Constrain the PID output using your new 11V maxPWM cap
  int pidOutput = constrain((int)output, -maxPWM, maxPWM);

// 4. Motor Direction Logic
  if (pidOutput < 0) {
    // Move backwards
    
    // Left Side (Kept Original)
    digitalWrite(back_left_1, LOW);  
    digitalWrite(back_left_2, HIGH);  
    digitalWrite(front_left_1, HIGH);
    digitalWrite(front_left_2, LOW);

    // Right Side (INVERTED)
    digitalWrite(back_right_1, HIGH); 
    digitalWrite(back_right_2, LOW);  
    digitalWrite(front_right_1, LOW); 
    digitalWrite(front_right_2, HIGH);

    int mappedSpeed = abs(pidOutput);
    speed_back_left = mappedSpeed;
    speed_back_right = mappedSpeed;
    speed_front_right = mappedSpeed;
    speed_front_left = mappedSpeed;
  }
  else if (pidOutput > 0) {
    // Move forwards
    
    // Left Side (Kept Original)
    digitalWrite(back_left_1, HIGH);
    digitalWrite(back_left_2, LOW);
    digitalWrite(front_left_1, LOW);
    digitalWrite(front_left_2, HIGH);

    // Right Side (INVERTED)
    digitalWrite(back_right_1, LOW);  
    digitalWrite(back_right_2, HIGH); 
    digitalWrite(front_right_1, HIGH);
    digitalWrite(front_right_2, LOW); 

    int mappedSpeed = pidOutput;
    speed_back_left = mappedSpeed;
    speed_back_right = mappedSpeed;
    speed_front_right = mappedSpeed;
    speed_front_left = mappedSpeed;
  }
  else {
    // Perfectly balanced
    speed_back_left = 0;
    speed_back_right = 0;
    speed_front_right = 0;
    speed_front_left = 0;
  }

  // 5. Apply Motor Deadband
  if (speed_back_left < motorDeadband) speed_back_left = 0;
  if (speed_back_right < motorDeadband) speed_back_right = 0;
  if (speed_front_right < motorDeadband) speed_front_right = 0;
  if (speed_front_left < motorDeadband) speed_front_left = 0;

  // 6. Send PWM signal to motors
  analogWrite(back_left_enable, speed_back_left); 
  analogWrite(back_right_enable, speed_back_right); 
  analogWrite(front_right_enable, speed_front_right); 
  analogWrite(front_left_enable, speed_front_left); 

  // --- DEBUG OUTPUT ---
  if (currentTime - lastPrintTime >= 100) {
    Serial.print("Raw Angle: ");
    Serial.print(rawAngle);
    Serial.print("  |  Corrected Angle: ");
    Serial.print(currentAngle);
    Serial.print("  |  PID Out: ");
    Serial.print(pidOutput);
    Serial.print("  |  Motor PWM: ");
    Serial.println(speed_back_left); 
    
    lastPrintTime = currentTime;
  }
}

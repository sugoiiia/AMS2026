#define TRIG_PIN     22
#define ECHO_PIN     23
#define FRONT_IR_PIN 24
#define RIGHT_IR_PIN 25

void setup() {
  Serial.begin(9600);
  pinMode(TRIG_PIN,     OUTPUT);
  pinMode(ECHO_PIN,     INPUT);
  pinMode(FRONT_IR_PIN, INPUT);
  pinMode(RIGHT_IR_PIN, INPUT);
  Serial.println("Sensor test ready...");
}

void loop() {
  // Ultrasonic
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  long duration = pulseIn(ECHO_PIN, HIGH, 30000);
  int distance = duration * 0.034 / 2;

  // IR sensors
  bool frontWall = (digitalRead(FRONT_IR_PIN) == LOW);
  bool rightWall = (digitalRead(RIGHT_IR_PIN) == LOW);

  Serial.print("Left dist: "); Serial.print(distance); Serial.print(" cm");
  Serial.print(" | Front IR: "); Serial.print(frontWall ? "WALL" : "clear");
  Serial.print(" | Right IR: "); Serial.println(rightWall ? "WALL" : "clear");

  delay(200);
}
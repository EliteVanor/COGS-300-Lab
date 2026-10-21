
#include <Servo.h>

// Ultrasonic Sensor 1
int trigPin1 = 13;
int echoPin1 = 12;

// Ultrasonic Sensor 2
int trigPin2 = 6;
int echoPin2 = 7;

// Servo
int servoPin = 9;
int pos = 0;

int maxDist = 1000;
int minDist = 0;

Servo myservo;

long duration1, cm1, inches1;
long duration2, cm2, inches2;

void setup() {
  Serial.begin(9600);

  myservo.attach(servoPin);

  // Sensor 1
  pinMode(trigPin1, OUTPUT);
  pinMode(echoPin1, INPUT);

  // Sensor 2
  pinMode(trigPin2, OUTPUT);
  pinMode(echoPin2, INPUT);
}

void loop() {
  // ===== SENSOR 1 =====
  digitalWrite(trigPin1, LOW);
  delayMicroseconds(5);
  digitalWrite(trigPin1, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin1, LOW);

  duration1 = pulseIn(echoPin1, HIGH, 30000);

  cm1 = (duration1 / 2) / 29.1;
  inches1 = (duration1 / 2) / 74;

  // Small gap to reduce interference
  delay(60);

  // ===== SENSOR 2 =====
  digitalWrite(trigPin2, LOW);
  delayMicroseconds(5);
  digitalWrite(trigPin2, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin2, LOW);

  duration2 = pulseIn(echoPin2, HIGH, 30000);

  cm2 = (duration2 / 2) / 29.1;
  inches2 = (duration2 / 2) / 74;

  // ===== SERIAL MONITOR =====
  Serial.print("Sensor 1: ");
  if (duration1 == 0) {
    Serial.print("No echo");
  } else {
    Serial.print(cm1);
    Serial.print(" cm");
  }

  Serial.print(" | Sensor 2: ");
  if (duration2 == 0) {
    Serial.print("No echo");
  } else {
    Serial.print(cm2);
    Serial.print(" cm");
  }

  Serial.println();

  // ===== SERVO CONTROL =====
  // Sensor 1 controls the servo, as before
  if (duration1 > 0) {
    pos = map(cm1, minDist, maxDist, 0, 180);
    pos = constrain(pos, 0, 180);
    myservo.write(pos);
  }

  delay(250);
}

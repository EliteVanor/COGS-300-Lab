// Motor 1 = left side, Motor 2 = right side (swap the pins if yours is mirrored)
int motor1pin1 = 2;
int motor1pin2 = 3;
int motor2pin1 = 4;
int motor2pin2 = 5;
int ENA = 9;
int ENB = 10;

int DRIVE_SPEED = 255;  // 0-255
int TURN_SPEED  = 255;  // spin-in-place speed
int CURVE_INNER = 140;   // inner wheel speed while moving + turning (0 = pivot, 255 = no turn)

unsigned long lastCmdTime = 0;

// Balance calibration: PWM value that makes each motor run at "full speed".
// Motor A (ENA) is stronger, so it gets a lower ceiling.
int MAX_A = 255;
int MAX_B = 255;

void setMotor(int p1, int p2, int en, int speed, int maxPwm) {
  digitalWrite(p1, speed > 0 ? HIGH : LOW);
  digitalWrite(p2, speed < 0 ? HIGH : LOW);
  // Scale 0-255 down to 0-maxPwm so every speed stays balanced, not just full speed
  analogWrite(en, map(abs(speed), 0, 255, 0, maxPwm));
}

void drive(int left, int right) {
  setMotor(motor1pin1, motor1pin2, ENA, left, MAX_A);
  setMotor(motor2pin1, motor2pin2, ENB, right, MAX_B);
}

void setup() {
  pinMode(motor1pin1, OUTPUT);
  pinMode(motor1pin2, OUTPUT);
  pinMode(motor2pin1, OUTPUT);
  pinMode(motor2pin2, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);
  Serial.begin(9600);
  drive(0, 0);
}

void loop() {
  while (Serial.available() > 0) {
    char c = Serial.read();
    lastCmdTime = millis();
    switch (c) {
      case 'F': drive(DRIVE_SPEED, DRIVE_SPEED); break;
      case 'B': drive(-DRIVE_SPEED, -DRIVE_SPEED); break;
      case 'L': drive(-TURN_SPEED, TURN_SPEED); break;
      case 'R': drive(TURN_SPEED, -TURN_SPEED); break;
      case 'Q': drive(CURVE_INNER, DRIVE_SPEED); break;
      case 'E': drive(DRIVE_SPEED, CURVE_INNER); break;
      case 'Z': drive(-CURVE_INNER, -DRIVE_SPEED); break;
      case 'C': drive(-DRIVE_SPEED, -CURVE_INNER); break;
      default:  drive(0, 0); break;
    }
  }

  // Failsafe: no command for 400 ms (sketch closed, cable pulled) -> stop
  if (millis() - lastCmdTime > 400) {
    drive(0, 0);
  }
}

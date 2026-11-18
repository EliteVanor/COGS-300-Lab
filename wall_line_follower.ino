// Left wall follower + white tape line follower.
//
// WALL MODE (unchanged from the working version)
//   Holds TARGET_CM from the left wall, wheel speed set by encoders.
//   FOLLOW : PD on side distance. Too far from wall -> right wheel faster (steer left).
//   SEARCH : left wall gone -> arc left until it is found again.
//   TURN_L : front blocked and no wall on the left -> pivot left.
//   TURN_R : front blocked and wall on the left    -> pivot right.
//            The direction is decided once, when the turn starts.
//
// LINE MODE (new)
//   Entered as soon as any IR sensor sees tape. Line following has priority over the wall.
//   TRACK  : steer from the 3 sensors. Middle only = straight, middle+side = soft, side only = hard.
//   lost after a side sensor saw it -> pivot toward that side until the line comes back.
//            This is what handles 90 degree and sharp (up to ~150 degree) corners.
//   lost while centered -> creep straight a few ticks (line was in the gap between sensors),
//            then pivot toward the side the line was last seen on, then the other way.
//   Left wall seen (side ultrasonic) -> WALL MODE immediately.
//   Nothing found after both sweeps -> back to WALL MODE.
//   Only the side ultrasonic runs in line mode, so the control loop stays fast.
//   All line speeds are scaled by LINE_SLOW.
//
// Wheels never reverse (a single-channel encoder cannot tell direction).
// Speeds are in encoder ticks per second. Each wheel is feedforward + PI.
// Motor A (left) reaches the same speed as motor B (right) at PWM 215 vs 255,
// so the left feedforward is scaled by 215/255. The PI trims whatever is left over.

// ---------- Motor pins ----------
const int L_IN1 = 3, L_IN2 = 2, ENA = 9;    // left motor (A)
const int R_IN1 = 5, R_IN2 = 4, ENB = 10;   // right motor (B)

// ---------- Ultrasonic pins ----------
const int TRIG_SIDE = 6,  ECHO_SIDE = 7;
const int TRIG_FRONT = 13, ECHO_FRONT = 12;

// ---------- Encoders (D0 output of each module) ----------
// If you move a wire, change the pin AND its bit (A0 = PC0, A1 = PC1, ...).
const int ENC_L_PIN = A1;
const int ENC_R_PIN = A0;
#define ENC_L_BIT PC1
#define ENC_R_BIT PC0
const unsigned long ENC_MIN_PERIOD_US = 200;     // closer ticks are noise
const unsigned long ENC_STALL_US      = 300000;  // no tick for this long = stopped

// ---------- Line sensors (D0 output of each IR module) ----------
// On port C like the encoders, so the same interrupt catches tape even while
// pulseIn() is blocking in wall mode. Change the pin AND its bit together.
const int IR_L_PIN = A2;
const int IR_M_PIN = A3;
const int IR_R_PIN = A4;
#define IR_L_BIT PC2
#define IR_M_BIT PC3
#define IR_R_BIT PC4
const uint8_t IR_MASK = _BV(IR_L_BIT) | _BV(IR_M_BIT) | _BV(IR_R_BIT);
const bool IR_TAPE_LOW = true;   // most TCRT5000 boards pull D0 LOW over white. Flip if yours are inverted.

// ---------- Wheel speed control ----------
const float MAX_TPS    = 80.0;          // right wheel speed at PWM 255. Measure it, then set it.
const float LEFT_SCALE = 215.0 / 255.0;  // left feedforward scale (motor A is stronger)
const float KP_W       = 1.5;            // PWM per tick/s of error
const float KI_W       = 4.0;            // PWM per second per tick/s of error
const float INTEG_MAX  = 100.0;          // cap on integral, in PWM
const int   PWM_MIN    = 70;             // PWM where a wheel just starts to turn

// ---------- Speeds (ticks per second). Keep CRUISE_TPS well under MAX_TPS. ----------
const float CRUISE_TPS = 50.0;           // front clear
const float MIN_TPS    = 30.0;           // front close
const float TURN_TPS   = 45.0;           // outer wheel while turning
const float TURN_INNER_FRAC  = 0.0;      // inner wheel during turn, 0 = pivot
const float SEARCH_INNER_FRAC = 0.3;     // left wheel while searching, fraction of speed

// ===================================================================================================
// ====================================== Tuning space ===============================================
// ===================================================================================================
// ---------- Wall following ----------
const float TARGET_CM   = 8.0;
const float DEADBAND_CM = 0.3;
const float ERR_MAX_CM  = 12.0;           // cap on position error, stops steep dives at the wall
const float KP          = 4.0;           // tick/s of steering per cm of error
const float KD          = 2.0;           // tick/s of steering per cm/s of drift
const float STEER_MAX   = 40.0;          // tick/s
const float WALL_MAX_CM = 50.0;          // side reading beyond this = no wall
const int   LOST_LOOPS  = 2;             // bad side readings in a row before the wall is lost

// ---------- Front ----------
const float FRONT_TURN_CM  = 15.0;       // closer than this -> turn
const float FRONT_CLEAR_CM = 25.0;       // must read above this to leave the turn
const float SLOW_START_CM  = 80.0;       // start slowing here

// ---------- Sensor timeouts ----------
const unsigned long SIDE_TIMEOUT_US  = 3500;   // about 60 cm
const unsigned long FRONT_TIMEOUT_US = 9000;   // about 150 cm

// ---------- Line following ----------
const float LINE_TPS        = 40.0;      // base speed, line under the middle sensor
const float LINE_EDGE_TPS   = 32.0;      // base speed when the line is off center
const float LINE_STEER_HALF = 0.8;       // middle + side: steer as a fraction of base speed
const float LINE_STEER_SIDE = 0.8;       // side only: steer as a fraction of base speed
const float LINE_PIVOT_TPS  = 35.0;      // outer wheel while pivoting to find the line (inner stopped)
const unsigned long LINE_PERIOD_US = 8000;   // control period in line mode
const int   LINE_ENTER_LOOPS = 2;        // wall loops in a row with tape before switching to line mode
const unsigned long LINE_CREEP_TICKS = 3;    // straight distance after losing a centered line
const float TICKS_PER_90    = 30.0;      // outer-wheel ticks for a 90 degree pivot. Measure it.
const float SEARCH1_DEG     = 220.0;     // first sweep, toward the side the line was last seen
const float SEARCH2_DEG     = 400.0;     // second sweep, the other way (undoes the first, then 180 more)
const float LINE_SLOW       = 0.8;       // all line-mode speeds are multiplied by this (0.7 = 30% slower)
const int   LINE_TO_WALL_LOOPS = 2;      // side wall seen this many line loops in a row -> wall mode

// ===================================================================================================
// ===================================================================================================

// ---------- State ----------
enum State { FOLLOW, TURN_L, TURN_R };
State state = FOLLOW;

float sideF = 0, prevSide = 0, dFilt = 0;
bool  haveSide = false;
int   missCount = LOST_LOOPS;     // start as "no wall" until one is seen

float lastFront = 999;
int   frontMisses = 0;

unsigned long loopUs = 0;
int printCount = 0;

// ---------- Line state ----------
enum LineState { LT_TRACK, LT_CREEP, LT_SEARCH };
bool lineMode = false;
LineState lineState = LT_TRACK;
float lastPos = 0;                // +1 line far left ... -1 line far right
int   lastSide = 1;               // side the line was last seen on: +1 left, -1 right
int   searchDir = 1;              // +1 pivot left, -1 pivot right
int   searchPhase = 1;
unsigned long markTicks = 0;
int   enterCount = 0;
int   wallCount = 0;              // line mode: side wall readings in a row

// ---------- Encoder state (ISR writes, loop reads with interrupts off) ----------
volatile unsigned long lastTickL = 0, lastTickR = 0;
volatile unsigned long periodL = 0, periodR = 0;
volatile unsigned long ticksL = 0, ticksR = 0;
volatile uint8_t prevPinC = 0;
volatile uint8_t irLatch = 0;     // IR bits that saw tape since the last readIR()

float measTps[2] = {0, 0};   // 0 = left, 1 = right
float integ[2]   = {0, 0};

ISR(PCINT1_vect) {
  uint8_t pins = PINC;
  uint8_t changed = pins ^ prevPinC;
  prevPinC = pins;
  unsigned long now = micros();

  if ((changed & _BV(ENC_L_BIT)) && (pins & _BV(ENC_L_BIT))) {
    unsigned long p = now - lastTickL;
    if (p > ENC_MIN_PERIOD_US) { periodL = p; lastTickL = now; ticksL++; }
  }
  if ((changed & _BV(ENC_R_BIT)) && (pins & _BV(ENC_R_BIT))) {
    unsigned long p = now - lastTickR;
    if (p > ENC_MIN_PERIOD_US) { periodR = p; lastTickR = now; ticksR++; }
  }

  // Remember any tape seen, so a short crossing is never missed between loops.
  irLatch |= (uint8_t)((IR_TAPE_LOW ? ~pins : pins) & IR_MASK);
}

// Speed from the time between the last two ticks. Stays accurate at low speed.
float wheelTps(bool left) {
  unsigned long last, per;
  noInterrupts();
  if (left) { last = lastTickL; per = periodL; }
  else      { last = lastTickR; per = periodR; }
  interrupts();

  unsigned long since = micros() - last;
  if (since > ENC_STALL_US || per == 0) return 0;
  return 1000000.0 / max(per, since);    // an overdue tick means the wheel is slowing
}

unsigned long ticksOf(bool left) {
  noInterrupts();
  unsigned long t = left ? ticksL : ticksR;
  interrupts();
  return t;
}

// Tape bits seen since the last call, plus what is under the sensors right now.
uint8_t readIR() {
  noInterrupts();
  uint8_t seen = irLatch;
  irLatch = 0;
  interrupts();
  uint8_t pins = PINC;
  seen |= (uint8_t)((IR_TAPE_LOW ? ~pins : pins) & IR_MASK);
  return seen;
}

// ---------- Motors ----------
void setMotor(int in1, int in2, int en, int pwm) {
  digitalWrite(in1, pwm > 0 ? HIGH : LOW);
  digitalWrite(in2, LOW);
  analogWrite(en, pwm);
}

// PWM for wheel i (0 left, 1 right) to hit targetTps.
int wheelPwm(int i, float targetTps, float dt) {
  if (targetTps <= 0) { integ[i] = 0; return 0; }

  float err = targetTps - measTps[i];
  integ[i] = constrain(integ[i] + KI_W * err * dt, -INTEG_MAX, INTEG_MAX);

  float ff = 255.0 * targetTps / MAX_TPS;
  if (i == 0) ff *= LEFT_SCALE;

  float pwm = ff + KP_W * err + integ[i];
  return constrain((int)pwm, PWM_MIN, 255);
}

void drive(float leftTps, float rightTps, float dt) {
  measTps[0] = 0.5 * measTps[0] + 0.5 * wheelTps(true);
  measTps[1] = 0.5 * measTps[1] + 0.5 * wheelTps(false);
  setMotor(L_IN1, L_IN2, ENA, wheelPwm(0, leftTps, dt));
  setMotor(R_IN1, R_IN2, ENB, wheelPwm(1, rightTps, dt));
}

// ---------- Ultrasonic ----------
// Distance in cm, or -1 if no echo.
float readCm(int trig, int echo, unsigned long timeoutUs) {
  digitalWrite(trig, LOW);  delayMicroseconds(5);
  digitalWrite(trig, HIGH); delayMicroseconds(10);
  digitalWrite(trig, LOW);
  long d = pulseIn(echo, HIGH, timeoutUs);
  if (d == 0) return -1;
  return d * 0.0343 / 2.0;
}

// ---------- Line mode ----------
void enterLineMode() {
  lineMode = true;
  lineState = LT_TRACK;
  lastPos = 0;
  enterCount = 0;
  wallCount = 0;
}

// Back to the wall follower with fresh state.
// wallSeen = true when we are leaving because the left wall showed up.
void exitLineMode(bool wallSeen) {
  lineMode = false;
  state = FOLLOW;
  haveSide = false;
  dFilt = 0;
  missCount = wallSeen ? 0 : LOST_LOOPS;
  lastFront = 999;
  frontMisses = 0;
}

// Pivot on the inner wheel. Rotation is measured on the outer wheel.
void startSearch(int dir, int phase) {
  searchDir = dir;
  searchPhase = phase;
  markTicks = ticksOf(dir < 0);          // pivot left -> right wheel is outer
  lineState = LT_SEARCH;
}

void lineStep(uint8_t ir, float dt) {
  bool L = ir & _BV(IR_L_BIT);
  bool M = ir & _BV(IR_M_BIT);
  bool R = ir & _BV(IR_R_BIT);
  float tl = 0, tr = 0;

  // Left wall showed up -> wall following takes over right away.
  float sideRaw = readCm(TRIG_SIDE, ECHO_SIDE, SIDE_TIMEOUT_US);
  wallCount = (sideRaw >= 0 && sideRaw < WALL_MAX_CM) ? wallCount + 1 : 0;
  if (wallCount >= LINE_TO_WALL_LOOPS) { exitLineMode(true); return; }

  if (L || M || R) {
    // Where the line is: + = left of center.
    float pos = 0;
    if (L && !R)      pos = M ? 0.5 : 1.0;
    else if (R && !L) pos = M ? -0.5 : -1.0;
    if (pos > 0) lastSide = 1; else if (pos < 0) lastSide = -1;
    lastPos = pos;
    lineState = LT_TRACK;

    float base = (pos == 0) ? LINE_TPS : LINE_EDGE_TPS;
    float k = (fabs(pos) > 0.75) ? LINE_STEER_SIDE : (fabs(pos) > 0.25) ? LINE_STEER_HALF : 0.0;
    float steer = (pos > 0 ? 1.0 : -1.0) * k * base;     // + = steer left
    tl = (base - steer) * LINE_SLOW;
    tr = (base + steer) * LINE_SLOW;
  } else {
    if (lineState == LT_TRACK) {
      if (lastPos != 0) {
        startSearch(lastPos > 0 ? 1 : -1, 1);  // it slid off one side: corner, turn that way
      } else {
        lineState = LT_CREEP;                   // probably sitting in the gap between sensors
        markTicks = (ticksOf(true) + ticksOf(false)) / 2;
      }
    }

    if (lineState == LT_CREEP) {
      if ((ticksOf(true) + ticksOf(false)) / 2 - markTicks >= LINE_CREEP_TICKS) startSearch(lastSide, 1);
      else { tl = LINE_EDGE_TPS * LINE_SLOW; tr = LINE_EDGE_TPS * LINE_SLOW; }
    }

    if (lineState == LT_SEARCH) {
      float deg = (ticksOf(searchDir < 0) - markTicks) * 90.0 / TICKS_PER_90;
      if (deg >= (searchPhase == 1 ? SEARCH1_DEG : SEARCH2_DEG)) {
        if (searchPhase == 1) startSearch(-searchDir, 2);
        else { exitLineMode(false); return; }   // no line anywhere: wall mode takes over
      }
      if (searchDir > 0) { tl = 0; tr = LINE_PIVOT_TPS * LINE_SLOW; }
      else               { tl = LINE_PIVOT_TPS * LINE_SLOW; tr = 0; }
    }
  }

  drive(tl, tr, dt);

  if (++printCount >= 12) {
    printCount = 0;
    Serial.print(lineState == LT_TRACK ? "L_TRACK" : lineState == LT_CREEP ? "L_CREEP" :
                 searchDir > 0 ? "L_PIVOT_L" : "L_PIVOT_R");
    Serial.print(" IR:"); Serial.print(L); Serial.print(M); Serial.print(R);
    Serial.print(" pos:"); Serial.print(lastPos);
    Serial.print(" tL:"); Serial.print(tl);
    Serial.print(" tR:"); Serial.print(tr);
    Serial.print(" wL:"); Serial.print(measTps[0]);
    Serial.print(" wR:"); Serial.print(measTps[1]);
    Serial.print(" nL:"); Serial.print(ticksOf(true));
    Serial.print(" nR:"); Serial.println(ticksOf(false));
  }
}

void setup() {
  pinMode(L_IN1, OUTPUT); pinMode(L_IN2, OUTPUT); pinMode(ENA, OUTPUT);
  pinMode(R_IN1, OUTPUT); pinMode(R_IN2, OUTPUT); pinMode(ENB, OUTPUT);
  pinMode(TRIG_SIDE, OUTPUT);  pinMode(ECHO_SIDE, INPUT);
  pinMode(TRIG_FRONT, OUTPUT); pinMode(ECHO_FRONT, INPUT);

  pinMode(ENC_L_PIN, INPUT);
  pinMode(ENC_R_PIN, INPUT);
  pinMode(IR_L_PIN, INPUT);
  pinMode(IR_M_PIN, INPUT);
  pinMode(IR_R_PIN, INPUT);
  prevPinC = PINC;
  PCICR  |= _BV(PCIE1);
  PCMSK1 |= _BV(ENC_L_BIT) | _BV(ENC_R_BIT) | IR_MASK;

  Serial.begin(115200);
  setMotor(L_IN1, L_IN2, ENA, 0);
  setMotor(R_IN1, R_IN2, ENB, 0);
  delay(1000);                 // time to put the car down
  loopUs = micros();
}

void loop() {
  // Line mode has no ultrasonic delays, so pace it to keep the wheel PI sane.
  if (lineMode) { while (micros() - loopUs < LINE_PERIOD_US) {} }

  // ---------- Time step ----------
  unsigned long nowUs = micros();
  float dt = constrain((nowUs - loopUs) / 1000000.0, 0.005, 0.2);
  loopUs = nowUs;

  // ---------- Line check: tape and no wall on the left -> line mode ----------
  // Wall wins when both are present, so the two modes don't flip back and forth.
  uint8_t ir = readIR();
  if (!lineMode) {
    bool noWall = (missCount >= LOST_LOOPS);
    enterCount = (ir && noWall) ? enterCount + 1 : 0;
    if (enterCount >= LINE_ENTER_LOOPS) enterLineMode();
  }
  if (lineMode) { lineStep(ir, dt); return; }

  // ===================== WALL MODE (unchanged) =====================

  // ---------- Sensors ----------
  float sideRaw = readCm(TRIG_SIDE, ECHO_SIDE, SIDE_TIMEOUT_US);
  delay(5);                    // so the two sensors don't hear each other
  float front = readCm(TRIG_FRONT, ECHO_FRONT, FRONT_TIMEOUT_US);

  // Front: no echo right after a close reading is the blind zone, not open space.
  if (front < 0) {
    if (frontMisses < 3 && lastFront < SLOW_START_CM) { frontMisses++; front = lastFront; }
    else { front = 999; lastFront = 999; }
  } else {
    frontMisses = 0;
    lastFront = front;
  }

  // Side: smooth the distance, estimate drift speed (positive = moving away).
  bool wallSeen = (sideRaw >= 0 && sideRaw < WALL_MAX_CM);
  if (wallSeen) {
    if (haveSide) {
      sideF = 0.6 * sideF + 0.4 * sideRaw;
      dFilt = 0.7 * dFilt + 0.3 * (sideF - prevSide) / dt;
    } else {
      sideF = sideRaw;
      dFilt = 0;
    }
    prevSide = sideF;
    haveSide = true;
    missCount = 0;
  } else if (missCount < LOST_LOOPS) {
    missCount++;
  }
  bool wallLost = (missCount >= LOST_LOOPS);
  if (wallLost) { haveSide = false; dFilt = 0; }

  // ---------- State change ----------
  if (state == FOLLOW && front < FRONT_TURN_CM) {
    state = wallLost ? TURN_L : TURN_R;        // no left wall -> left, otherwise right
  } else if (state != FOLLOW && front >= FRONT_CLEAR_CM) {
    state = FOLLOW;
    haveSide = false;                          // car has rotated, restart side history
    dFilt = 0;
    missCount = 0;
  }

  // ---------- Base speed: slows as the front gets close ----------
  float t = constrain((front - FRONT_TURN_CM) / (SLOW_START_CM - FRONT_TURN_CM), 0.0, 1.0);
  float speed = MIN_TPS + t * (CRUISE_TPS - MIN_TPS);

  // ---------- Behaviour ----------
  float tl = speed, tr = speed, steer = 0;

  if (state == TURN_L) {
    tl = TURN_TPS * TURN_INNER_FRAC;
    tr = TURN_TPS;
  } else if (state == TURN_R) {
    tl = TURN_TPS;
    tr = TURN_TPS * TURN_INNER_FRAC;
  } else if (wallLost) {
    tl = speed * SEARCH_INNER_FRAC;            // arc left to find the wall
    tr = speed;
  } else if (haveSide) {
    float err = constrain(sideF - TARGET_CM, -ERR_MAX_CM, ERR_MAX_CM);   // + = too far
    if (fabs(err) < DEADBAND_CM) err = 0;
    steer = constrain(KP * err + KD * dFilt, -STEER_MAX, STEER_MAX);     // + = steer left
    tl = max(speed - steer, 0.0f);
    tr = max(speed + steer, 0.0f);
  }

  drive(tl, tr, dt);

  // ---------- Debug ----------
  if (++printCount >= 5) {
    printCount = 0;
    Serial.print(state == TURN_L ? "TURN_L" : state == TURN_R ? "TURN_R" : wallLost ? "SEARCH" : "FOLLOW");
    Serial.print(" S:");  Serial.print(sideF);
    Serial.print(" dS:"); Serial.print(dFilt);
    Serial.print(" F:");  Serial.print(front);
    Serial.print(" tL:"); Serial.print(tl);
    Serial.print(" tR:"); Serial.print(tr);
    Serial.print(" wL:"); Serial.print(measTps[0]);
    Serial.print(" wR:"); Serial.println(measTps[1]);
  }
}

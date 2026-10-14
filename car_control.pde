import processing.serial.*;

// Change this to the port your Arduino shows up on (Tools > Port in the Arduino IDE)
String PORT_NAME = "COM9";

Serial port;
boolean connected = false;
boolean w, a, s, d;
char lastCmd = 'S';
int lastSend = 0;

// Visual state (simulated from the commands, not read back from the car)
float vL = 0, vR = 0;        // smoothed wheel speeds, -1..1
float heading = 0;           // radians, 0 = up, positive = clockwise
float worldX = 0, worldY = 0;
float treadL = 0, treadR = 0;

// Palette
color BG       = color(18, 20, 28);
color PANEL    = color(30, 34, 46);
color KEY_OFF  = color(40, 45, 60);
color ACCENT   = color(0, 200, 255);
color TEXT_DIM = color(120, 128, 150);

float KEY = 56;   // keycap size
float GAP = 8;

void setup() {
  size(864, 552);
  surface.setTitle("Car Control");
  println(Serial.list()); // shows available ports if the name above is wrong
  try {
    port = new Serial(this, PORT_NAME, 9600);
    connected = true;
  } catch (Exception e) {
    println("Could not open " + PORT_NAME + ": " + e.getMessage());
  }
}

void draw() {
  background(BG);

  // If the window loses focus, key releases are never seen. Release everything
  // so the car cannot keep driving on a stuck key.
  if (!focused) { w = false; a = false; s = false; d = false; }

  char cmd = readCommand();
  sendCommand(cmd);
  updateMotion(cmd);

  drawFloor();
  drawCar();
  drawHud(cmd);
  drawKeys();
  drawWheelBars();
}

// ---------- control logic ----------

char readCommand() {
  int fwd  = (w ? 1 : 0) - (s ? 1 : 0);
  int turn = (d ? 1 : 0) - (a ? 1 : 0);

  char cmd = 'S';
  if (fwd > 0) {
    if (turn < 0) cmd = 'Q';       // forward + left curve
    else if (turn > 0) cmd = 'E';  // forward + right curve
    else cmd = 'F';
  } else if (fwd < 0) {
    if (turn < 0) cmd = 'Z';       // reverse + left curve
    else if (turn > 0) cmd = 'C';  // reverse + right curve
    else cmd = 'B';
  } else {
    if (turn < 0) cmd = 'L';       // spin left in place
    else if (turn > 0) cmd = 'R';  // spin right in place
  }
  return cmd;
}

// Send on change, and also every 100 ms as a heartbeat.
// The Arduino stops itself if the heartbeat disappears.
void sendCommand(char cmd) {
  if (cmd != lastCmd || millis() - lastSend > 100) {
    if (connected) port.write(cmd);
    lastCmd = cmd;
    lastSend = millis();
  }
}

// Wheel speeds the firmware uses for each command (left, right), as -1..1
float[] wheelTarget(char c) {
  float inner = 150.0 / 255.0;  // matches CURVE_INNER
  float spin  = 205.0 / 255.0; // matches TURN_SPEED
  switch (c) {
    case 'F': return new float[] { 1, 1 };
    case 'B': return new float[] { -1, -1 };
    case 'L': return new float[] { -spin, spin };
    case 'R': return new float[] { spin, -spin };
    case 'Q': return new float[] { inner, 1 };
    case 'E': return new float[] { 1, inner };
    case 'Z': return new float[] { -inner, -1 };
    case 'C': return new float[] { -1, -inner };
  }
  return new float[] { 0, 0 };
}

String commandName(char c) {
  switch (c) {
    case 'F': return "FORWARD";
    case 'B': return "REVERSE";
    case 'L': return "SPIN LEFT";
    case 'R': return "SPIN RIGHT";
    case 'Q': return "FORWARD LEFT";
    case 'E': return "FORWARD RIGHT";
    case 'Z': return "REVERSE LEFT";
    case 'C': return "REVERSE RIGHT";
  }
  return "STOPPED";
}

void updateMotion(char cmd) {
  float[] t = wheelTarget(cmd);
  vL = lerp(vL, t[0], 0.2);
  vR = lerp(vR, t[1], 0.2);

  float v = (vL + vR) / 2;
  heading += (vL - vR) * 0.025;   // right wheel faster = turn left
  worldX += sin(heading) * v * 4;
  worldY -= cos(heading) * v * 4;
  treadL += vL * 4;
  treadR += vR * 4;
}

// ---------- drawing ----------

void drawFloor() {
  float sp = 48;
  float ox = -(((worldX % sp) + sp) % sp);
  float oy = -(((worldY % sp) + sp) % sp);
  stroke(255, 14);
  strokeWeight(1);
  for (float x = ox; x < width; x += sp) line(x, 0, x, height);
  for (float y = oy; y < height; y += sp) line(0, y, width, y);
}

void drawCar() {
  float v = (vL + vR) / 2;

  pushMatrix();
  translate(width / 2, height / 2 - 10);
  rotate(heading);
  rectMode(CENTER);

  // headlight beam
  noStroke();
  fill(255, 235, 170, 45 * max(0, v));
  quad(-22, -50, 22, -50, 70, -150, -70, -150);

  // shadow
  fill(0, 70);
  rect(5, 8, 64, 104, 18);

  // wheels
  drawWheel(-38, -30, treadL);
  drawWheel(-38, 30, treadL);
  drawWheel(38, -30, treadR);
  drawWheel(38, 30, treadR);

  // body
  stroke(120, 235, 255);
  strokeWeight(2);
  fill(24, 120, 150);
  rect(0, 0, 64, 104, 18);

  // cabin and front marker
  noStroke();
  fill(14, 60, 80);
  rect(0, 8, 40, 44, 10);
  fill(120, 235, 255);
  triangle(-9, -26, 9, -26, 0, -40);

  // headlights and taillights
  fill(255, 240, 190, 120 + 135 * max(0, v));
  ellipse(-20, -46, 10, 6);
  ellipse(20, -46, 10, 6);
  fill(255, 60, 60, 80 + 175 * max(0, -v));
  ellipse(-20, 47, 10, 6);
  ellipse(20, 47, 10, 6);

  popMatrix();
}

void drawWheel(float cx, float cy, float tread) {
  rectMode(CENTER);
  noStroke();
  fill(15, 17, 24);
  rect(cx, cy, 14, 36, 4);

  stroke(70, 76, 96);
  strokeWeight(2);
  float sp = 9;
  float off = ((tread % sp) + sp) % sp;
  for (float y = -17 + off; y < 17; y += sp) {
    line(cx - 5, cy + y, cx + 5, cy + y);
  }
}

void drawHud(char cmd) {
  textSize(13);
  textAlign(LEFT, CENTER);
  fill(TEXT_DIM);
  text("CAR CONTROL", 28, 30);

  // connection status
  String status = connected ? PORT_NAME + " connected" : PORT_NAME + " not connected";
  textAlign(RIGHT, CENTER);
  text(status, width - 28, 30);
  noStroke();
  fill(connected ? color(70, 220, 130) : color(240, 90, 90));
  ellipse(width - 28 - textWidth(status) - 12, 30, 8, 8);

  // current command chip
  String name = commandName(cmd);
  textSize(14);
  float cw = textWidth(name) + 36;
  rectMode(CENTER);
  noStroke();
  fill(PANEL);
  rect(width / 2, 30, cw, 30, 15);
  textAlign(CENTER, CENTER);
  fill(cmd == 'S' ? TEXT_DIM : ACCENT);
  text(name, width / 2, 29);
}

void drawKeys() {
  float x0 = 32;
  float y0 = height - 32 - (2 * KEY + GAP);

  rectMode(CORNER);
  noStroke();
  fill(PANEL, 220);
  rect(x0 - 16, y0 - 16, 3 * KEY + 2 * GAP + 32, 2 * KEY + GAP + 32, 18);

  drawKey("W", x0 + KEY + GAP, y0, w);
  drawKey("A", x0, y0 + KEY + GAP, a);
  drawKey("S", x0 + KEY + GAP, y0 + KEY + GAP, s);
  drawKey("D", x0 + 2 * (KEY + GAP), y0 + KEY + GAP, d);
}

void drawKey(String label, float x, float y, boolean on) {
  rectMode(CORNER);
  if (on) {
    noStroke();
    for (int i = 4; i > 0; i--) {
      fill(ACCENT, 18);
      rect(x - i * 3, y - i * 3, KEY + i * 6, KEY + i * 6, 12 + i * 3);
    }
    fill(ACCENT);
    stroke(180, 240, 255);
  } else {
    fill(KEY_OFF);
    stroke(60, 66, 86);
  }
  strokeWeight(2);
  rect(x, y, KEY, KEY, 12);

  fill(on ? color(10, 20, 30) : TEXT_DIM);
  textAlign(CENTER, CENTER);
  textSize(22);
  text(label, x + KEY / 2, y + KEY / 2 - 2);
}

void drawWheelBars() {
  float by = height - 32 - 90 - 18;
  drawBar(width - 74, by, vL, "L");
  drawBar(width - 44, by, vR, "R");
}

void drawBar(float x, float y, float v, String label) {
  float h = 90;
  rectMode(CORNER);
  noStroke();
  fill(KEY_OFF);
  rect(x, y, 14, h, 7);

  fill(v >= 0 ? ACCENT : color(255, 140, 80));
  float len = abs(v) * h / 2;
  if (v >= 0) rect(x, y + h / 2 - len, 14, len, 3);
  else rect(x, y + h / 2, 14, len, 3);

  fill(TEXT_DIM);
  textAlign(CENTER, TOP);
  textSize(12);
  text(label, x + 7, y + h + 6);
}

// ---------- input ----------

void keyPressed()  { setKey(key, true); }
void keyReleased() { setKey(key, false); }

void setKey(char k, boolean down) {
  switch (Character.toLowerCase(k)) {
    case 'w': w = down; break;
    case 'a': a = down; break;
    case 's': s = down; break;
    case 'd': d = down; break;
  }
}

void stop() {
  if (port != null) port.write('S');
  super.stop();
}

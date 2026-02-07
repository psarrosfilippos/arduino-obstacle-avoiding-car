// 4WD robot (two motor channels) + HC-SR04 obstacle avoidance
// Extra behavior: tries to avoid getting stuck turning in the same pattern ("anti-loop").
//Psarros Filippos mscdrones80966106

// -------------------- Pin mapping (Arduino -> L298N) --------------------
// Left side (one H-bridge direction inputs)
const int LeftMotorForward  = 6;
const int LeftMotorBackward = 7;

// Right side (one H-bridge direction inputs)
const int RightMotorForward  = 4;
const int RightMotorBackward = 5;

// Enable pins for the two H-bridges (kept HIGH = full speed ON)
const int enA = 8;
const int enB = 3;

// -------------------- Ultrasonic sensor pins --------------------
//TRIG is on 12 and ECHO is on 11
#define trig_pin 12
#define echo_pin 11

// -------------------- Distance thresholds --------------------
// If distance <= DIST_LIMIT_CM -> obstacle detected -> avoidance maneuver
#define DIST_LIMIT_CM 30

// Hysteresis threshold: once we are "clear", we reset burst counter.
#define CLEAR_LIMIT_CM 30

// -------------------- Base timings --------------------
int backMsBase = 250;   // how long we reverse when we see an obstacle
int turnMsBase = 420;   // how long we rotate to turn left/right

// -------------------- Anti-loop state --------------------
// turnHist stores the last turns as bits (1 = Right, 0 = Left). Example: 101100...
uint8_t turnHist = 0;

// histLen shows how many turns are currently stored (up to 8)
uint8_t histLen = 0;

// Used to detect "bursts" of repeated obstacles (robot might be stuck in a tight area)
unsigned long lastObstacleMs = 0;
int obstacleBurst = 0;

void setup() {
  // Motor direction pins
  pinMode(LeftMotorForward, OUTPUT);
  pinMode(LeftMotorBackward, OUTPUT);
  pinMode(RightMotorForward, OUTPUT);
  pinMode(RightMotorBackward, OUTPUT);

  // Motor enable pins
  pinMode(enA, OUTPUT);
  pinMode(enB, OUTPUT);

  // Ultrasonic pins
  pinMode(trig_pin, OUTPUT);
  pinMode(echo_pin, INPUT);

  // Keep both motor channels enabled (full ON)
  digitalWrite(enA, HIGH);
  digitalWrite(enB, HIGH);

  // Seed randomness (A0 left floating helps randomness)
  randomSeed(analogRead(A0));

  // Start safely with motors stopped
  moveStop();
}

void loop() {
  // Read current distance in centimeters
  int d = readDistanceCm();

  // Simple hysteresis idea:
  // when we are clearly far from obstacles, reset the "burst" counter.
  if (d > CLEAR_LIMIT_CM) {
    obstacleBurst = 0;
  }

  // Decision: obstacle or free path
  if (d <= DIST_LIMIT_CM) {
    handleObstacle();
  } else {
    moveForward();
  }

  // Small delay to keep loop stable and reduce sensor spamming
  delay(40);
}

void handleObstacle() {
  unsigned long now = millis();

  // Detect if obstacles are happening repeatedly within a short time window
  // (this usually means the robot is trapped between objects or oscillating).
  if (now - lastObstacleMs < 1500) {
    obstacleBurst++;
  } else {
    obstacleBurst = 1;
  }
  lastObstacleMs = now;

  // Determine if we are likely stuck in a loop:
  // pattern of turns strongly biased one way (isLooping())
  // or many obstacles close together (obstacleBurst)
  bool looping = isLooping() || (obstacleBurst >= 4);

  // If stuck, increase maneuver strength (reverse longer + turn longer)
  int backMs = backMsBase + (looping ? 200 : 0);
  int turnMs = turnMsBase + (looping ? 250 : 0);

  // Basic avoidance sequence: stop -> reverse -> stop
  moveStop();
  delay(120);

  moveBackward();
  delay(backMs);

  moveStop();
  delay(80);

  // Choose turn direction:
  // - Normal case: random left/right
  // - Looping case: bias away from the most frequent recent turn direction
  bool goRight;
  if (!looping) {
    goRight = (random(0, 2) == 1);  // 0 or 1
  } else {
    // countRights() tells how many of the stored turns were to the right.
    // If we've been turning right a lot, go left; if not, go right.
    goRight = (countRights() <= 4);
  }

  // Store the decision in the history
  pushTurn(goRight);

  // Execute the turn (spin in place)
  if (goRight) {
    turnRight(turnMs);
  } else {
    turnLeft(turnMs);
  }

  // Stop briefly after turning to avoid instant re-triggering
  moveStop();
  delay(80);
}

bool isLooping() {
  // Need enough history before calling it a pattern
  if (histLen < 8) return false;

  int rights = countRights();

  // If most of the last 8 turns were the same direction,
  // it's a strong hint the robot is cycling.
  return (rights >= 6 || rights <= 2);
}

int countRights() {
  int c = 0;

  // Count how many bits (among histLen) are 1
  for (int i = 0; i < histLen; i++) {
    if (turnHist & (1 << i)) c++;
  }
  return c;
}

void pushTurn(bool right) {
  // Increase history length until it reaches 8 entries
  if (histLen < 8) histLen++;

  // Shift left to make room for the newest decision
  turnHist = (turnHist << 1) & 0xFF;

  // Put the newest turn in the least significant bit
  if (right) turnHist |= 1;
}

int readDistanceCm() {
  // Send a short trigger pulse (HC-SR04 standard procedure)
  digitalWrite(trig_pin, LOW);
  delayMicroseconds(2);

  digitalWrite(trig_pin, HIGH);
  delayMicroseconds(10);

  digitalWrite(trig_pin, LOW);

  // Measure the echo pulse width (timeout ~25ms)
  // If timeout happens, duration==0, meaning "no echo"
  unsigned long duration = pulseIn(echo_pin, HIGH, 25000UL);

  // Treat "no echo" as far away
  if (duration == 0) return 250;

  // Convert microseconds to cm (speed of sound ~0.0343 cm/us, divide by 2 for round trip)
  return (int)(duration * 0.0343 / 2.0);
}

void moveStop() {
  // Disable both directions on both sides
  digitalWrite(LeftMotorForward, LOW);
  digitalWrite(LeftMotorBackward, LOW);
  digitalWrite(RightMotorForward, LOW);
  digitalWrite(RightMotorBackward, LOW);
}

void moveForward() {
  // Both sides forward
  digitalWrite(LeftMotorForward, HIGH);
  digitalWrite(LeftMotorBackward, LOW);

  digitalWrite(RightMotorForward, HIGH);
  digitalWrite(RightMotorBackward, LOW);
}

void moveBackward() {
  // Both sides backward
  digitalWrite(LeftMotorForward, LOW);
  digitalWrite(LeftMotorBackward, HIGH);

  digitalWrite(RightMotorForward, LOW);
  digitalWrite(RightMotorBackward, HIGH);
}

void turnLeft(int ms) {
  // Spin left: left side backward, right side forward
  digitalWrite(LeftMotorForward, LOW);
  digitalWrite(LeftMotorBackward, HIGH);

  digitalWrite(RightMotorForward, HIGH);
  digitalWrite(RightMotorBackward, LOW);

  delay(ms);
}

void turnRight(int ms) {
  // Spin right: left side forward, right side backward
  digitalWrite(LeftMotorForward, HIGH);
  digitalWrite(LeftMotorBackward, LOW);

  digitalWrite(RightMotorForward, LOW);
  digitalWrite(RightMotorBackward, HIGH);

  delay(ms);
}

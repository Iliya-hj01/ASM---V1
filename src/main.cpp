#include <Arduino.h>

// Forward declarations
bool performHoming(uint8_t tiltPwm = 200, uint8_t risePwm = 110);

// ======= Pins =======
//tilt motor
#define tiltMotorPWM 13 
#define tiltMotorIN2 17 
#define tiltMotorEncoder 11  // PD3 / INT1 — interrupt-driven (MightyCore standard pinout)
#define tiltMotorEncoderINT 1 // INT1 vector

//rise actuator
#define riseActuatorPWM 12
#define riseActuatorIN2 16 // HIGH - opening, LOW - closing
#define riseActuatorPot 27
uint8_t risePositionGap = 5;

// ======= Rise Actuator Test Limits =======
int riseLimitLow = 200;   // lower pot position limit
int riseLimitHigh = 600;  // upper pot position limit

// ======= Tilt Test Limits =======
long tiltLimitLow = 100;     // lower encoder position limit
long tiltLimitHigh = 10000;  // upper encoder position limit


//spin motor
#define spinMotorPWM 3
#define spinMotorIN2 18 

//External button
#define externalButton 10

//not currently used
#define moduleInPlaceSW 14
#define capsuleInPlaceSW 19
#define spinHall 2

// Tilt home microswitch (PC4, active LOW with pull-up)
#define tiltHomeSW 20

// ======= Stage & Sequence Definition =======
struct Stage {
  int      risePosition;   // target pot position (0..1024)
  uint8_t  riseSpeed;      // PWM 0..255
  uint8_t  spinSpeed;      // PWM 0..255
  bool     spinDirection;  // HIGH = CCW, LOW = CW
  long     tiltPosition;   // target encoder count
  uint8_t  tiltSpeed;      // PWM 0..255
  uint32_t duration;       // ms
};

#define NUM_STAGES 5
Stage sequence[NUM_STAGES] = {
  // risePos, riseSpd, spinSpd, spinDir, tiltPos, tiltSpd, duration
  {200, 120, 100, HIGH, 5000,  50, 10000},
  {400, 120, 150, HIGH, 10000, 50, 10000},
  {600, 120, 100, HIGH, 5000,  50, 10000},
  {300, 120, 150, HIGH, 3000,  50, 10000},
  {200, 120, 100, HIGH, 5000,  50, 10000},
};

// ======= Global State =======
bool isHomed = false;
volatile long encoderCount = 0;
bool tiltCurrentDirection = HIGH;
long tiltPositionGap = 5;  // encoder dead-band

//New functions
void moveTilt(uint8_t tiltSpeedMove, bool tiltDirectionMove){
  tiltDirectionMove = !tiltDirectionMove; // reverse physical direction
  if (tiltDirectionMove){
    analogWrite(tiltMotorPWM,(255 - tiltSpeedMove));
  } else {
    analogWrite(tiltMotorPWM,tiltSpeedMove);
  }
  digitalWrite(tiltMotorIN2,tiltDirectionMove);    
}

void stopTilt(){
  analogWrite(tiltMotorPWM,0);
  digitalWrite(tiltMotorIN2,LOW);
}

void stopRise(){
  analogWrite(riseActuatorPWM,0);
  digitalWrite(riseActuatorIN2,LOW);
}

void moveSpin(uint8_t spinSpeedMove, bool spinDirectionMove){
  if (spinDirectionMove){
    analogWrite(spinMotorPWM,(255 - spinSpeedMove));
  } else {
    analogWrite(spinMotorPWM,spinSpeedMove);
  }
  digitalWrite(spinMotorIN2,spinDirectionMove);
}

void stopSpin(){
  analogWrite(spinMotorPWM,0);
  digitalWrite(spinMotorIN2,LOW);
}

void moveRise(uint8_t riseSpeedMove, bool riseDirectionMove){
  if (riseDirectionMove){
    analogWrite(riseActuatorPWM,(255 - riseSpeedMove));
  } else {
    analogWrite(riseActuatorPWM,riseSpeedMove);
  }
  digitalWrite(riseActuatorIN2,riseDirectionMove);    
}

bool risePositionCheck(int risePositionCheckTarget){
  // check whether rise position is within acceptable location, if yes return HIGH, if no return LOW
  int riseCurrentPosition = analogRead(riseActuatorPot);
  int positionDiff = riseCurrentPosition - risePositionCheckTarget;
  if ((positionDiff < risePositionGap) && (positionDiff > (-risePositionGap))){
    return HIGH;
  } else {
    return LOW;
  }
}

void moveRiseToPosition(uint8_t riseSpeedMoveToPosition, int risePositionMoveTarget){
  int riseCurrentPosition = analogRead(riseActuatorPot);
  int positionDiff = riseCurrentPosition - risePositionMoveTarget;
  if (risePositionCheck(risePositionMoveTarget)){
    stopRise();
  } else if (positionDiff >= risePositionGap){
    moveRise(riseSpeedMoveToPosition, LOW); // LOW since the current position is higher than the target
  }else if (positionDiff <= -risePositionGap) {
    moveRise(riseSpeedMoveToPosition, HIGH); // HIGH since the current position is LOWER than the target
  }
}

// ======= Tilt Position Tracking =======
// tiltMovingForward: true when motor physically moves forward (encoder increases)
volatile bool tiltMovingForward = true;

// ISR: called on every CHANGE edge of PD3 (INT1).
// Increment / decrement encoderCount based on current motor direction.
void tiltEncoderISR() {
  if (tiltMovingForward) {
    encoderCount++;
  } else {
    encoderCount--;
  }
}

// Kept as a no-op shim so existing call sites still compile.
// All counting now happens in tiltEncoderISR().
inline void pollEncoder() { /* handled by INT1 ISR */ }

bool tiltPositionCheck(long targetPos) {
  long diff = encoderCount - targetPos;
  return (diff < tiltPositionGap) && (diff > -tiltPositionGap);
}

void moveTiltToPosition(uint8_t speed, long targetPos) {
  long diff = encoderCount - targetPos;
  if (tiltPositionCheck(targetPos)) {
    stopTilt();
  } else if (diff > 0) {
    // encoder above target → need to decrease → move toward home (HIGH)
    tiltMovingForward = false;
    moveTilt(speed, HIGH);
  } else {
    // encoder below target → need to increase → move forward (LOW)
    tiltMovingForward = true;
    moveTilt(speed, LOW);
  }
}

// ======= Run Sequence =======
void runSequence() {
  performHoming();
  Serial.println("--- Sequence start ---");
  uint32_t seqStart = millis();
  for (uint8_t i = 0; i < NUM_STAGES; i++) {
    Stage &s = sequence[i];
    uint32_t stageStart = millis();
    bool riseReached = false;
    bool tiltReached = false;

    Serial.print("Stage ");
    Serial.print(i);
    Serial.print(" start | rise: ");
    Serial.print(analogRead(riseActuatorPot));
    Serial.print(" tilt: ");
    Serial.println(encoderCount);

    moveSpin(s.spinSpeed, s.spinDirection);

    while (true) {
      pollEncoder();

      if (!riseReached) {
        moveRiseToPosition(s.riseSpeed, s.risePosition);
        riseReached = risePositionCheck(s.risePosition);
        if (riseReached) stopRise();
      }

      if (!tiltReached) {
        moveTiltToPosition(s.tiltSpeed, s.tiltPosition);
        tiltReached = tiltPositionCheck(s.tiltPosition);
        if (tiltReached) stopTilt();
      }

      if (riseReached && tiltReached) {
        stopTilt();
        stopRise();
        // Hold position until duration expires
        if (millis() - stageStart >= s.duration) {
          break;
        }
      }

      // Duration expired during movement — skip to next stage
      if (millis() - stageStart >= s.duration) {
        riseReached = false; // mark as not reached for end print
        break;
      }
    }

    Serial.print("Stage ");
    Serial.print(i);
    Serial.print(" end (");
    Serial.print(millis() - stageStart);
    Serial.print("ms) rise: ");
    Serial.print(analogRead(riseActuatorPot));
    Serial.print(" tilt: ");
    Serial.print(encoderCount);
    Serial.print(" - ");
    Serial.println(riseReached && tiltReached ? "REACHED" : "TIMEOUT");
  }

  stopTilt();
  stopRise();
  stopSpin();
  Serial.print("--- Sequence done (");
  Serial.print(millis() - seqStart);
  Serial.println("ms) ---");
}

// ======= Tilt Button Test =======
bool tiltDirection = HIGH;
bool lastButtonState = HIGH;
uint32_t lastDebounceTime = 0;
const uint32_t debounceDelay = 200;

uint32_t lastPrintTime = 0;

void tiltButtonTest(uint8_t tiltSpeed = 1) {
  bool buttonState = digitalRead(externalButton);

  // Debounced falling-edge detection (button pressed = LOW with INPUT_PULLUP)
  if (buttonState == LOW && lastButtonState == HIGH && (millis() - lastDebounceTime) > debounceDelay) {
    tiltDirection = !tiltDirection;
    lastDebounceTime = millis();
    Serial.print("Direction changed to: ");
    Serial.println(tiltDirection ? "HIGH" : "LOW");
  }
  lastButtonState = buttonState;

  // Keep direction flag in sync for the ISR
  // (tiltDirection HIGH = reversed in moveTilt(), so forward = !tiltDirection)
  tiltMovingForward = !tiltDirection;
  moveTilt(tiltSpeed, tiltDirection);

  // Encoder counting handled by INT1 ISR (tiltEncoderISR)
  if (millis() - lastPrintTime >= 1000) {
    Serial.print("Encoder: ");
    Serial.println(encoderCount);
    lastPrintTime = millis();
  }
}

// ======= Spin Motor Test =======
bool spinDirection = HIGH;
bool spinLastButtonState = HIGH;
uint32_t spinLastDebounceTime = 0;
uint32_t spinLastPrintTime = 0;

void spinMotorTest(uint8_t spinSpeed = 150) {
  bool buttonState = digitalRead(externalButton);

  if (buttonState == LOW && spinLastButtonState == HIGH && (millis() - spinLastDebounceTime) > debounceDelay) {
    spinDirection = !spinDirection;
    spinLastDebounceTime = millis();
    Serial.print("Spin direction changed to: ");
    Serial.println(spinDirection ? "CCW" : "CW");
  }
  spinLastButtonState = buttonState;

  moveSpin(spinSpeed, spinDirection);

  if (millis() - spinLastPrintTime >= 1000) {
    Serial.print("Spin running at PWM ");
    Serial.print(spinSpeed);
    Serial.print("  dir: ");
    Serial.println(spinDirection ? "CCW" : "CW");
    spinLastPrintTime = millis();
  }
}

// ======= Rise Direction Pre-Test =======
bool risePreTestDone = false;

void riseDirectionPreTest(uint8_t testSpeed = 120, uint32_t testDuration = 2000) {
  int posBefore, posAfter;

  // --- Test direction HIGH ---
  posBefore = analogRead(riseActuatorPot);
  Serial.print("Testing dir HIGH... start pos: ");
  Serial.println(posBefore);
  uint32_t t0 = millis();
  while (millis() - t0 < testDuration) {
    moveRise(testSpeed, HIGH);
  }
  stopRise();
  delay(200);
  posAfter = analogRead(riseActuatorPot);
  Serial.print("Dir HIGH end pos: ");
  Serial.print(posAfter);
  Serial.print("  delta: ");
  Serial.println(posAfter - posBefore);

  // --- Test direction LOW ---
  posBefore = analogRead(riseActuatorPot);
  Serial.print("Testing dir LOW... start pos: ");
  Serial.println(posBefore);
  t0 = millis();
  while (millis() - t0 < testDuration) {
    moveRise(testSpeed, LOW);
  }
  stopRise();
  delay(200);
  posAfter = analogRead(riseActuatorPot);
  Serial.print("Dir LOW end pos: ");
  Serial.print(posAfter);
  Serial.print("  delta: ");
  Serial.println(posAfter - posBefore);

  Serial.println("--- Pre-test done. Check deltas above. ---");
  Serial.println("HIGH delta should be positive (pos increases).");
  Serial.println("LOW delta should be negative (pos decreases).");
  Serial.println("If reversed, swap HIGH/LOW in moveRiseToPosition().");
  risePreTestDone = true;
}

// ======= Rise Actuator Test =======
bool riseTestMovingUp = true;
uint32_t riseLastPrintTime = 0;

void riseActuatorTest(uint8_t riseTestSpeed = 50) {
  int pos = analogRead(riseActuatorPot);

  // Switch direction at limits
  if (riseTestMovingUp && risePositionCheck(riseLimitHigh)) {
    riseTestMovingUp = false;
  } else if (!riseTestMovingUp && risePositionCheck(riseLimitLow)) {
    riseTestMovingUp = true;
  }

  moveRiseToPosition(riseTestSpeed, riseTestMovingUp ? riseLimitHigh : riseLimitLow);

  if (millis() - riseLastPrintTime >= 1000) {
    Serial.print("Rise pos: ");
    Serial.print(pos);
    Serial.print("  target: ");
    Serial.print(riseTestMovingUp ? riseLimitHigh : riseLimitLow);
    Serial.print("  dir: ");
    Serial.println(riseTestMovingUp ? "UP" : "DOWN");
    riseLastPrintTime = millis();
  }
}

// ======= Tilt Actuator Test =======
bool tiltTestMovingUp = true;
uint32_t tiltTestLastPrintTime = 0;

void tiltActuatorTest(uint8_t tiltTestSpeed = 200) {
  pollEncoder();

  // Switch direction at limits
  if (tiltTestMovingUp && tiltPositionCheck(tiltLimitHigh)) {
    tiltTestMovingUp = false;
  } else if (!tiltTestMovingUp && tiltPositionCheck(tiltLimitLow)) {
    tiltTestMovingUp = true;
  }

  long target = tiltTestMovingUp ? tiltLimitHigh : tiltLimitLow;
  moveTiltToPosition(tiltTestSpeed, target);

  if (millis() - tiltTestLastPrintTime >= 100) {
    Serial.print("Tilt pos: ");
    Serial.print(encoderCount);
    Serial.print("  target: ");
    Serial.print(target);
    Serial.print("  motorDir: ");
    Serial.print(tiltMovingForward ? "FWD" : "HOME");
    Serial.print("  dir: ");
    Serial.println(tiltTestMovingUp ? "UP" : "DOWN");
    tiltTestLastPrintTime = millis();
  }
}

// ======= Tilt Direction Test =======
uint32_t tiltDirTestLastPrintTime = 0;

void tiltDirectionTest(uint8_t speed = 150) {
  pollEncoder();
  tiltMovingForward = true;
  moveTilt(speed, LOW);

  if (millis() - tiltDirTestLastPrintTime >= 500) {
    Serial.print("Tilt dir LOW | encoder: ");
    Serial.println(encoderCount);
    tiltDirTestLastPrintTime = millis();
  }
}

// ======= Tilt Home Switch Test =======
uint32_t tiltSwLastPrintTime = 0;

void tiltHomeSwitchTest() {
  if (millis() - tiltSwLastPrintTime >= 500) {
    bool state = digitalRead(tiltHomeSW);
    Serial.print("Tilt home switch: ");
    Serial.println(state == HIGH ? "ENGAGED" : "OPEN");
    tiltSwLastPrintTime = millis();
  }
}

// ======= Homing =======
bool performHoming(uint8_t tiltPwm = 200, uint8_t risePwm = 110) {
  bool tiltDirectionHoming = HIGH;  // direction toward home switch
  uint32_t timeoutDuration = 10000; // safety timeout in ms
  bool tiltHomed = false;
  bool riseHomed = false;

  Serial.println("Homing...");
  uint32_t t0 = millis();

  while (millis() - t0 < timeoutDuration) {
    // Tilt: move until home switch engages (HIGH = engaged)
    if (!tiltHomed) {
      if (digitalRead(tiltHomeSW) == HIGH) {
        stopTilt();
        encoderCount = 0;  // reset encoder origin
        tiltHomed = true;
        Serial.println("Tilt homed.");
      } else {
        moveTilt(tiltPwm, tiltDirectionHoming);
      }
    }

    // Rise: move to lower limit
    if (!riseHomed) {
      moveRiseToPosition(risePwm, riseLimitLow);
      riseHomed = risePositionCheck(riseLimitLow);
      if (riseHomed) {
        stopRise();
        Serial.println("Rise homed.");
      }
    }

    if (tiltHomed && riseHomed) {
      break;
    }
  }

  stopTilt();
  stopRise();

  if (!tiltHomed || !riseHomed) {
    Serial.println("Homing TIMEOUT!");
    isHomed = false;
    return false;
  }

  isHomed = true;
  Serial.println("Homing complete.");
  return true;
}

void setup() {
  // tilt motor
  pinMode(tiltMotorPWM, OUTPUT);
  pinMode(tiltMotorIN2, OUTPUT);
  pinMode(tiltMotorEncoder, INPUT_PULLUP);  // PD3 / INT1
  // Attach interrupt on every edge so we count both rising and falling transitions
  attachInterrupt(tiltMotorEncoderINT, tiltEncoderISR, CHANGE);
  // rise actuator
  pinMode(riseActuatorPWM, OUTPUT);
  pinMode(riseActuatorIN2, OUTPUT);
  pinMode(riseActuatorPot, INPUT);
  // spin motor
  pinMode(spinMotorPWM, OUTPUT);
  pinMode(spinMotorIN2, OUTPUT);
  // button
  pinMode(externalButton, INPUT_PULLUP);
  // tilt home switch (active HIGH, external pull-down)
  pinMode(tiltHomeSW, INPUT);

  Serial.begin(9600);

  performHoming();

}

bool seqLastButtonState = HIGH;

void loop() {
  bool btnState = digitalRead(externalButton);
  if (btnState == LOW && seqLastButtonState == HIGH) {
    runSequence();
  }
  seqLastButtonState = btnState;
  // tiltDirectionTest();
  tiltActuatorTest();
  //riseActuatorTest();
  // tiltHomeSwitchTest();
}
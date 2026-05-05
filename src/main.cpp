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
  {600, 120, 0, HIGH, 12000,  100, 5000},
  {600, 70, 150, HIGH, 12000, 100, 5000},
  {350, 70, 100, HIGH, 12000,  100, 5000},
  {300, 70, 150, HIGH, 12000,  100, 5000},
  {200, 70, 100, HIGH, 500,  100, 5000},
};

// ======= Global State =======
bool isHomed = false;
volatile long encoderCount = 0;
bool tiltCurrentDirection = HIGH;
long tiltPositionGap = 50;  // encoder dead-band

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
  // Active brake: drive both H-bridge inputs HIGH to short the motor terminals.
  digitalWrite(tiltMotorIN2, HIGH);
  analogWrite(tiltMotorPWM, 255);
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

// ======= Tilt approach tuning =======
long tiltSlowZone = 800;     // counts before target where we begin slowing down
uint8_t tiltMinSpeed = 30;   // minimum PWM near the target

// Gravity compensation: in the direction gravity assists, PWM ramping doesn't
// slow the load (motor freewheels / regenerates). Engage the brake early so
// the load has room to decelerate against gravity before reaching the target.
// `tiltGravityDecreasesCount` = true means gravity pulls encoderCount downward
// (toward 0). Set false if gravity pulls counts upward instead.
bool tiltGravityDecreasesCount = true;
long tiltBrakeEarlyGravity = 250;  // counts before target to brake in gravity dir

void moveTiltToPosition(uint8_t speed, long targetPos) {
  long diff = encoderCount - targetPos;
  long absDiff = diff < 0 ? -diff : diff;

  if (tiltPositionCheck(targetPos)) {
    stopTilt();
    return;
  }

  bool needToDecrease = (diff > 0);
  bool gravityAssisted = (needToDecrease == tiltGravityDecreasesCount);

  // In the gravity-assisted direction, PWM ramping is ineffective (gravity
  // overdrives the motor). Brake early and let the load coast to the target.
  if (gravityAssisted && absDiff < tiltBrakeEarlyGravity) {
    stopTilt();
    return;
  }

  // Linear ramp: full `speed` outside slow zone, down to tiltMinSpeed at target.
  uint8_t effSpeed = speed;
  if (speed > tiltMinSpeed && absDiff < tiltSlowZone) {
    effSpeed = tiltMinSpeed +
               (uint8_t)(((long)(speed - tiltMinSpeed) * absDiff) / tiltSlowZone);
  }

  if (needToDecrease) {
    // encoder above target → need to decrease → move toward home (HIGH)
    tiltMovingForward = false;
    moveTilt(effSpeed, HIGH);
  } else {
    // encoder below target → need to increase → move forward (LOW)
    tiltMovingForward = true;
    moveTilt(effSpeed, LOW);
  }
}

// ======= Run Sequence =======
void runSequence() {
  performHoming();
  for (uint8_t i = 0; i < NUM_STAGES; i++) {
    Stage &s = sequence[i];
    uint32_t stageStart = millis();
    bool riseReached = false;
    bool tiltReached = false;

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
        break;
      }
    }
  }

  stopTilt();
  stopRise();
  stopSpin();
}

// ======= Tilt Button Test =======
bool tiltDirection = HIGH;
bool lastButtonState = HIGH;
uint32_t lastDebounceTime = 0;
const uint32_t debounceDelay = 200;

void tiltButtonTest(uint8_t tiltSpeed = 1) {
  bool buttonState = digitalRead(externalButton);

  // Debounced falling-edge detection (button pressed = LOW with INPUT_PULLUP)
  if (buttonState == LOW && lastButtonState == HIGH && (millis() - lastDebounceTime) > debounceDelay) {
    tiltDirection = !tiltDirection;
    lastDebounceTime = millis();
  }
  lastButtonState = buttonState;

  // Keep direction flag in sync for the ISR
  // (tiltDirection HIGH = reversed in moveTilt(), so forward = !tiltDirection)
  tiltMovingForward = !tiltDirection;
  moveTilt(tiltSpeed, tiltDirection);
}

// ======= Spin Motor Test =======
bool spinDirection = HIGH;
bool spinLastButtonState = HIGH;
uint32_t spinLastDebounceTime = 0;

void spinMotorTest(uint8_t spinSpeed = 150) {
  bool buttonState = digitalRead(externalButton);

  if (buttonState == LOW && spinLastButtonState == HIGH && (millis() - spinLastDebounceTime) > debounceDelay) {
    spinDirection = !spinDirection;
    spinLastDebounceTime = millis();
  }
  spinLastButtonState = buttonState;

  moveSpin(spinSpeed, spinDirection);
}

// ======= Rise Direction Pre-Test =======
bool risePreTestDone = false;

void riseDirectionPreTest(uint8_t testSpeed = 120, uint32_t testDuration = 2000) {
  // --- Test direction HIGH ---
  uint32_t t0 = millis();
  while (millis() - t0 < testDuration) {
    moveRise(testSpeed, HIGH);
  }
  stopRise();
  delay(200);

  // --- Test direction LOW ---
  t0 = millis();
  while (millis() - t0 < testDuration) {
    moveRise(testSpeed, LOW);
  }
  stopRise();
  delay(200);

  risePreTestDone = true;
}

// ======= Rise Actuator Test =======
bool riseTestMovingUp = true;

void riseActuatorTest(uint8_t riseTestSpeed = 50) {
  // Switch direction at limits
  if (riseTestMovingUp && risePositionCheck(riseLimitHigh)) {
    riseTestMovingUp = false;
  } else if (!riseTestMovingUp && risePositionCheck(riseLimitLow)) {
    riseTestMovingUp = true;
  }

  moveRiseToPosition(riseTestSpeed, riseTestMovingUp ? riseLimitHigh : riseLimitLow);
}

// ======= Tilt Actuator Test =======
bool tiltTestMovingUp = true;

void tiltActuatorTest(uint8_t tiltTestSpeed = 150) {
  static bool tiltResting = false;
  static uint32_t tiltRestStart = 0;

  pollEncoder();

  if (tiltResting) {
    stopTilt();
    if (millis() - tiltRestStart >= 3000) {
      tiltResting = false;
    }
    return;
  }

  // Switch direction at limits; begin 3-second rest
  if (tiltTestMovingUp && tiltPositionCheck(tiltLimitHigh)) {
    tiltTestMovingUp = false;
    stopTilt();
    tiltResting = true;
    tiltRestStart = millis();
    return;
  } else if (!tiltTestMovingUp && tiltPositionCheck(tiltLimitLow)) {
    tiltTestMovingUp = true;
    stopTilt();
    tiltResting = true;
    tiltRestStart = millis();
    return;
  }

  long target = tiltTestMovingUp ? tiltLimitHigh : tiltLimitLow;
  moveTiltToPosition(tiltTestSpeed, target);
}

// ======= Tilt Direction Test =======
void tiltDirectionTest(uint8_t speed = 150) {
  pollEncoder();
  tiltMovingForward = true;
  moveTilt(speed, LOW);
}

// ======= Tilt Home Switch Test =======
void tiltHomeSwitchTest() {
  // No-op: state is reported by printState()
}

// ======= State Printing =======
uint32_t printStateInterval = 500; // ms; configurable
uint32_t printStateLastTime = 0;

void printState() {
  if (millis() - printStateLastTime < printStateInterval) return;
  printStateLastTime = millis();

  Serial.print("t=");
  Serial.print(millis());
  Serial.print(" tilt=");
  Serial.print(encoderCount);
  Serial.print(" tiltFwd=");
  Serial.print(tiltMovingForward ? 1 : 0);
  Serial.print(" rise=");
  Serial.print(analogRead(riseActuatorPot));
  Serial.print(" homeSW=");
  Serial.print(digitalRead(tiltHomeSW) == HIGH ? "ENG" : "OPEN");
  Serial.print(" btn=");
  Serial.print(digitalRead(externalButton) == LOW ? "DOWN" : "UP");
  Serial.print(" homed=");
  Serial.println(isHomed ? 1 : 0);
}

// ======= Homing =======
bool performHoming(uint8_t tiltPwm = 200, uint8_t risePwm = 110) {
  bool tiltDirectionHoming = HIGH;  // direction toward home switch
  uint32_t timeoutDuration = 10000; // safety timeout in ms
  bool tiltHomed = false;
  bool riseHomed = false;

  uint32_t t0 = millis();

  while (millis() - t0 < timeoutDuration) {
    // Tilt: move until home switch engages (HIGH = engaged)
    if (!tiltHomed) {
      if (digitalRead(tiltHomeSW) == HIGH) {
        stopTilt();
        encoderCount = 0;  // reset encoder origin
        tiltHomed = true;
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
      }
    }

    if (tiltHomed && riseHomed) {
      break;
    }
  }

  stopTilt();
  stopRise();

  if (!tiltHomed || !riseHomed) {
    isHomed = false;
    return false;
  }

  isHomed = true;
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
  // tiltActuatorTest();
  //riseActuatorTest();
  // tiltHomeSwitchTest();
  printState();
}
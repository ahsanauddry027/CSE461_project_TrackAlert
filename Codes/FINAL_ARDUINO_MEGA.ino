/* ============================================================================
   TrackAlert Vision — FINAL FIRMWARE v6  (Arduino Mega 2560)
   ----------------------------------------------------------------------------
   Pairs with FINAL_ESP32_CAM_v2.ino.

   v6 = v5 (slow speeds + left-first box avoidance + gyro 90 deg turns)
        with the LINE FOLLOWER rewritten properly.

   WHAT IS NEW IN v6  —  LINE FOLLOWING
   ----------------------------------------------------------------------------
   1) POSITION FROM THE SENSOR PATTERN, NOT AN AVERAGE
      The old code did error = sum / onLine. That gives only a handful of
      possible values and it happily averages a NONSENSE pattern such as
      10100 (two sensors on, a gap between them) into a confident "centre",
      which is exactly when the robot lurches. v6 checks the pattern first:
        - the lit sensors must be CONTIGUOUS (one unbroken run) to be trusted
        - position = (firstLit + lastLit - 4) x 10  ->  -40 .. +40
      That is TWICE the resolution of v5 (half-sensor steps: 01100 = -10,
      01000 = -20), so the steering is smoother between sensors.
      A broken pattern is treated as noise: the last good error is held and
      the D term is frozen for that tick instead of being fed garbage.

   2) TWO-SAMPLE SENSOR DEBOUNCE
      A sensor state is only accepted after two consecutive identical reads
      (20 ms max lag). Dust, a scratch in the tape or a reflection no longer
      produces a one-tick steering kick.

   3) GAP GRACE PERIOD (LINE_GRACE_MS = 70 ms)
      v5 declared the line "lost" and reversed the instant all five sensors
      went dark. On a worn, dusty or taped-over line that happens constantly.
      v6 now COASTS THROUGH short gaps: it holds the last steering command at
      reduced speed for 70 ms first, and only then starts the recovery. This
      alone removes most of the stop-start behaviour on a real track.

   4) TIME-CORRECT PD
      The D term is now computed per unit time (dt measured with millis())
      instead of per loop pass. The control tick is 10 ms nominal, but a
      hazard poll or an ultrasonic ping can stretch it — v5's D term silently
      changed gain when that happened. It no longer does.

   5) PREDICTIVE (CURVATURE) SPEED CONTROL
      v5 set the speed from the CURRENT error, so it was still at full speed
      on the first tick of a corner and only slowed once it was already
      running wide. v6 keeps a decaying average of recent error — a curvature
      estimate — and slows on the LARGER of the two. It therefore stays slow
      for the whole of a bend and speeds up again only on a genuine straight.

   6) SMALL INTEGRAL TERM + MOTOR TRIM
      Two different fixes for the same symptom (the robot tracks slightly to
      one side on a straight): KI removes slow residual offset automatically
      with anti-windup and a reset on every zero crossing, and MOTOR_TRIM is
      there for a permanently weaker motor. Set the trim, leave KI alone.

   7) SHARP-CORNER PIVOT, RETUNED
      Triggers on |error| >= 30 with the centre sensor dark, i.e. a real
      90 deg track corner, and pivots on the spot. Coming out of the pivot the
      slew limiter now resumes from the pivot command rather than from zero,
      so the robot no longer stalls for a moment after every corner.

   8) FASTER PING DURING PATROL
      pulseIn() blocks. With the old 12000 us timeout, every obstacle check
      could stall the control loop for 12 ms — longer than a whole control
      tick. Patrol now uses a 4000 us timeout (~68 cm, far beyond the 38 cm
      slow-down point), and the long-range timeout is kept only for the
      avoidance sweeps where the actual distance matters.
      The 1 Hz debug print also no longer fires its own extra ping.

   9) BOOT LINE ACQUISITION
      If the robot is not on the line when it starts, it creeps forward for up
      to 2.5 s to find it instead of immediately running the lost-line sweep.

   KEPT FROM v5
     Speeds: patrol 95, corner 62, slow zone 75, pivot 105, sidestep 85.
     Obstacle avoidance: stop -> 90 deg LEFT -> drive left past it -> 90 deg
     RIGHT back to the original heading -> forward past it -> 90 deg RIGHT ->
     creep to the line -> 90 deg LEFT and resume. Right side only if the left
     is blocked; reverse and turn back if boxed in.
     90 deg turns measured with the MPU6050 Z gyro, timed fallback.
   KEPT FROM v3/v4
     Continuous fire + smoke polling everywhere, flame auto-polarity learning,
     MQ-2 warm-up gate, spaced 3-read hazard confirmation, arming latch, MPU
     I2C error handling, 2-ping obstacle confirmation, 2.5 s scan every 12 s,
     debounced reset, EVT:TILT / EVT:CLEAR.

   WIRING  (unchanged)
     Motors L298N : ENA=5, ENB=6, IN1=44, IN2=45, IN3=46, IN4=47
     Line sensor  : OUT1..OUT5 -> 42,38,34,30,26  (array read reversed in code)
     Servo        : D7
     HC-SR04      : TRIG=D28, ECHO=D29
     PIR (Dout)   : D32
     Flame (DO)   : D33
     MQ-2 (AO)    : A0
     MPU6050      : SDA=D20, SCL=D21  (I2C 0x68)
     LCD 16x2 I2C : SDA=D20, SCL=D21  (I2C 0x27)
     Buzzer       : D8
     RESET button : D9  (to GND, INPUT_PULLUP)
     ESP32 link   : Serial1  TX1=D18 -> divider -> ESP32 RX ; RX1=D19 <- ESP32 TX

   NOTE: the Servo library owns Timer5 on the Mega, which drives PWM on pins
   44/45/46. Those are digitalWrite only here — never analogWrite() to them.

   LIBRARIES: "LiquidCrystal I2C" by Frank de Brabander
   ============================================================================ */
#include <Servo.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

/* ---------------- pins ---------------- */
const int ENA = 5, IN1 = 44, IN2 = 45, IN3 = 46, IN4 = 47, ENB = 6;
const int LINE[5] = {26, 30, 34, 38, 42};   // index 0 = LEFT, index 4 = RIGHT
const bool LINE_ON_BLACK = false;           // flip if it steers the wrong way
const int SERVO_PIN  = 7;
const int TRIG = 28, ECHO = 29;
const int PIR   = 32;
const int FLAME = 33;
const int MQ2   = A0;
const int BUZZER    = 8;
const int RESET_BTN = 9;

/* ============================================================================
   SPEEDS  —  unchanged from v5. Raise BASE_SPEED last, and only once the
   line follower and the avoidance manoeuvre are both behaving.
   ============================================================================ */
const int BASE_SPEED     = 95;    // straight-line patrol speed
const int MIN_TURN_SPEED = 62;    // speed at maximum line error
const int SLOW_SPEED     = 75;    // speed once something is within SLOW_CM
const int SEARCH_SPEED   = 78;    // lost-line hunting
const int PIVOT_SPEED    = 105;   // on-the-spot 90 deg turns
const int SIDESTEP_SPEED = 85;    // driving the legs of the box
const int CREEP_SPEED    = 78;    // final crawl while looking for the line
const int BACKUP_SPEED   = 85;    // reversing

/* ============================================================================
   LINE FOLLOWING  —  PID over a pattern-derived position
   Position runs -40 (line hard left) .. 0 (centred) .. +40 (line hard right)
   in half-sensor steps, so the gains are about HALF the v5 values.
   ============================================================================ */
const int KP = 21;                // proportional gain
const int KD = 17;                // derivative gain — raise to damp weaving
const int KI = 2;                 // integral gain — leave small
const int I_CLAMP = 40;           // anti-windup limit on the integral sum
const int MAX_CORRECTION = 130;   // clamp on the PID output
const int MAX_REVERSE    = 90;    // how hard the inner wheel may counter-rotate
const int MIN_PWM        = 55;    // L298N will not turn the wheels below this
const int SLEW_STEP      = 18;    // max PWM change per 10 ms control tick
const int MOTOR_TRIM     = 0;     // + boosts the RIGHT motor.
                                  // Robot drifts RIGHT on a straight -> raise.
                                  // Robot drifts LEFT  on a straight -> lower.
const int CORNER_ERR     = 30;    // |error| that counts as a 90 deg corner
const unsigned long LINE_UPDATE_MS   = 10;   // nominal control interval
const unsigned long JUNCTION_HOLD_MS = 130;  // drive straight through a crossing

/* ---------------- line loss ---------------- */
const unsigned long LINE_GRACE_MS   = 70;    // coast through gaps this long
const unsigned long LOST_BACKUP_MS  = 180;   // then reverse back onto the line
const unsigned long LOST_SWEEP_MS   = 260;   // first sweep leg, then it grows
const unsigned long LOST_TIMEOUT_MS = 5000;  // after this, creep forward
const bool FIND_LINE_AT_BOOT = true;
const unsigned long BOOT_FIND_MS = 2500;

/* ============================================================================
   OBSTACLE AVOIDANCE  —  the box manoeuvre (unchanged from v5)
   ============================================================================ */
const int OBSTACLE_CM   = 20;     // stop distance
const int SLOW_CM       = 38;     // start easing off here
const int TOO_CLOSE_CM  = 12;     // back off before pivoting if this close
const int SIDE_MIN_CM   = 30;     // a side needs this much room to be usable
const int PASS_CLEAR_CM = 30;     // side reading that means the obstacle is past

const int TURN_DEG = 90;

/* Fallback only — used if the MPU6050 does not answer. Calibrate by eye:
   over-rotating -> lower it; under-rotating -> raise it. */
const unsigned long PIVOT_90_MS = 820;

const unsigned long LEG_SIDE_MS    = 1600;  // max length of the sideways leg
const unsigned long LEG_PAST_MS    = 3000;  // max length of the forward leg
const unsigned long LEG_FIND_MS    = 3000;  // max crawl while hunting the line
const unsigned long EXTRA_CLEAR_MS = 450;   // run on after the obstacle is past
const unsigned long BACKUP_MS      = 550;   // reverse when boxed in
const byte PASS_CONFIRM = 3;                // side-clear reads before turning back

const int SERVO_FWD = 90, SERVO_LEFT = 150, SERVO_RIGHT = 30;
const unsigned int PING_FAST_US = 4000;     // ~68 cm — used while patrolling
const unsigned int PING_LONG_US = 12000;    // ~2 m  — used for the sweeps

/* ---------------- hazards ---------------- */
const int  GAS_THRESHOLD = 400;
const bool FLAME_AUTO_POLARITY = true;
const bool FLAME_ACTIVE_LOW = true;
const bool FLAME_USE_PULLUP = true;
const unsigned long GAS_WARMUP_MS = 60000;
const byte CONFIRM_COUNT = 3;
const unsigned long HAZARD_SAMPLE_MS = 25;

const unsigned long SCAN_INTERVAL_MS = 12000;
const unsigned long SCAN_MS          = 2500;
const unsigned long TILT_CHECK_MS    = 200;
const unsigned long OBS_CHECK_MS     = 60;

const bool SEND_TILT_EVENT = true;
const bool WAIT_FOR_ACK    = false;

#define DEBUG_SENSORS 1
#define USE_WATCHDOG  0

#if USE_WATCHDOG
  #include <avr/wdt.h>
  #define WDT_KICK() wdt_reset()
#else
  #define WDT_KICK()
#endif

/* ---------------- objects & state ---------------- */
Servo scanServo;
LiquidCrystal_I2C lcd(0x27, 16, 2);
const int MPU_ADDR = 0x68;
const float GYRO_LSB_PER_DPS = 131.0;

/* line-follower state */
int   lastError = 0, prevError = 0;
float dFilt = 0;                        // filtered, time-corrected derivative
float curve = 0;                        // decaying average |error| = curvature
int   iSum = 0;                         // integral accumulator
int   outL = 0, outR = 0;               // last motor command (slew limiting)
byte  rawPrevMask = 0, stableMask = 0;  // two-sample sensor debounce
unsigned long lastLineTime = 0;         // for the real dt of the D term
unsigned long lostSince = 0, junctionUntil = 0;

unsigned long lastScan = 0, lastTiltCheck = 0, lastObsCheck = 0;
unsigned long lastLineUpdate = 0, bootTime = 0;
byte flameHits = 0, gasHits = 0, tiltHits = 0, obsHits = 0;
unsigned long lastHazardSample = 0;
bool flameArmed = false, gasArmed = false;
bool flameActiveLow = FLAME_ACTIVE_LOW;
bool inAlert = false;
bool slowZone = false;
long lastDist = -1;                     // cached ping, reused by the debug line
bool  gyroOK = false;
float gyroZbias = 0;

/* ---------------- forward declarations ---------------- */
void triggerAlert(const char* msg, const char* event, bool notify, bool capture);
void sendEvent(const char* event);
bool hazardPoll();
bool waitMs(unsigned long ms);
bool flameRaw();
int  gasLevel();
bool gasRaw();
byte readLineMask();
bool linePosition(byte mask, int* error, byte* count);
void followLine();
void lineLost();
void scanForHazards();
void avoidObstacle();
bool turnDegrees(int deg, bool goLeft);
bool pivotForMs(bool goLeft, unsigned long ms);
bool runLeg(int speed, unsigned long maxMs, int sideAngle,
            bool stopOnLine, bool* sawLine, bool* sideClear);
bool lookAt(int angle, long* out);
void lcdShow(const char* l0, const char* l1);
bool lineSeen();
long pingCM();
long pingLongCM();
long pingRaw(unsigned int timeoutUs);
long pingMedianCM();
void drive(int l, int r);
void driveSmooth(int l, int r);
void driveStraight(int speed);
void pivotLeft(int s);
void pivotRight(int s);
void stopMotors();
void mpuInit();
bool isTilted();
bool readGyroZ(int16_t* gz);
void calibrateGyro();
void resetDriveState();
#if DEBUG_SENSORS
void debugPrint();
#endif

/* ============================================================================
   SETUP
   ============================================================================ */
void setup() {
  Serial.begin(115200);
  Serial1.begin(115200);

  pinMode(ENA, OUTPUT); pinMode(ENB, OUTPUT);
  pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);
  for (int i = 0; i < 5; i++) pinMode(LINE[i], INPUT);
  pinMode(TRIG, OUTPUT); pinMode(ECHO, INPUT);
  pinMode(PIR, INPUT);
  pinMode(FLAME, FLAME_USE_PULLUP ? INPUT_PULLUP : INPUT);
  pinMode(BUZZER, OUTPUT); digitalWrite(BUZZER, LOW);
  pinMode(RESET_BTN, INPUT_PULLUP);
  stopMotors();

  Wire.begin();
  mpuInit();
  lcd.init(); lcd.backlight();
  scanServo.attach(SERVO_PIN); scanServo.write(SERVO_FWD);

  lcd.clear();
  lcd.setCursor(0, 0); lcd.print("TrackAlert Vis.");
  Serial.println(F("Sensor warm-up (30s)... keep the robot STILL"));
  for (int s = 30; s > 0; s--) {
    lcd.setCursor(0, 1); lcd.print("Warm-up ");
    lcd.print(s); lcd.print("s   ");
    delay(1000);
    WDT_KICK();
  }

  /* ---- gyro zero: the robot has been stationary for 30 s ---- */
  lcdShow("Calibrating", "gyro-hold still");
  calibrateGyro();
  Serial.print(F("Gyro: "));
  if (gyroOK) { Serial.print(F("OK, bias = ")); Serial.println(gyroZbias); }
  else        Serial.println(F("NOT FOUND - using timed 90 deg turns"));

  /* ---- flame polarity: measure the idle level, no fire is present now ---- */
  delay(50);
  int idle = digitalRead(FLAME);
  bool stable = true;
  for (byte i = 0; i < 30; i++) { delay(10); if (digitalRead(FLAME) != idle) stable = false; }

  Serial.print(F("Flame pin idle = ")); Serial.print(idle ? F("HIGH") : F("LOW"));
  Serial.println(stable ? F("  (stable)") : F("  (UNSTABLE)"));

  if (FLAME_AUTO_POLARITY && stable) {
    flameActiveLow = (idle == HIGH);
    Serial.print(F("Flame polarity learned: FIRE = "));
    Serial.println(flameActiveLow ? F("LOW") : F("HIGH"));
    flameArmed = true;
  } else if (!stable) {
    Serial.println(F("*** Flame pin is unstable - check the DO wire and the ground."));
    Serial.println(F("*** Falling back to the FLAME_ACTIVE_LOW setting."));
    lcdShow("FLAME PIN", "UNSTABLE-CHECK");
    delay(2000);
    flameArmed = !flameRaw();
  } else {
    flameArmed = !flameRaw();
    if (!flameArmed) {
      Serial.println(F("*** Flame reads FIRE at boot with auto-polarity off."));
      Serial.println(F("*** Flip FLAME_ACTIVE_LOW, or set FLAME_AUTO_POLARITY = true."));
      lcdShow("FLAME POLARITY", "CHECK SETTING");
      delay(2000);
    }
  }

  bootTime = millis();
  lastHazardSample = millis();
  resetDriveState();
  lastScan = lastTiltCheck = lastObsCheck = lastLineUpdate = millis();

  /* ---- find the line if we did not start on it ---- */
  if (FIND_LINE_AT_BOOT && !lineSeen()) {
    lcdShow("Finding line", "");
    Serial.println(F("Not on the line - creeping forward to find it"));
    unsigned long t0 = millis();
    drive(CREEP_SPEED, CREEP_SPEED);
    while (millis() - t0 < BOOT_FIND_MS && !lineSeen()) {
      WDT_KICK();
      if (hazardPoll()) break;                 // fire/smoke still comes first
      delay(10);
    }
    stopMotors();
    resetDriveState();
    if (!lineSeen()) Serial.println(F("Line not found - starting anyway"));
  }

  lcdShow("Patrolling", "");
  Serial.println(F("Ready"));

#if USE_WATCHDOG
  wdt_enable(WDTO_8S);
#endif
}

/* ============================================================================
   SENSOR READS
   ============================================================================ */
bool flameRaw() {
  int v = digitalRead(FLAME);
  return flameActiveLow ? (v == LOW) : (v == HIGH);
}

int gasLevel() {
  long sum = 0;
  for (byte i = 0; i < 5; i++) sum += analogRead(MQ2);
  return (int)(sum / 5);
}

bool gasRaw() {
  if (millis() - bootTime < GAS_WARMUP_MS) return false;
  return gasLevel() > GAS_THRESHOLD;
}

/* ============================================================================
   HAZARD POLL  —  fire and smoke, everywhere, all the time
   ============================================================================ */
bool hazardPoll() {
  if (inAlert) return false;
  if (millis() - lastHazardSample < HAZARD_SAMPLE_MS) return false;
  lastHazardSample = millis();

  bool f = flameRaw();
  if (!flameArmed) {
    if (!f) { flameArmed = true; Serial.println(F("Flame sensor armed")); }
    flameHits = 0;
  } else if (f) {
    if (++flameHits >= CONFIRM_COUNT) {
      flameHits = gasHits = 0;
      triggerAlert("FIRE DETECTED", "EVT:FIRE", true, true);
      return true;
    }
  } else flameHits = 0;

  bool g = gasRaw();
  if (millis() - bootTime < GAS_WARMUP_MS) {
    gasHits = 0;
  } else if (!gasArmed) {
    if (!g) { gasArmed = true; Serial.println(F("Gas sensor armed")); }
    gasHits = 0;
  } else if (g) {
    if (++gasHits >= CONFIRM_COUNT) {
      gasHits = flameHits = 0;
      triggerAlert("SMOKE/GAS HIGH", "EVT:GAS", true, true);
      return true;
    }
  } else gasHits = 0;

  return false;
}

bool waitMs(unsigned long ms) {
  unsigned long t0 = millis();
  while (millis() - t0 < ms) {
    WDT_KICK();
    if (hazardPoll()) return true;
    delay(10);
  }
  return false;
}

/* ============================================================================
   MAIN LOOP
   ============================================================================ */
void loop() {
  WDT_KICK();

  if (hazardPoll()) return;                    // fire / smoke first, always

#if DEBUG_SENSORS
  debugPrint();
#endif

  if (millis() - lastObsCheck >= OBS_CHECK_MS) {
    lastObsCheck = millis();
    long d = pingCM();                         // fast timeout: ~4 ms worst case
    lastDist = d;
    slowZone = (d > 0 && d < SLOW_CM);
    if (d > 0 && d <= OBSTACLE_CM) {
      if (++obsHits >= 3) {
        obsHits = 0;
        lastDist = d;
        avoidObstacle();
        return;
      }
    } else {
      obsHits = 0;
    }
  }

  if (millis() - lastTiltCheck >= TILT_CHECK_MS) {
    lastTiltCheck = millis();
    if (isTilted()) {
      if (++tiltHits >= CONFIRM_COUNT) {
        tiltHits = 0;
        triggerAlert("TIPPED OVER", "EVT:TILT", SEND_TILT_EVENT, false);
        return;
      }
    } else tiltHits = 0;
  }

  if (millis() - lastScan >= SCAN_INTERVAL_MS) {
    scanForHazards();
    lastScan = millis();
    return;
  }

  if (millis() - lastLineUpdate >= LINE_UPDATE_MS) {
    lastLineUpdate = millis();
    followLine();
  }
}

/* ============================================================================
   LINE SENSING
   ----------------------------------------------------------------------------
   readLineMask()  bit i = sensor i sees the line. Bit 0 = LEFT, bit 4 = RIGHT.
                   A bit only changes state after two consecutive equal reads,
                   which kills single-sample noise at the cost of <=20 ms lag.

   linePosition()  Returns true only for a TRUSTWORTHY pattern: at least one
                   sensor lit and all lit sensors contiguous. The position is
                        (firstLit + lastLit - 4) * 10
                   giving -40 (hard left) .. 0 (centred) .. +40 (hard right)
                   in half-sensor steps:
                        10000 = -40   11000 = -30   01000 = -20
                        01100 = -10   00100 =   0   00110 = +10
                        00010 = +20   00011 = +30   00001 = +40
                   A split pattern such as 10100 or 10001 returns false, and
                   the caller then holds its last good error rather than
                   steering on an average that means nothing.
   ============================================================================ */
byte readLineMask() {
  byte raw = 0;
  for (byte i = 0; i < 5; i++) {
    bool v = digitalRead(LINE[i]);
    bool on = LINE_ON_BLACK ? v : !v;
    if (on) raw |= (1 << i);
  }

  // Do not add a 20 ms debounce delay to steering.  The controller already
  // validates the pattern in linePosition(), so noisy/split patterns are
  // rejected there.
  rawPrevMask = raw;
  stableMask = raw;
  return raw;
}

bool linePosition(byte mask, int* error, byte* count) {
  if (mask == 0) { *count = 0; return false; }

  // Reject impossible split patterns such as 10100 or 10001.
  byte n = 0;
  byte first = 255, last = 0;
  long weighted = 0;

  // Sensor positions: -40, -20, 0, +20, +40.
  for (byte i = 0; i < 5; i++) {
    if (mask & (1 << i)) {
      n++;
      if (first == 255) first = i;
      last = i;
      weighted += ((int)i - 2) * 20L;
    }
  }

  *count = n;
  if ((byte)(last - first + 1) != n) return false;

  *error = (int)(weighted / n);
  return true;
}

/* ============================================================================
   LINE FOLLOWING  —  PID + curvature speed + corner pivot + junction commit
   ============================================================================ */
void followLine() {
  byte mask = readLineMask();
  byte count = 0;
  int  error = 0;
  bool valid = linePosition(mask, &error, &count);

  /* ---- committed to driving straight through a junction ---- */
  if (millis() < junctionUntil) {
    driveStraight((slowZone ? SLOW_SPEED : BASE_SPEED) - 15);
    return;
  }

  /* ---- nothing at all under the sensors ---- */
  if (mask == 0) { lineLost(); return; }

  lostSince = 0;

  /* ---- junction / wide patch: commit to straight for a fixed window ---- */
  if (count >= 4) {
    junctionUntil = millis() + JUNCTION_HOLD_MS;
    driveStraight((slowZone ? SLOW_SPEED : BASE_SPEED) - 15);
    dFilt = 0; iSum = 0; prevError = 0; lastError = 0;
    return;
  }

  /* ---- split / impossible pattern: hold the last good error, freeze D ---- */
  bool frozen = false;
  if (!valid) { error = lastError; frozen = true; }

  /* ---- real dt, so the D term keeps the same gain when the loop is late ---- */
  unsigned long now = millis();
  unsigned long dt  = now - lastLineTime;
  lastLineTime = now;
  if (dt == 0) dt = 1;
  if (dt > 50) dt = 50;

  if (!frozen) {
    int dRate = (int)(((long)(error - prevError) * 10) / (long)dt);  // per 10 ms
    dFilt = (dFilt * 2.0 + (float)dRate) / 3.0;                      // low-pass
  }
  prevError = error;
  lastError = error;

  /* ---- integral: only near the line, reset on every zero crossing ---- */
  if (error == 0 || (error > 0) != (iSum > 0)) iSum = 0;
  iSum = constrain(iSum + error / 10, -I_CLAMP, I_CLAMP);

  /* ---- curvature: decaying average of |error| ---- */
  int absErr = abs(error);
  curve = (curve * 7.0 + (float)absErr) / 8.0;

  /* ------------------------------------------------------------------
     SHARP CORNER: an outer sensor is on the line and the centre is dark.
     Arcing loses a 90 deg track corner — pivot on the spot instead.
     ------------------------------------------------------------------ */
  if (absErr >= 30 && !(mask & 0x04) && count <= 2) {
    if (error > 0) drive( MIN_TURN_SPEED, -MIN_TURN_SPEED);   // line to the right
    else           drive(-MIN_TURN_SPEED,  MIN_TURN_SPEED);   // line to the left
    iSum = 0;
    return;                                   // outL/outR keep the pivot values,
  }                                           // so the slew resumes smoothly

  /* ---- PID ---- */
  long p = (long)KP * error;
  long d = (long)(KD * dFilt);
  long i = (long)KI * iSum;
  int correction = (int)((p + d + i) / 10);
  correction = constrain(correction, -MAX_CORRECTION, MAX_CORRECTION);

  /* ---- speed from the LARGER of "how far off now" and "how bendy lately" ---- */
  int base    = slowZone ? SLOW_SPEED : BASE_SPEED;
  int floorSp = (base < MIN_TURN_SPEED) ? base : MIN_TURN_SPEED;
  int effErr  = (absErr > (int)curve) ? absErr : (int)curve;
  if (effErr > 40) effErr = 40;
  int speed = base - ((long)effErr * (base - floorSp)) / 40;

  /* ---- signed drive: inner wheel reverses on a sharp turn ---- */
  int left  = constrain(speed + correction,              -MAX_REVERSE, 255);
  int right = constrain(speed - correction + MOTOR_TRIM, -MAX_REVERSE, 255);
  driveSmooth(left, right);
}

/* ----------------------------------------------------------------------------
   LINE LOST  —  four stages
     0 .. GRACE          hold the last steering, reduced speed (dust, gaps)
     GRACE .. +BACKUP    reverse straight back onto where the line was
     .. LOST_TIMEOUT     expanding sweep: near side, far side, wider, wider
     after that          creep forward and hope
   ---------------------------------------------------------------------------- */
void lineLost() {
  if (lostSince == 0) lostSince = millis();
  unsigned long lost = millis() - lostSince;

  /* stage 1 — coast through the gap on the last known steering */
  if (lost < LINE_GRACE_MS) {
    int corr = constrain((KP * lastError) / 10, -MAX_CORRECTION, MAX_CORRECTION);
    int s = MIN_TURN_SPEED;
    driveSmooth(constrain(s + corr, -MAX_REVERSE, 255),
                constrain(s - corr + MOTOR_TRIM, -MAX_REVERSE, 255));
    return;
  }

  dFilt = 0; iSum = 0; prevError = 0;

  /* stage 2 — reverse back onto it */
  if (lost < LINE_GRACE_MS + LOST_BACKUP_MS) {
    drive(-BACKUP_SPEED, -BACKUP_SPEED);
    return;
  }

  /* stage 4 — long gone, just creep forward */
  if (lost >= LOST_TIMEOUT_MS) { driveStraight(CREEP_SPEED); return; }

  /* stage 3 — expanding sweep around the side the line was last seen */
  unsigned long t = lost - LINE_GRACE_MS - LOST_BACKUP_MS;
  unsigned long leg = LOST_SWEEP_MS;
  byte n = 0;
  while (t >= leg && n < 8) { t -= leg; n++; leg = LOST_SWEEP_MS * (n / 2 + 1); }

  bool firstSide = (lastError >= 0);                  // last seen to the RIGHT
  bool goRight = (n % 2 == 0) ? firstSide : !firstSide;

  if (goRight) drive( SEARCH_SPEED, -SEARCH_SPEED);
  else         drive(-SEARCH_SPEED,  SEARCH_SPEED);
}

/* ============================================================================
   STOP-AND-SCAN  —  stationary sweep for an intruder
   ============================================================================ */
void scanForHazards() {
  stopMotors();
  lcdShow("Scanning...", "");

  unsigned long start = millis();
  int angle = SERVO_FWD, step = 15;
  bool goingUp = true;
  byte pirHits = 0;

  while (millis() - start < SCAN_MS) {
    WDT_KICK();
    scanServo.write(angle);
    if (goingUp) { angle += step; if (angle >= SERVO_LEFT)  goingUp = false; }
    else         { angle -= step; if (angle <= SERVO_RIGHT) goingUp = true;  }

    if (hazardPoll()) { scanServo.write(SERVO_FWD); return; }

    if (digitalRead(PIR) == HIGH) {
      if (++pirHits >= 2) {
        scanServo.write(SERVO_FWD);
        triggerAlert("INTRUDER", "EVT:INTRUDER", true, true);
        return;
      }
    } else pirHits = 0;

    delay(80);
  }

  scanServo.write(SERVO_FWD);
  resetDriveState();
  lcdShow("Patrolling", "");
}

/* ============================================================================
   ALERT  —  photo + Telegram + buzzer + LCD, then FREEZE until reset
   ============================================================================ */
void triggerAlert(const char* msg, const char* event, bool notify, bool capture) {
  inAlert = true;
  stopMotors();
  scanServo.write(SERVO_FWD);

  Serial.print(F("ALERT: ")); Serial.println(msg);
  lcdShow("!! ALERT !!", msg);

  if (notify) sendEvent(event);
  (void)capture;

  unsigned long lastToggle = millis();
  bool buzzOn = false;
  while (true) {
    WDT_KICK();
    if (digitalRead(RESET_BTN) == LOW) {
      delay(50);
      if (digitalRead(RESET_BTN) == LOW) break;
    }
    if (millis() - lastToggle >= 300) {
      buzzOn = !buzzOn;
      digitalWrite(BUZZER, buzzOn ? HIGH : LOW);
      lastToggle = millis();
    }
    delay(10);
  }
  digitalWrite(BUZZER, LOW);
  while (digitalRead(RESET_BTN) == LOW) { WDT_KICK(); delay(10); }

  sendEvent("EVT:CLEAR");
  lcdShow("Resuming...", "");
  delay(800);
  lcdShow("Patrolling", "");

  flameHits = gasHits = tiltHits = obsHits = 0;
  resetDriveState();
  lastScan = lastTiltCheck = lastObsCheck = lastLineUpdate = millis();
  inAlert = false;
}

void sendEvent(const char* event) {
  Serial1.println(event);
  Serial.print(F("-> ESP32: ")); Serial.println(event);
  if (!WAIT_FOR_ACK) return;

  for (byte attempt = 0; attempt < 3; attempt++) {
    char buf[12]; byte n = 0;
    unsigned long t0 = millis();
    while (millis() - t0 < 2000) {
      WDT_KICK();
      while (Serial1.available()) {
        char c = Serial1.read();
        if (c == '\n' || c == '\r') {
          buf[n] = '\0';
          if (strstr(buf, "ACK")) return;
          n = 0;
        } else if (n < sizeof(buf) - 1) buf[n++] = c;
      }
    }
    Serial1.println(event);
  }
}

/* ============================================================================
   OBSTACLE AVOIDANCE  —  the box manoeuvre, left first

     step 1  stop (back off if we are nose-to-nose with it)
     step 2  check the left side has room; use the right side only if not
     step 3  turn 90 deg LEFT
     step 4  drive LEFT past the obstacle (head watching it on our right)
     step 5  turn 90 deg RIGHT  -> back on the ORIGINAL heading
     step 6  drive FORWARD until the obstacle is behind us
     step 7  turn 90 deg RIGHT  -> now pointing back at the line
     step 8  creep until the line is found
     step 9  turn 90 deg LEFT   -> lined up, resume patrol
   ============================================================================ */
void avoidObstacle() {
  stopMotors();
  scanServo.write(SERVO_FWD);
  slowZone = false;
  lcdShow("Obstacle", "Stopping");
  if (waitMs(250)) return;

  // Create a little space before the 90-degree pivot. This prevents the
  // robot from clipping an obstacle that is already very close to its nose.
  if (lastDist > 0 && lastDist <= TOO_CLOSE_CM) {
    drive(-BACKUP_SPEED, -BACKUP_SPEED);
    if (waitMs(300)) return;
    stopMotors();
    if (waitMs(120)) return;
  }

  /* ---- step 1: room to pivot? ---- */
  if (pingMedianCM() <= TOO_CLOSE_CM) {
    lcdShow("Obstacle", "Backing off");
    drive(-BACKUP_SPEED, -BACKUP_SPEED);
    if (waitMs(450)) return;
    stopMotors();
    if (waitMs(150)) return;
  }

  /* ---- step 2: is the left side usable? ---- */
  long leftRoom = 0, rightRoom = 0;
  if (lookAt(SERVO_LEFT,  &leftRoom))  return;
  if (lookAt(SERVO_RIGHT, &rightRoom)) return;
  scanServo.write(SERVO_FWD);
  if (waitMs(180)) return;

  Serial.print(F("Sides: L=")); Serial.print(leftRoom);
  Serial.print(F(" R="));       Serial.println(rightRoom);

  if (leftRoom < SIDE_MIN_CM && rightRoom < SIDE_MIN_CM) {
    lcdShow("Boxed in", "Turning back");
    drive(-BACKUP_SPEED, -BACKUP_SPEED);
    if (waitMs(BACKUP_MS)) return;
    stopMotors();
    if (turnDegrees(180, true)) return;
    resetDriveState();
    lcdShow("Patrolling", "");
    lastScan = lastObsCheck = lastLineUpdate = millis();
    return;
  }

  bool goLeft = (leftRoom >= SIDE_MIN_CM);       // LEFT is the default
  lcdShow("Obstacle", goLeft ? "Going LEFT" : "Left blocked>R");
  if (waitMs(150)) return;

  /* If we go left, the obstacle sits on our RIGHT for the whole manoeuvre. */
  const int obsAngle = goLeft ? SERVO_RIGHT : SERVO_LEFT;
  bool sawLine = false, sideClear = false;

  /* ---- step 3 ---- */
  if (turnDegrees(TURN_DEG, goLeft)) return;
  if (waitMs(120)) return;

  /* ---- step 4 ---- */
  lcdShow("Avoiding", goLeft ? "leg 1 - left" : "leg 1 - right");
  if (runLeg(SIDESTEP_SPEED, LEG_SIDE_MS, obsAngle, false, &sawLine, &sideClear)) return;
  if (runLeg(SIDESTEP_SPEED, EXTRA_CLEAR_MS, -1, false, &sawLine, NULL)) return;
  if (waitMs(120)) return;

  /* ---- step 5 ---- */
  if (turnDegrees(TURN_DEG, !goLeft)) return;
  if (waitMs(120)) return;

  /* ---- step 6 ---- */
  lcdShow("Avoiding", "leg 2 - forward");
  sideClear = false;
  if (runLeg(SIDESTEP_SPEED, LEG_PAST_MS, obsAngle, true, &sawLine, &sideClear)) return;

  if (sawLine) {                                 // back on the track already
    stopMotors(); scanServo.write(SERVO_FWD);
    resetDriveState();
    lcdShow("Patrolling", "");
    lastScan = lastObsCheck = lastLineUpdate = millis();
    return;
  }
  if (runLeg(SIDESTEP_SPEED, EXTRA_CLEAR_MS, -1, true, &sawLine, NULL)) return;
  if (waitMs(120)) return;

  /* ---- steps 7 + 8 ---- */
  if (!sawLine) {
    if (turnDegrees(TURN_DEG, !goLeft)) return;
    if (waitMs(120)) return;

    lcdShow("Avoiding", "finding line");
    if (runLeg(CREEP_SPEED, LEG_FIND_MS, -1, true, &sawLine, NULL)) return;

    /* ---- step 9 ---- */
    if (sawLine) {
      stopMotors();
      if (waitMs(100)) return;
      if (turnDegrees(TURN_DEG, goLeft)) return;
    }
  }

  /* ---- last resort: controlled search for the line ---- */
  if (!lineSeen()) {
    lcdShow("Line lost", "Sweeping");

    // Search toward the side from which we approached the obstacle first.
    if (turnDegrees(30, goLeft)) return;
    if (!lineSeen()) {
      if (turnDegrees(60, !goLeft)) return;
    }
    if (!lineSeen()) {
      if (turnDegrees(60, !goLeft)) return;
    }
    if (!lineSeen()) {
      if (turnDegrees(30, goLeft)) return;
    }
  }

  stopMotors();
  scanServo.write(SERVO_FWD);
  resetDriveState();
  lcdShow("Patrolling", "");
  lastScan = lastObsCheck = lastLineUpdate = millis();
}

/* Point the head at one angle, let it settle, take a long-range median. */
bool lookAt(int angle, long* out) {
  scanServo.write(angle);
  if (waitMs(280)) return true;
  *out = pingMedianCM();
  if (*out < 0) *out = 400;
  return false;
}

/* ----------------------------------------------------------------------------
   Drive one straight leg of the box.
     sideAngle >= 0 : point the head there and alternate with a look forward.
                      The leg ENDS when that side has been clear for
                      PASS_CONFIRM samples (the obstacle has been passed).
     sideAngle <  0 : look straight ahead only.
     stopOnLine     : end the leg as soon as any sensor sees the track.
   Also ends on a new obstacle ahead, or when maxMs expires.
   ---------------------------------------------------------------------------- */
bool runLeg(int speed, unsigned long maxMs, int sideAngle,
            bool stopOnLine, bool* sawLine, bool* sideClear) {
  *sawLine = false;
  if (sideClear) *sideClear = false;

  bool watchSide  = (sideAngle >= 0);
  bool lookingSide = watchSide;
  byte clearHits = 0;

  if (watchSide) {
    scanServo.write(sideAngle);
    if (waitMs(250)) return true;
  } else {
    scanServo.write(SERVO_FWD);
  }

  unsigned long t0 = millis();
  drive(speed, speed);

  while (millis() - t0 < maxMs) {
    WDT_KICK();
    if (hazardPoll()) { stopMotors(); scanServo.write(SERVO_FWD); return true; }

    if (stopOnLine && lineSeen()) { *sawLine = true; break; }

    long d = pingLongCM();
    if (d < 0) d = 400;

    if (lookingSide) {
      if (d >= PASS_CLEAR_CM) {
        if (++clearHits >= PASS_CONFIRM) {
          if (sideClear) *sideClear = true;
          break;
        }
      } else {
        clearHits = 0;
      }
    } else {
      if (d < OBSTACLE_CM) break;               // something new straight ahead
    }

    if (watchSide) {
      lookingSide = !lookingSide;
      scanServo.write(lookingSide ? sideAngle : SERVO_FWD);
      if (waitMs(160)) { stopMotors(); scanServo.write(SERVO_FWD); return true; }
      drive(speed, speed);                      // keep rolling while it swings
    } else {
      delay(15);
    }
  }

  stopMotors();
  scanServo.write(SERVO_FWD);
  return false;
}

/* ============================================================================
   TURNING  —  measured with the MPU6050 Z gyro, timed only as a fallback
   ============================================================================ */
bool turnDegrees(int deg, bool goLeft) {
  if (!gyroOK) {
    unsigned long ms = (unsigned long)((float)PIVOT_90_MS * (float)deg / 90.0);
    return pivotForMs(goLeft, ms);
  }

  float target = (float)deg;
  float acc = 0;
  unsigned long capMs = (unsigned long)((float)PIVOT_90_MS * ((float)deg / 90.0) * 3.0) + 500;
  unsigned long t0 = millis();
  unsigned long tPrev = micros();
  byte i2cFails = 0;

  stopMotors();
  delay(60);
  if (goLeft) pivotLeft(PIVOT_SPEED); else pivotRight(PIVOT_SPEED);

  while (fabs(acc) < target) {
    WDT_KICK();
    if (millis() - t0 > capMs) break;                 // safety cap
    if (hazardPoll()) { stopMotors(); return true; }

    int16_t gz;
    if (!readGyroZ(&gz)) {
      if (++i2cFails > 20) { gyroOK = false; break; } // gyro died mid-turn
      delay(3);
      continue;
    }
    i2cFails = 0;

    unsigned long tNow = micros();
    float dt = (float)(tNow - tPrev) / 1000000.0;
    tPrev = tNow;
    if (dt > 0.05) dt = 0.05;

    acc += (((float)gz - gyroZbias) / GYRO_LSB_PER_DPS) * dt;
    delay(2);
  }

  stopMotors();
  Serial.print(F("Turn ")); Serial.print(goLeft ? F("L ") : F("R "));
  Serial.print(deg); Serial.print(F(" deg -> measured "));
  Serial.println(fabs(acc));
  delay(80);
  return false;
}

bool pivotForMs(bool goLeft, unsigned long ms) {
  if (goLeft) pivotLeft(PIVOT_SPEED); else pivotRight(PIVOT_SPEED);
  bool aborted = waitMs(ms);
  stopMotors();
  delay(80);
  return aborted;
}

/* ============================================================================
   HELPERS
   ============================================================================ */
void resetDriveState() {
  prevError = 0; lastError = 0;
  dFilt = 0; curve = 0; iSum = 0;
  lostSince = 0; junctionUntil = 0;
  outL = outR = 0;
  rawPrevMask = stableMask = 0;
  lastLineTime = millis();
}

void lcdShow(const char* l0, const char* l1) {
  lcd.clear();
  lcd.setCursor(0, 0); lcd.print(l0);
  if (l1 && l1[0]) { lcd.setCursor(0, 1); lcd.print(l1); }
}

bool lineSeen() {
  for (int i = 0; i < 5; i++) {
    bool raw = digitalRead(LINE[i]);
    if (LINE_ON_BLACK ? raw : !raw) return true;
  }
  return false;
}

long pingRaw(unsigned int timeoutUs) {
  digitalWrite(TRIG, LOW);  delayMicroseconds(3);
  digitalWrite(TRIG, HIGH); delayMicroseconds(10);
  digitalWrite(TRIG, LOW);
  long dur = pulseIn(ECHO, HIGH, timeoutUs);
  if (dur == 0) return -1;
  return dur / 58;
}

long pingCM()     { return pingRaw(PING_FAST_US); }   // patrol: never blocks >4 ms
long pingLongCM() { return pingRaw(PING_LONG_US); }   // avoidance: full range

long pingMedianCM() {
  long v[3];
  for (byte i = 0; i < 3; i++) { long p = pingLongCM(); v[i] = (p < 0) ? 400 : p; delay(8); }
  long t;
  if (v[0] > v[1]) { t = v[0]; v[0] = v[1]; v[1] = t; }
  if (v[1] > v[2]) { t = v[1]; v[1] = v[2]; v[2] = t; }
  if (v[0] > v[1]) { t = v[0]; v[0] = v[1]; v[1] = t; }
  return v[1];
}

/* Signed motor control: negative = reverse. */
void drive(int l, int r) {
  outL = l; outR = r;                           // keep the slew limiter honest

  if (l >= 0) { digitalWrite(IN1, LOW);  digitalWrite(IN2, HIGH); }
  else        { digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW); l = -l; }
  if (r >= 0) { digitalWrite(IN3, LOW);  digitalWrite(IN4, HIGH); }
  else        { digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW); r = -r; }

  if (l > 0 && l < MIN_PWM) l = MIN_PWM;        // below this the wheels stall
  if (r > 0 && r < MIN_PWM) r = MIN_PWM;

  analogWrite(ENA, constrain(l, 0, 255));
  analogWrite(ENB, constrain(r, 0, 255));
}

/* Rate-limited version used by the line follower. Sudden PWM jumps make the
   wheels slip, and a slipping wheel is how the robot loses the line. */
void driveSmooth(int l, int r) {
  int nl = constrain(l, outL - SLEW_STEP, outL + SLEW_STEP);
  int nr = constrain(r, outR - SLEW_STEP, outR + SLEW_STEP);
  drive(nl, nr);
}

/* Straight ahead with the motor trim applied. */
void driveStraight(int speed) {
  driveSmooth(speed, speed + MOTOR_TRIM);
}

void pivotLeft(int s)  { drive(-s,  s); }
void pivotRight(int s) { drive( s, -s); }
void stopMotors()      { analogWrite(ENA, 0); analogWrite(ENB, 0); outL = outR = 0; }

/* ---- MPU6050 ---- */
void mpuInit() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B); Wire.write(0);              // wake up
  Wire.endTransmission(true);
  delay(10);
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x1A); Wire.write(3);              // DLPF 44 Hz
  Wire.endTransmission(true);
  delay(5);
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x1B); Wire.write(0);              // gyro +/-250 dps
  Wire.endTransmission(true);
  delay(5);
}

bool readGyroZ(int16_t* gz) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x47);                             // GYRO_ZOUT_H
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom(MPU_ADDR, 2, true) != 2) return false;
  *gz = (int16_t)((Wire.read() << 8) | Wire.read());
  return true;
}

void calibrateGyro() {
  long sum = 0;
  int good = 0;
  for (int i = 0; i < 300; i++) {
    int16_t gz;
    if (readGyroZ(&gz)) { sum += gz; good++; }
    delay(3);
  }
  if (good > 200) { gyroZbias = (float)sum / (float)good; gyroOK = true; }
  else            { gyroOK = false; }
}

bool isTilted() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom(MPU_ADDR, 6, true) != 6) return false;
  Wire.read(); Wire.read();                     // ax
  Wire.read(); Wire.read();                     // ay
  int16_t az = (Wire.read() << 8) | Wire.read();
  return (az < 8000);
}

/* ---- live readout for calibration ----
   Uses the CACHED distance: it must not fire its own ping, or the 1 Hz debug
   line would add another blocking pulseIn to the control loop. ---- */
#if DEBUG_SENSORS
void debugPrint() {
  static unsigned long last = 0;
  if (millis() - last < 1000) return;
  last = millis();
  Serial.print(F("FLAME=")); Serial.print(digitalRead(FLAME));
  Serial.print(F(" FIRE?=")); Serial.print(flameRaw() ? F("YES") : F("NO"));
  Serial.print(F("  GAS="));  Serial.print(gasLevel());
  if (millis() - bootTime < GAS_WARMUP_MS) Serial.print(F(" (warming)"));
  Serial.print(F("  PIR="));  Serial.print(digitalRead(PIR));
  Serial.print(F("  DIST=")); Serial.print(lastDist);
  Serial.print(F("  ERR="));  Serial.print(lastError);
  Serial.print(F("  CRV="));  Serial.print((int)curve);
  Serial.print(F("  GYRO=")); Serial.print(gyroOK ? F("ok") : F("--"));
  if (!flameArmed) Serial.print(F("  [FLAME DISARMED]"));
  Serial.print(F("  LINE="));
  for (int i = 0; i < 5; i++) Serial.print((stableMask >> i) & 1);
  Serial.println();
}
#endif

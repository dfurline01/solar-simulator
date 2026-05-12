
// Solar Simulator — UPLOAD TO SIMULATOR ARDUINO MEGA
// Pan:  DM542  — PUL+ pin 2, DIR+ pin 3, PUL-/DIR- to GND
// Tilt: TB6600 — PUL+ pin 4, DIR+ pin 5, PUL-/DIR- to GND
//
// Tower link (Serial1, 115200 baud):
//   Simulator TX1 (pin 18) → Tower RX2 (pin 17)   [START + H msgs]
//   Simulator RX1 (pin 19) ← Tower TX2 (pin 16)   [READY / HOMED]
//   GND ↔ GND
//
// Serial Protocol (USB, 115200 baud):
//   T<pan>,<tilt>,<ms>,<az>,<el>\n   → TOK\n
//   MP<steps>\n                      → OK\n
//   MT<steps>\n                      → OK\n
//   S\n                              → S:<pp>,<tp>,<po>,<to>,0\n
//   W\n                              → WOK\n | WERR\n

#include <AccelStepper.h>

// Pins
#define PAN_STEP_PIN   2
#define PAN_DIR_PIN    3
#define TILT_STEP_PIN  4
#define TILT_DIR_PIN   5

// Speed / acceleration
const float PAN_TRACK_SPEED       = 2000.0;
const float PAN_TRACK_ACCEL       = 1000.0;
const float PAN_SLEW_SPEED        = 5000.0;
const float PAN_SLEW_ACCEL        = 2000.0;

const float TILT_TRACK_SPEED      = 2000.0;
const float TILT_TRACK_ACCEL_UP   = 2000.0;
const float TILT_TRACK_ACCEL_DOWN = 2000.0;
const float TILT_SLEW_SPEED       = 2000.0;
const float TILT_SLEW_ACCEL_UP    = 2000.0;
const float TILT_SLEW_ACCEL_DOWN  = 2000.0;

// Stepper objects
AccelStepper panMotor (AccelStepper::DRIVER, PAN_STEP_PIN,  PAN_DIR_PIN);
AccelStepper tiltMotor(AccelStepper::DRIVER, TILT_STEP_PIN, TILT_DIR_PIN);

// State
String inputBuffer  = "";
bool   trackingMode = false;

// Forward declarations
void processCommand(String cmd);
void handleTarget(String cmd);
void handleBlockingMove(AccelStepper &motor, String stepsStr,
                        float speed, float accelUp, float accelDown);
void handleStatus();
void handleWakeTower();

// Setup
void setup() {
    Serial.begin(115200);   // USB — Python communication
    Serial1.begin(115200);  // Tower link (TX1=pin18, RX1=pin19)

    panMotor.setMaxSpeed(PAN_TRACK_SPEED);
    panMotor.setAcceleration(PAN_TRACK_ACCEL);
    panMotor.setMinPulseWidth(10);

    tiltMotor.setMaxSpeed(TILT_TRACK_SPEED);
    tiltMotor.setAcceleration(TILT_TRACK_ACCEL_UP);
    tiltMotor.setMinPulseWidth(10);
}

// Main loop
void loop() {
    if (trackingMode) {
        if (panMotor.distanceToGo()  != 0) panMotor.runSpeed();
        if (tiltMotor.distanceToGo() != 0) tiltMotor.runSpeed();
        if (panMotor.distanceToGo()  == 0 && tiltMotor.distanceToGo() == 0)
            trackingMode = false;
    } else {
        panMotor.run();
        tiltMotor.run();
    }

    while (Serial.available()) {
        char c = Serial.read();
        if (c == '\n') {
            processCommand(inputBuffer);
            inputBuffer = "";
        } else {
            inputBuffer += c;
        }
    }
}

// Command dispatcher
void processCommand(String cmd) {
    cmd.trim();
    if (cmd.length() == 0) return;

    if (cmd.startsWith("T")) {
        handleTarget(cmd);
    } else if (cmd.startsWith("MP")) {
        handleBlockingMove(panMotor, cmd.substring(2),
                           PAN_SLEW_SPEED, PAN_SLEW_ACCEL, PAN_SLEW_ACCEL);
    } else if (cmd.startsWith("MT")) {
        handleBlockingMove(tiltMotor, cmd.substring(2),
                           TILT_SLEW_SPEED, TILT_SLEW_ACCEL_UP, TILT_SLEW_ACCEL_DOWN);
    } else if (cmd.startsWith("S")) {
        handleStatus();
    } else if (cmd.startsWith("W")) {
        handleWakeTower();
    }
}

// W — Wake tower with robust handshake
//
// 1. Flush Serial1 (discard boot garbage)
// 2. Wait up to 30s for tower to send "READY"
// 3. Send "START" to tower
// 4. Wait up to 120s for tower to send "HOMED"
// 5. Reply WOK or WERR to Python
void handleWakeTower() {
    // Flush any stale data
    while (Serial1.available()) Serial1.read();

    //Phase 1: Wait for READY from tower
    String buf      = "";
    bool   gotReady = false;
    unsigned long t0 = millis();

    while (millis() - t0 < 30000UL) {          // 30s for tower to boot
        while (Serial1.available()) {
            char c = Serial1.read();
            if (c == '\n') {
                buf.trim();
                if (buf == "READY") { gotReady = true; }
                buf = "";
            } else {
                buf += c;
            }
            if (gotReady) break;
        }
        if (gotReady) break;
    }

    if (!gotReady) {
        Serial.println("WERR");                 // tower never showed up
        Serial.flush();
        return;
    }

    // Phase 2: Send START, wait for HOMED
    // Tower sends "HOMING" heartbeats every ~2s during homing.
    // We reset our timer on each heartbeat — only time out if
    // the tower goes silent for 30s (meaning something is stuck).
    delay(50);
    while (Serial1.available()) Serial1.read();

    Serial1.println("START");

    buf = "";
    bool gotHomed = false;
    unsigned long lastHeard = millis();

    while (millis() - lastHeard < 30000UL) {    // 30s silence = timeout
        while (Serial1.available()) {
            char c = Serial1.read();
            if (c == '\n') {
                buf.trim();
                if (buf == "HOMED")  { gotHomed = true; }
                if (buf == "HOMING") { lastHeard = millis(); }  // reset timer
                buf = "";
            } else {
                buf += c;
            }
            if (gotHomed) break;
        }
        if (gotHomed) break;
    }

    Serial.println(gotHomed ? "WOK" : "WERR");
    Serial.flush();
}

// T<pan>,<tilt>,<interval_ms>,<azimuth>,<elevation>
// Constant velocity tracking — also forwards az/el to tower
void handleTarget(String cmd) {
    int c1 = cmd.indexOf(',');
    int c2 = cmd.indexOf(',', c1 + 1);
    int c3 = cmd.indexOf(',', c2 + 1);
    int c4 = cmd.indexOf(',', c3 + 1);
    if (c1 < 0 || c2 < 0 || c3 < 0 || c4 < 0) return;

    long  panTarget  = cmd.substring(1,      c1).toInt();
    long  tiltTarget = cmd.substring(c1 + 1, c2).toInt();
    float intervalMs = cmd.substring(c2 + 1, c3).toFloat();
    float azimuth    = cmd.substring(c3 + 1, c4).toFloat();
    float elevation  = cmd.substring(c4 + 1).toFloat();

    float intervalSec = intervalMs / 1000.0;

    float panSteps  = (float)(panTarget  - panMotor.currentPosition());
    float tiltSteps = (float)(tiltTarget - tiltMotor.currentPosition());

    float panSpeed  = (intervalSec > 0) ? panSteps  / intervalSec : 0;
    float tiltMax   = (tiltSteps < 0)   ? TILT_TRACK_SPEED * 0.6 : TILT_TRACK_SPEED;
    float tiltSpeed = (intervalSec > 0) ? tiltSteps / intervalSec : 0;

    panSpeed  = constrain(panSpeed,  -PAN_TRACK_SPEED, PAN_TRACK_SPEED);
    tiltSpeed = constrain(tiltSpeed, -tiltMax,         tiltMax);

    panMotor.moveTo(panTarget);
    tiltMotor.moveTo(tiltTarget);
    panMotor.setSpeed(panSpeed);
    tiltMotor.setSpeed(tiltSpeed);
    trackingMode = true;

    // Forward sun position to tower
    Serial1.print("H");
    Serial1.print(azimuth,   4);
    Serial1.print(",");
    Serial1.println(elevation, 4);

    Serial.println("TOK");
    Serial.flush();
}

// Blocking move — test and initial slew
void handleBlockingMove(AccelStepper &motor, String stepsStr,
                        float speed, float accelUp, float accelDown) {
    trackingMode   = false;
    long steps     = stepsStr.toInt();
    bool goingDown = (&motor == &tiltMotor) && (steps < 0);

    motor.setMaxSpeed(speed);
    motor.setAcceleration(goingDown ? accelDown : accelUp);
    motor.move(steps);

    while (motor.distanceToGo() != 0) {
        motor.run();
    }

    Serial.println("OK");
    Serial.flush();
}

// S — status report
void handleStatus() {
    bool panOnTarget  = (panMotor.distanceToGo()  == 0);
    bool tiltOnTarget = (tiltMotor.distanceToGo() == 0);

    Serial.print("S:");
    Serial.print(panMotor.currentPosition());  Serial.print(",");
    Serial.print(tiltMotor.currentPosition()); Serial.print(",");
    Serial.print(panOnTarget  ? "1" : "0");    Serial.print(",");
    Serial.print(tiltOnTarget ? "1" : "0");    Serial.print(",");
    Serial.println("0");
    Serial.flush();
}
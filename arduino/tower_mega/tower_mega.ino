
// Tower Mirror Controller — UPLOAD TO TOWER ARDUINO MEGA

// Now uses AccelStepper for smooth constant-velocity tracking
// and two-speed homing (fast search, slow edge detection).
//
// Wiring:
//   Simulator TX1 (pin 18) → Tower RX2 (pin 17)
//   Tower TX2 (pin 16)     → Simulator RX1 (pin 19)
//   GND ↔ GND


#include <math.h>
#include <AccelStepper.h>


// MOTOR PINS

#define AZ_PULSE_PIN  25
#define AZ_DIR_PIN    27
#define ZEN_PULSE_PIN 24
#define ZEN_DIR_PIN   26


// HOMING SENSOR PINS

#define PROX_PIN       45
#define LIMIT_MAX_PIN  39
#define LIMIT_MIN_PIN  38


// MOTOR CONFIGURATION

const int   STEPS_PER_REV    = 200;
const int   MICROSTEPS       = 32;
const float MICROSTEPS_REV   = (float)STEPS_PER_REV * MICROSTEPS;  // 6400

const float GEAR_RATIO       = 72.0 / 42.0;    // azimuth
const float SPROCKET_RATIO   = 40.0 / 10.0;    // zenith

// Steps per degree for each axis
const float AZ_STEPS_PER_DEG  = MICROSTEPS_REV * GEAR_RATIO / 360.0;
const float ZEN_STEPS_PER_DEG = MICROSTEPS_REV * SPROCKET_RATIO / 360.0;


// SPEED SETTINGS (steps/sec)

// Tracking — tune these for smooth motion without skipping
const float AZ_TRACK_SPEED   = 200.0;
const float AZ_TRACK_ACCEL   = 200.0;
const float ZEN_TRACK_SPEED  = 200.0;
const float ZEN_TRACK_ACCEL  = 200.0;

// Homing — fast search, slow for edge detection
const float AZ_HOME_FAST     = 200.0;    // fast search speed
const float AZ_HOME_SLOW     = 150.0;    // slow edge detection
const float AZ_HOME_ACCEL    = 200.0;
const float ZEN_HOME_FAST    = 250.0;
const float ZEN_HOME_SLOW    = 120.0;
const float ZEN_HOME_ACCEL   = 250.0;


// STEPPER OBJECTS

AccelStepper azMotor (AccelStepper::DRIVER, AZ_PULSE_PIN,  AZ_DIR_PIN);
AccelStepper zenMotor(AccelStepper::DRIVER, ZEN_PULSE_PIN, ZEN_DIR_PIN);


// TOWER TARGET COORDINATES (inches, relative to mirror)

float xTower = 0;
float yTower = -24.75;
float zTower = 55.4 - 30.825;
float rTower;


// SERIAL STATE

String serial2Buffer = "";
bool   newSunData    = false;
float  AzInput       = 0;
float  ZenInput      = 0;


// FORWARD DECLARATIONS

void waitForStart();
void HomingSequence();
void processIncoming(String line);
void updateMirrorTarget(float sunAz, float sunZen);
float xBisect(float az, float zen, float x, float r);
float yBisect(float az, float zen, float y, float r);
float zBisect(float az, float zen, float z, float r);
float magBisect(float x, float y, float z);
float azBisect(float x, float y);
float zenBisect(float x, float y, float z);
float Az2Print(float Az);


// SETUP

void setup() {
    pinMode(PROX_PIN,      INPUT);
    pinMode(LIMIT_MAX_PIN, INPUT_PULLUP);
    pinMode(LIMIT_MIN_PIN, INPUT_PULLUP);

    Serial.begin(115200);    // USB debug (was 9600 — too slow, caused stutter)
    Serial2.begin(115200);

    delay(500);
    while (Serial2.available()) Serial2.read();

    // Configure steppers
    azMotor.setMinPulseWidth(10);
    azMotor.setMaxSpeed(AZ_TRACK_SPEED);
    azMotor.setAcceleration(AZ_TRACK_ACCEL);

    zenMotor.setMinPulseWidth(10);
    zenMotor.setMaxSpeed(ZEN_TRACK_SPEED);
    zenMotor.setAcceleration(ZEN_TRACK_ACCEL);

    rTower = sqrt(pow(xTower, 2) + pow(yTower, 2) + pow(zTower, 2));

    Serial.println("========================================");
    Serial.println("  TOWER MIRROR CONTROLLER");
    Serial.println("========================================");
    Serial.print("  Target: x="); Serial.print(xTower, 2);
    Serial.print("  y=");         Serial.print(yTower, 2);
    Serial.print("  z=");         Serial.println(zTower, 2);
    Serial.print("  Target distance: ");
    Serial.print(rTower, 2);
    Serial.println(" inches");
    Serial.println();

    Serial.println("  Sending READY to simulator...");
    waitForStart();

    Serial.println("========================================");
    Serial.println("  HOMING SEQUENCE — STARTING");
    Serial.println("========================================");
    HomingSequence();

    // Zero position after homing
    azMotor.setCurrentPosition(0);
    zenMotor.setCurrentPosition(0);

    // Restore tracking speeds
    azMotor.setMaxSpeed(AZ_TRACK_SPEED);
    azMotor.setAcceleration(AZ_TRACK_ACCEL);
    zenMotor.setMaxSpeed(ZEN_TRACK_SPEED);
    zenMotor.setAcceleration(ZEN_TRACK_ACCEL);

    Serial.println("========================================");
    Serial.println("  HOMING SEQUENCE — COMPLETE");
    Serial.println("========================================");
    Serial.println();

    Serial2.println("HOMED");
    Serial.println("  HOMED signal sent to simulator.");
    Serial.println("  Entering tracking loop.");
    Serial.println();
}


// WAIT FOR START

void waitForStart() {
    String buf = "";
    unsigned long lastSend = 0;

    while (true) {
        if (millis() - lastSend >= 500) {
            Serial2.println("READY");
            Serial.println("  → READY");
            buf = "";
            lastSend = millis();
        }

        while (Serial2.available()) {
            char c = Serial2.read();
            if (c == '\n') {
                buf.trim();
                if (buf == "START") {
                    Serial.println("  ← START received!");
                    return;
                }
                buf = "";
            } else if (isPrintable(c)) {
                buf += c;
            }
        }
    }
}

// MAIN LOOP — non-blocking, smooth motion
void loop() {
    // Motors FIRST — this must run as fast as possible
    azMotor.run();
    zenMotor.run();

    // Read incoming serial data
    while (Serial2.available()) {
        char c = Serial2.read();
        if (c == '\n') {
            processIncoming(serial2Buffer);
            serial2Buffer = "";
        } else {
            serial2Buffer += c;
        }
    }

    // Process new sun data — just update targets, don't block
    if (newSunData) {
        newSunData = false;
        updateMirrorTarget(AzInput, ZenInput);
    }

    // Call run() again — double-tap for maximum step rate
    azMotor.run();
    zenMotor.run();
}

// SERIAL2 PARSING
void processIncoming(String line) {
    line.trim();
    if (line.length() == 0)    return;
    if (line.charAt(0) != 'H') return;
    int commaIdx = line.indexOf(',');
    if (commaIdx < 0)          return;

    AzInput  = line.substring(1, commaIdx).toFloat();
    float el = line.substring(commaIdx + 1).toFloat();
    ZenInput = 90.0 - el;    // elevation → zenith

    newSunData = true;
}

// MIRROR TARGET — computes bisector and sets AccelStepper targets
// No Serial.print here — printing at 9600 baud blocks run() and
// causes motor stutter. Use debugMirror flag for troubleshooting.

const bool debugMirror = false;   // set true to enable debug output

void updateMirrorTarget(float sunAz, float sunZen) {
    float xBis = xBisect(sunAz, sunZen, xTower, rTower);
    float yBis = yBisect(sunAz, sunZen, yTower, rTower);
    float zBis = zBisect(sunAz, sunZen, zTower, rTower);
    float rBis = magBisect(xBis, yBis, zBis);

    float mirrorAz  = azBisect(xBis / rBis, yBis / rBis);
    float mirrorZen = zenBisect(xBis / rBis, yBis / rBis, zBis / rBis);

    if (debugMirror) {
        Serial.print("  Sun: az=");
        Serial.print(sunAz, 2);
        Serial.print("°  zen=");
        Serial.print(sunZen, 2);
        Serial.print("°  →  az=");
        Serial.print(Az2Print(mirrorAz), 2);
        Serial.print("°  zen=");
        Serial.print(mirrorZen, 2);
        Serial.println("°");
    }

    // Convert mirror angles to step targets
    float targetAzDeg = mirrorAz;
    if (targetAzDeg < 0) targetAzDeg += 360.0;

    float currentAzDeg = (float)azMotor.currentPosition() / AZ_STEPS_PER_DEG;
    if (currentAzDeg < 0) currentAzDeg += 360.0;

    float azDiff = targetAzDeg - currentAzDeg;
    if (azDiff < -180.0) azDiff += 360.0;
    if (azDiff >  180.0) azDiff -= 360.0;

    long azTarget = azMotor.currentPosition() + (long)(azDiff * AZ_STEPS_PER_DEG);

    mirrorZen = constrain(mirrorZen, 0.0, 90.0);
    long zenTarget = (long)(mirrorZen * ZEN_STEPS_PER_DEG);

    azMotor.moveTo(azTarget);
    zenMotor.moveTo(zenTarget);
}

// VECTOR MATH (unchanged)
float xBisect(float az, float zen, float x, float r) {
    return (x / r) + sin(zen * M_PI / 180) * cos(az * M_PI / 180);
}
float yBisect(float az, float zen, float y, float r) {
    return (y / r) + sin(zen * M_PI / 180) * sin(az * M_PI / 180);
}
float zBisect(float az, float zen, float z, float r) {
    return (z / r) + cos(zen * M_PI / 180);
}
float magBisect(float x, float y, float z) {
    return sqrt(x * x + y * y + z * z);
}
float azBisect(float x, float y) {
    return atan2(-y, x) * 180.0 / M_PI;
}
float zenBisect(float x, float y, float z) {
    return atan2(sqrt(x * x + y * y), z) * 180.0 / M_PI;
}
float Az2Print(float Az) {
    Az = -Az;
    if (Az < 0) Az += 360;
    return Az;
}

// HOMING SEQUENCE — two-speed: fast search, slow edge detection
void HomingSequence() {

    // Azimuth homing
    Serial.println("  [AZ]  Starting azimuth homing...");

    azMotor.setMaxSpeed(AZ_HOME_FAST);
    azMotor.setAcceleration(AZ_HOME_ACCEL);

    // If starting on the metal strip, back off at fast speed
    if (digitalRead(PROX_PIN) == LOW) {
        Serial.println("  [AZ]  On strip — backing off (fast)...");
        azMotor.move(-MICROSTEPS_REV * GEAR_RATIO / 12);
        while (azMotor.distanceToGo() != 0 && digitalRead(PROX_PIN) == LOW) {
            azMotor.run();
        }
        azMotor.stop();
        while (azMotor.run()) {}
    }

    // Fast search — just get near the strip (overshoot is fine)
    Serial.println("  [AZ]  Fast search for strip...");
    azMotor.move(MICROSTEPS_REV * GEAR_RATIO);
    while (azMotor.distanceToGo() != 0) {
        azMotor.run();
        if (digitalRead(PROX_PIN) == LOW) break;
    }
    azMotor.stop();
    while (azMotor.run()) {}
    Serial.println("  [AZ]  Strip detected — overshot is OK.");

    // Back off past the strip so we approach from a known side
    Serial.println("  [AZ]  Backing off for slow approach...");
    azMotor.setMaxSpeed(AZ_HOME_SLOW);
    azMotor.move(-MICROSTEPS_REV * GEAR_RATIO / 8);
    while (azMotor.distanceToGo() != 0) {
        azMotor.run();
        if (digitalRead(PROX_PIN) == HIGH) break;  // cleared the strip
    }
    // Keep going a small margin past the edge
    azMotor.move(-200);
    while (azMotor.distanceToGo() != 0) {
        azMotor.run();
    }

    // Slow approach to find leading edge precisely
    Serial.println("  [AZ]  Slow approach — finding leading edge...");
    azMotor.move(MICROSTEPS_REV * GEAR_RATIO / 4);
    while (azMotor.distanceToGo() != 0) {
        azMotor.run();
        if (digitalRead(PROX_PIN) == LOW) break;
    }
    azMotor.setCurrentPosition(0);  // mark leading edge as reference
    Serial.println("  [AZ]  Leading edge found.");

    // Slow pass to find trailing edge and measure width
    Serial.println("  [AZ]  Finding trailing edge...");
    azMotor.move(MICROSTEPS_REV * GEAR_RATIO / 4);
    while (azMotor.distanceToGo() != 0) {
        azMotor.run();
        if (digitalRead(PROX_PIN) == HIGH) break;
    }
    long stripWidth = azMotor.currentPosition();

    Serial.print("  [AZ]  Strip width: ");
    Serial.print(stripWidth);
    Serial.println(" steps — centering...");

    // Center on strip
    azMotor.moveTo(stripWidth / 2);
    while (azMotor.distanceToGo() != 0) {
        azMotor.run();
    }
    Serial.println("  [AZ]  Azimuth homed.");

    // Zenith homing
    Serial.println("  [ZEN] Starting zenith homing...");

    zenMotor.setMaxSpeed(ZEN_HOME_FAST);
    zenMotor.setAcceleration(ZEN_HOME_ACCEL);

    // Zenith homing
    // Direction reference (from original working code):
    //   Positive speed (DIR HIGH / CCW) = toward MAX limit
    //   Negative speed (DIR LOW  / CW)  = toward MIN limit
    Serial.println("  [ZEN] Starting zenith homing...");

    // If at max limit, back off AWAY from max (negative = toward min)
    if (digitalRead(LIMIT_MAX_PIN) == LOW) {
        Serial.println("  [ZEN] At max limit — backing off...");
        zenMotor.setSpeed(-ZEN_HOME_SLOW);
        while (digitalRead(LIMIT_MAX_PIN) == LOW) {
            zenMotor.runSpeed();
        }
        for (int i = 0; i < 200; i++) zenMotor.runSpeed();
    }

    // Search for positive/max limit (positive = toward max)
    Serial.println("  [ZEN] Searching for positive limit...");
    zenMotor.setSpeed(ZEN_HOME_SLOW);
    while (digitalRead(LIMIT_MAX_PIN) == HIGH) {
        zenMotor.runSpeed();
    }
    zenMotor.setCurrentPosition(0);
    Serial.println("  [ZEN] Positive limit found.");
    delay(300);

    // Back off from max (negative = away from max, toward min)
    zenMotor.setSpeed(-ZEN_HOME_SLOW);
    for (int i = 0; i < 300; i++) zenMotor.runSpeed();

    // Search for negative/min limit (negative = toward min)
    Serial.println("  [ZEN] Searching for negative limit...");
    zenMotor.setSpeed(-ZEN_HOME_SLOW);
    while (digitalRead(LIMIT_MIN_PIN) == HIGH) {
        zenMotor.runSpeed();
    }
    long zenTravel = abs(zenMotor.currentPosition());
    Serial.print("  [ZEN] Negative limit found. Travel: ");
    Serial.print(zenTravel);
    Serial.println(" steps.");
    delay(200);

    // Back off from min (positive = away from min, toward max)
    zenMotor.setSpeed(ZEN_HOME_SLOW);
    for (int i = 0; i < 300; i++) zenMotor.runSpeed();

    // Center — max limit was position 0, so center is at -(zenTravel/2)
    Serial.println("  [ZEN] Centering...");
    long centerPos = -(zenTravel / 2);
    zenMotor.setMaxSpeed(ZEN_HOME_FAST);
    zenMotor.setAcceleration(ZEN_HOME_ACCEL);
    zenMotor.moveTo(centerPos);
    while (zenMotor.distanceToGo() != 0) {
        zenMotor.run();
    }
    Serial.println("  [ZEN] Zenith homed.");
}
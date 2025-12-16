#include <MKSServoE.h>
#include <transport/adapters/AdapterSelector.h>

CanBusAdapter bus;
MKSServoE servo(bus);

const uint16_t kServoId = 0x01;
const uint16_t kSafeSpeedRpm = 200;
const uint8_t kSafeAcc = 5;
const uint32_t kMoveWindowMs = 3500;

unsigned long lastTelemetryMs = 0;
unsigned long moveStartMs = 0;
uint8_t moveIndex = 0;
bool errorLatched = false;

static void printStatus(const char *label, MKSServoE::ERROR rc, uint8_t status) {
  Serial.print(label);
  Serial.print(": ");
  Serial.print(rc == MKSServoE::ERROR_OK ? "ok" : "err");
  Serial.print(" (status=0x");
  Serial.print(status, HEX);
  Serial.println(")");
}

static void safeDisable() {
  uint8_t status = 0;
  MKSServoE::ERROR rc = servo.disable();
  if (rc == MKSServoE::ERROR_OK) {
    status = 1;
  }
  printStatus("Disabled (safety)", rc, status);
  errorLatched = true;
}

static bool sendMove(uint8_t index) {
  uint8_t status = 0;
  bool ok = false;
  switch (index) {
    case 0: {
      MKSServoE::ERROR rc = servo.runPositionMode1Relative(0, kSafeSpeedRpm, kSafeAcc, 1600, status);
      printStatus("Mode1 rel +1600 pulses", rc, status);
      ok = (rc == MKSServoE::ERROR_OK);
      break;
    }
    case 1: {
      MKSServoE::ERROR rc = servo.runPositionMode2Absolute(0, kSafeSpeedRpm, kSafeAcc, 3200, status);
      printStatus("Mode2 abs 3200 pulses", rc, status);
      ok = (rc == MKSServoE::ERROR_OK);
      break;
    }
    case 2: {
      MKSServoE::ERROR rc = servo.runPositionMode3RelativeAxis(kSafeSpeedRpm, kSafeAcc, 0x0800, status);
      printStatus("Mode3 rel +0x0800 axis", rc, status);
      ok = (rc == MKSServoE::ERROR_OK);
      break;
    }
    case 3: {
      MKSServoE::ERROR rc = servo.runPositionMode4AbsoluteAxis(kSafeSpeedRpm, kSafeAcc, 0x1000, status);
      printStatus("Mode4 abs 0x1000 axis", rc, status);
      ok = (rc == MKSServoE::ERROR_OK);
      break;
    }
    default: {
      ok = false;
      status = 0;
      break;
    }
  }
  moveStartMs = millis();
  return ok;
}

static void printTelemetry() {
  int16_t rpm = 0;
  int64_t pos = 0;
  uint8_t busStatus = 0;
  const bool haveStatus = (servo.queryBusStatus(busStatus) == MKSServoE::ERROR_OK);
  const bool haveSpeed = (servo.readSpeedRpm(rpm) == MKSServoE::ERROR_OK);
  const bool havePos = (servo.readEncoderAddition(pos) == MKSServoE::ERROR_OK);
  if (haveStatus) {
    Serial.print("Bus status=0x");
    Serial.print(busStatus, HEX);
    Serial.print(" ");
  }
  if (haveSpeed && havePos) {
    Serial.print("rpm=");
    Serial.print(rpm);
    Serial.print(" pos=");
    Serial.print((long)pos);
    Serial.println();
  } else {
    Serial.println("Telemetry read failed");
  }
}

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 2000) {}

  if (!bus.begin(500000)) {
    Serial.println("CAN init failed");
    return;
  }

  servo.setTargetId(kServoId);
  servo.setTxId(kServoId);

  MKSServoE::ERROR rc = servo.enable();
  if (rc != MKSServoE::ERROR_OK) {
    Serial.println("Enable failed");
    errorLatched = true;
    return;
  }

  uint8_t status = 0;
  rc = servo.setMode(0x05, status);
  if (rc != MKSServoE::ERROR_OK) {
    Serial.println("Mode init failed");
  }
  rc = servo.setCurrentMa(1200, status);
  if (rc != MKSServoE::ERROR_OK) {
    Serial.println("Current init failed");
  }
  rc = servo.setMicrostep(16, status);
  if (rc != MKSServoE::ERROR_OK) {
    Serial.println("Microstep init failed");
  }
  rc = servo.setAxisZero(status);
  if (rc != MKSServoE::ERROR_OK) {
    Serial.println("Axis zero init failed");
  }

  sendMove(moveIndex);
}

void loop() {
  if (errorLatched) {
    return;
  }

  const unsigned long now = millis();
  if (now - lastTelemetryMs >= 500) {
    lastTelemetryMs = now;
    printTelemetry();
  }

  if (now - moveStartMs >= kMoveWindowMs) {
    moveIndex = (uint8_t)((moveIndex + 1) % 4);
    if (!sendMove(moveIndex)) {
      safeDisable();
    }
  }
}

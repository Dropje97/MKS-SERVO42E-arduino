#include <MKSServoE.h>
#include <transport/adapters/UnoR4CanBus.h>

UnoR4CanBus bus;
MKSServoE servo(bus);

const uint16_t kServoId = 0x01;
const uint16_t kSafeSpeedRpm = 200;
const uint8_t kSafeAcc = 5;
const uint32_t kMoveWindowMs = 3500;

unsigned long lastTelemetryMs = 0;
unsigned long moveStartMs = 0;
uint8_t moveIndex = 0;
bool errorLatched = false;

static void printStatus(const char *label, bool ok, uint8_t status) {
  Serial.print(label);
  Serial.print(": ");
  Serial.print(ok ? "ok" : "err");
  Serial.print(" (status=0x");
  Serial.print(status, HEX);
  Serial.println(")");
}

static void safeDisable() {
  uint8_t status = 0;
  if (servo.disable()) {
    status = 1;
  }
  printStatus("Disabled (safety)", true, status);
  errorLatched = true;
}

static bool sendMove(uint8_t index) {
  uint8_t status = 0;
  bool ok = false;
  switch (index) {
    case 0: {
      ok = servo.runPositionMode1Relative(0, kSafeSpeedRpm, kSafeAcc, 1600, status);
      printStatus("Mode1 rel +1600 pulses", ok, status);
      break;
    }
    case 1: {
      ok = servo.runPositionMode2Absolute(0, kSafeSpeedRpm, kSafeAcc, 3200, status);
      printStatus("Mode2 abs 3200 pulses", ok, status);
      break;
    }
    case 2: {
      ok = servo.runPositionMode3RelativeAxis(kSafeSpeedRpm, kSafeAcc, 0x0800, status);
      printStatus("Mode3 rel +0x0800 axis", ok, status);
      break;
    }
    case 3: {
      ok = servo.runPositionMode4AbsoluteAxis(kSafeSpeedRpm, kSafeAcc, 0x1000, status);
      printStatus("Mode4 abs 0x1000 axis", ok, status);
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
  bool haveStatus = servo.queryBusStatus(busStatus);
  bool haveSpeed = servo.readSpeedRpm(rpm);
  bool havePos = servo.readEncoderAddition(pos);
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

  if (!servo.enable()) {
    Serial.println("Enable failed");
    errorLatched = true;
    return;
  }

  uint8_t status = 0;
  servo.setMode(0x05, status);
  servo.setCurrentMa(1200, status);
  servo.setMicrostep(16, status);
  servo.setAxisZero(status);

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

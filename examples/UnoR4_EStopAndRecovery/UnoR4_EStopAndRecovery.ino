#include <MKSServoE.h>
#include <transport/adapters/UnoR4CanBus.h>

UnoR4CanBus bus;
MKSServoE servo(bus);

const uint16_t kServoId = 0x01;
const uint16_t kRunSpeedRpm = 300;
const uint8_t kRunAcc = 4;

enum class DemoState {
  Idle,
  Spinning,
  EStopped,
  Recovering,
  Done
};

DemoState state = DemoState::Idle;
unsigned long stateStartMs = 0;
unsigned long lastPollMs = 0;
bool errorLatched = false;

static void logStatus(const char *label, bool ok, uint8_t status) {
  Serial.print(label);
  Serial.print(": ");
  Serial.print(ok ? "ok" : "err");
  Serial.print(" (status=0x");
  Serial.print(status, HEX);
  Serial.println(")");
}

static void safeDisable() {
  uint8_t status = 0;
  servo.disable();
  Serial.println("Motor disabled for safety");
  errorLatched = true;
}

static void startSpin() {
  uint8_t status = 0;
  bool ok = servo.runSpeed(0, kRunSpeedRpm, kRunAcc, status);
  logStatus("Run speed", ok, status);
  state = ok ? DemoState::Spinning : DemoState::Idle;
  stateStartMs = millis();
}

static void doEmergencyStop() {
  uint8_t status = 0;
  bool ok = servo.emergencyStop(status);
  logStatus("Emergency stop", ok, status);
  state = ok ? DemoState::EStopped : DemoState::Idle;
  stateStartMs = millis();
}

static void recoverFromStop() {
  uint8_t status = 0;
  servo.releaseStallProtection(status);
  servo.disable();
  bool ok = servo.enable();
  Serial.print("Re-enable after stop: ");
  Serial.println(ok ? "ok" : "err");
  state = ok ? DemoState::Recovering : DemoState::Idle;
  stateStartMs = millis();
}

static void printDiagnostics() {
  uint8_t stall = 0;
  uint8_t en = 0;
  uint8_t busStatus = 0;
  servo.readStallState(stall);
  servo.readEnStatus(en);
  servo.queryBusStatus(busStatus);
  Serial.print("Diag stall=");
  Serial.print(stall);
  Serial.print(" en=");
  Serial.print(en);
  Serial.print(" bus=0x");
  Serial.println(busStatus, HEX);
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
    return;
  }

  uint8_t status = 0;
  servo.setMode(0x05, status);
  servo.setCurrentMa(1400, status);
  servo.setMicrostep(16, status);

  startSpin();
}

void loop() {
  if (errorLatched) {
    return;
  }

  const unsigned long now = millis();
  if (now - lastPollMs >= 500) {
    lastPollMs = now;
    printDiagnostics();
  }

  switch (state) {
    case DemoState::Spinning: {
      if (now - stateStartMs >= 3000) {
        doEmergencyStop();
      }
      break;
    }
    case DemoState::EStopped: {
      if (now - stateStartMs >= 800) {
        recoverFromStop();
      }
      break;
    }
    case DemoState::Recovering: {
      if (now - stateStartMs >= 800) {
        startSpin();
      }
      break;
    }
    case DemoState::Idle: {
      safeDisable();
      break;
    }
    case DemoState::Done: {
      break;
    }
  }
}

#include <MKSServoE.h>
#include <transport/adapters/AdapterSelector.h>

CanBusAdapter bus;
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

static void logStatus(const char *label, MKSServoE::ERROR rc, uint8_t status) {
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
  if (rc != MKSServoE::ERROR_OK) {
    Serial.println("Disable command failed");
  }
  Serial.println("Motor disabled for safety");
  errorLatched = true;
}

static void startSpin() {
  uint8_t status = 0;
  MKSServoE::ERROR rc = servo.runSpeed(0, kRunSpeedRpm, kRunAcc, status);
  logStatus("Run speed", rc, status);
  state = (rc == MKSServoE::ERROR_OK) ? DemoState::Spinning : DemoState::Idle;
  stateStartMs = millis();
}

static void doEmergencyStop() {
  uint8_t status = 0;
  MKSServoE::ERROR rc = servo.emergencyStop(status);
  logStatus("Emergency stop", rc, status);
  state = (rc == MKSServoE::ERROR_OK) ? DemoState::EStopped : DemoState::Idle;
  stateStartMs = millis();
}

static void recoverFromStop() {
  uint8_t status = 0;
  MKSServoE::ERROR rc = servo.releaseStallProtection(status);
  if (rc != MKSServoE::ERROR_OK) {
    Serial.println("Release stall protection failed");
  }
  rc = servo.disable();
  if (rc != MKSServoE::ERROR_OK) {
    Serial.println("Disable failed during recovery");
  }
  rc = servo.enable();
  Serial.print("Re-enable after stop: ");
  Serial.println(rc == MKSServoE::ERROR_OK ? "ok" : "err");
  state = (rc == MKSServoE::ERROR_OK) ? DemoState::Recovering : DemoState::Idle;
  stateStartMs = millis();
}

static void printDiagnostics() {
  uint8_t stall = 0;
  uint8_t en = 0;
  uint8_t busStatus = 0;
  MKSServoE::ERROR rc = servo.readStallState(stall);
  if (rc != MKSServoE::ERROR_OK) {
    Serial.println("Stall state read failed");
  }
  rc = servo.readEnStatus(en);
  if (rc != MKSServoE::ERROR_OK) {
    Serial.println("EN state read failed");
  }
  rc = servo.queryBusStatus(busStatus);
  if (rc != MKSServoE::ERROR_OK) {
    Serial.println("Bus status read failed");
  }
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

  MKSServoE::ERROR rc = servo.enable();
  if (rc != MKSServoE::ERROR_OK) {
    Serial.println("Enable failed");
    return;
  }

  uint8_t status = 0;
  rc = servo.setMode(0x05, status);
  if (rc != MKSServoE::ERROR_OK) {
    Serial.println("Mode init failed");
  }
  rc = servo.setCurrentMa(1400, status);
  if (rc != MKSServoE::ERROR_OK) {
    Serial.println("Current init failed");
  }
  rc = servo.setMicrostep(16, status);
  if (rc != MKSServoE::ERROR_OK) {
    Serial.println("Microstep init failed");
  }

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

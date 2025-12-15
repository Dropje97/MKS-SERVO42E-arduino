#include <MKSServoE.h>
#include <transport/adapters/UnoR4CanBus.h>

UnoR4CanBus bus;
MKSServoE servo(bus);

const uint16_t kServoId = 0x01;
const bool kWriteUserId = false;           // Set to true to push kDemoUserId.
const uint32_t kDemoUserId = 0x12345678;   // Demo payload for CMD 0x42 write.

static void printStatus(const char *label, bool ok, uint8_t status) {
  Serial.print(label);
  Serial.print(": ");
  Serial.print(ok ? "ok" : "err");
  Serial.print(" (status=0x");
  Serial.print(status, HEX);
  Serial.println(")");
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

  uint8_t status = 0;
  if (!servo.enable()) {
    Serial.println("Bus enable failed");
  }

  uint32_t userId = 0;
  if (servo.readUserId(userId)) {
    Serial.print("User ID: 0x");
    Serial.println(userId, HEX);
  } else {
    Serial.println("User ID read failed");
  }

  if (kWriteUserId) {
    if (servo.writeUserId(kDemoUserId, status)) {
      printStatus("User ID write", true, status);
    } else {
      printStatus("User ID write", false, status);
    }
  }

  bool okMode = servo.setMode(0x05, status);
  printStatus("Mode (05=bus FOC)", okMode, status);

  bool okCurrent = servo.setCurrentMa(1600, status);
  printStatus("Current (mA)", okCurrent, status);

  bool okMicro = servo.setMicrostep(16, status);
  printStatus("Microstep", okMicro, status);

  bool okDir = servo.setDirection(0, status);
  printStatus("Direction (0=CW)", okDir, status);

  bool okEn = servo.setEnActive(0, status);
  printStatus("EN active-low", okEn, status);

  bool okDelay = servo.setPulseDelay(2, status);
  printStatus("Pulse delay (20ms)", okDelay, status);

  Serial.println("Running encoder calibration...");
  uint8_t calStatus = 0;
  if (servo.calibrateEncoder(calStatus)) {
    Serial.print("Calibration status: ");
    Serial.println(calStatus); // 0=running,1=success,2=fail
  } else {
    Serial.println("Calibration command failed");
  }
}

void loop() {
  static unsigned long lastStatus = 0;
  if (millis() - lastStatus >= 1000) {
    lastStatus = millis();
    uint8_t status = 0;
    if (servo.queryBusStatus(status)) {
      Serial.print("Bus status: 0x");
      Serial.println(status, HEX);
    }
  }
}

#include <MKSServoE.h>
#include <transport/adapters/UnoR4CanBus.h>

UnoR4CanBus bus;
MKSServoE servo(bus);

const uint16_t kServoId = 0x01;
const bool kWriteUserId = false;           // Set to true to push kDemoUserId.
const uint32_t kDemoUserId = 0x12345678;   // Demo payload for CMD 0x42 write.

static void printStatus(const char *label, MKSServoE::ERROR rc, uint8_t status) {
  const bool ok = (rc == MKSServoE::ERROR_OK);
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
  MKSServoE::ERROR rc = servo.enable();
  if (rc != MKSServoE::ERROR_OK) {
    Serial.println("Bus enable failed");
  }

  uint32_t userId = 0;
  rc = servo.readUserId(userId);
  if (rc == MKSServoE::ERROR_OK) {
    Serial.print("User ID: 0x");
    Serial.println(userId, HEX);
  } else {
    Serial.println("User ID read failed");
  }

  if (kWriteUserId) {
    rc = servo.writeUserId(kDemoUserId, status);
    printStatus("User ID write", rc, status);
  }

  rc = servo.setMode(0x05, status);
  printStatus("Mode (05=bus FOC)", rc, status);

  rc = servo.setCurrentMa(1600, status);
  printStatus("Current (mA)", rc, status);

  rc = servo.setMicrostep(16, status);
  printStatus("Microstep", rc, status);

  rc = servo.setDirection(0, status);
  printStatus("Direction (0=CW)", rc, status);

  rc = servo.setEnActive(0, status);
  printStatus("EN active-low", rc, status);

  rc = servo.setPulseDelay(2, status);
  printStatus("Pulse delay (20ms)", rc, status);

  Serial.println("Running encoder calibration...");
  uint8_t calStatus = 0;
  rc = servo.calibrateEncoder(calStatus);
  if (rc == MKSServoE::ERROR_OK) {
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
    MKSServoE::ERROR rc = servo.queryBusStatus(status);
    if (rc == MKSServoE::ERROR_OK) {
      Serial.print("Bus status: 0x");
      Serial.println(status, HEX);
    }
  }
}

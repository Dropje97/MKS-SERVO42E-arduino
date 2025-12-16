#include <MKSServoE.h>
#include <transport/adapters/AdapterSelector.h>

CanBusAdapter bus;
MKSServoE servo(bus);

const uint16_t kServoId = 0x01;
const uint32_t kPollMs = 1500;
unsigned long lastPollMs = 0;
bool errorLatched = false;

static void safeDisable() {
  MKSServoE::ERROR rc = servo.disable();
  if (rc != MKSServoE::ERROR_OK) {
    Serial.println("Disable command failed");
  }
  Serial.println("Motor disabled (error)");
  errorLatched = true;
}

static bool readParamValue(uint8_t code, uint8_t *buf, uint8_t &len) {
  uint8_t outLen = 0;
  MKSServoE::ERROR rc = servo.readParam(code, buf, len, outLen);
  len = outLen;
  return rc == MKSServoE::ERROR_OK;
}

static void printCanConfig() {
  uint8_t buf[4];
  uint8_t len = sizeof(buf);
  if (readParamValue(0x8A, buf, len)) {
    Serial.print("CAN bitrate code=0x");
    Serial.println(buf[0], HEX);
  } else {
    Serial.println("Read CAN bitrate failed");
  }

  len = sizeof(buf);
  if (readParamValue(0x8B, buf, len) && len >= 2) {
    uint16_t id = (uint16_t)buf[0] | ((uint16_t)buf[1] << 8);
    Serial.print("CAN ID=0x");
    Serial.println(id, HEX);
  } else {
    Serial.println("Read CAN ID failed");
  }

  len = sizeof(buf);
  if (readParamValue(0x8C, buf, len) && len >= 2) {
    Serial.print("Respond=");
    Serial.print(buf[0]);
    Serial.print(" Active=");
    Serial.println(buf[1]);
  } else {
    Serial.println("Read respond/active failed");
  }

  len = sizeof(buf);
  if (readParamValue(0x8D, buf, len) && len >= 2) {
    uint16_t groupId = (uint16_t)buf[0] | ((uint16_t)buf[1] << 8);
    Serial.print("Group ID=0x");
    Serial.println(groupId, HEX);
  } else {
    Serial.println("Read group ID failed");
  }

  len = sizeof(buf);
  if (readParamValue(0x8F, buf, len)) {
    Serial.print("Lock axis=");
    Serial.println(buf[0]);
  } else {
    Serial.println("Read lock axis failed");
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
}

void loop() {
  if (errorLatched) {
    return;
  }

  const unsigned long now = millis();
  if (now - lastPollMs < kPollMs) {
    return;
  }
  lastPollMs = now;

  uint8_t status = 0;
  MKSServoE::ERROR rc = servo.queryBusStatus(status);
  if (rc != MKSServoE::ERROR_OK) {
    Serial.println("Bus status read failed");
    safeDisable();
    return;
  }
  Serial.print("Bus status=0x");
  Serial.println(status, HEX);

  printCanConfig();
}

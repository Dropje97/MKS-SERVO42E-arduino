#include <MKSServoE.h>
#include <transport/adapters/AdapterSelector.h>

CanBusAdapter bus;
MKSServoE servo(bus);

const uint16_t kServoId = 0x01;
const uint32_t kPollIntervalMs = 800;
unsigned long lastPollMs = 0;
bool errorLatched = false;

static void printIoStatus(uint8_t io) {
  const bool alarm = (io & 0x40) != 0;
  const bool pend = (io & 0x20) != 0;
  const bool in1 = (io & 0x08) != 0;
  const bool en = (io & 0x02) != 0;
  Serial.print("IO ALM=");
  Serial.print(alarm ? "1" : "0");
  Serial.print(" PEND=");
  Serial.print(pend ? "1" : "0");
  Serial.print(" IN1=");
  Serial.print(in1 ? "1" : "0");
  Serial.print(" EN=");
  Serial.println(en ? "1" : "0");
}

static void safeDisable() {
  MKSServoE::ERROR rc = servo.disable();
  if (rc != MKSServoE::ERROR_OK) {
    Serial.println("Disable command failed");
  }
  Serial.println("Motor disabled (error)");
  errorLatched = true;
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
}

void loop() {
  if (errorLatched) {
    return;
  }

  const unsigned long now = millis();
  if (now - lastPollMs < kPollIntervalMs) {
    return;
  }
  lastPollMs = now;

  uint8_t io = 0;
  int32_t pulses = 0;
  int32_t posError = 0;
  uint8_t en = 0;
  int16_t rpm = 0;
  int64_t position = 0;

  MKSServoE::ERROR rc = servo.readIoStatus(io);
  if (rc == MKSServoE::ERROR_OK) {
    rc = servo.readInputPulses(pulses);
  }
  if (rc == MKSServoE::ERROR_OK) {
    rc = servo.readPositionError(posError);
  }
  if (rc == MKSServoE::ERROR_OK) {
    rc = servo.readEnStatus(en);
  }
  if (rc == MKSServoE::ERROR_OK) {
    rc = servo.readSpeedRpm(rpm);
  }
  if (rc == MKSServoE::ERROR_OK) {
    rc = servo.readEncoderAddition(position);
  }

  if (rc != MKSServoE::ERROR_OK) {
    Serial.println("Read failed; disabling");
    safeDisable();
    return;
  }

  printIoStatus(io);
  Serial.print("Input pulses=");
  Serial.println(pulses);
  Serial.print("Position error=");
  Serial.println(posError);
  Serial.print("EN pin=");
  Serial.println(en);
  Serial.print("Speed rpm=");
  Serial.print(rpm);
  Serial.print(" Position=");
  Serial.println((long)position);
}

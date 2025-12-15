#include <MKSServoE.h>
#include <transport/adapters/UnoR4CanBus.h>

UnoR4CanBus bus;
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
  servo.disable();
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
  if (!servo.enable()) {
    Serial.println("Enable failed");
    errorLatched = true;
    return;
  }

  uint8_t status = 0;
  servo.setMode(0x05, status);
  servo.setCurrentMa(1200, status);
  servo.setMicrostep(16, status);
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

  bool ok = servo.readIoStatus(io);
  ok = ok && servo.readInputPulses(pulses);
  ok = ok && servo.readPositionError(posError);
  ok = ok && servo.readEnStatus(en);
  ok = ok && servo.readSpeedRpm(rpm);
  ok = ok && servo.readEncoderAddition(position);

  if (!ok) {
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

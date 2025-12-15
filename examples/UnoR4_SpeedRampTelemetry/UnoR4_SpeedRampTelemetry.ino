#include <MKSServoE.h>
#include <transport/adapters/UnoR4CanBus.h>

UnoR4CanBus bus;
MKSServoE servo(bus);

struct RampStep {
  uint16_t speed;
  uint8_t dir;
  uint16_t holdMs;
};

const RampStep kSteps[] = {
  {200, 0, 4000},
  {600, 0, 4000},
  {1000, 1, 4000},
  {300, 1, 4000},
  {0, 0, 4000}, // glide to stop
};

const uint8_t kAcc = 8;
size_t stepIndex = 0;
unsigned long stepStartMs = 0;
unsigned long lastTelemetryMs = 0;

static void startStep() {
  const RampStep &step = kSteps[stepIndex];
  uint8_t status = 0;
  if (servo.runSpeed(step.dir, step.speed, kAcc, status)) {
    Serial.print("Speed cmd ");
    Serial.print(step.speed);
    Serial.print("rpm dir=");
    Serial.print(step.dir ? "REV" : "FWD");
    Serial.print(" status=");
    Serial.println(status);
  } else {
    Serial.print("Speed cmd failed, status=");
    Serial.println(status);
  }
}

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 2000) {}

  if (!bus.begin(500000)) {
    Serial.println("CAN init failed");
    return;
  }

  servo.setTargetId(0x01);
  servo.setTxId(0x01);
  if (!servo.enable()) {
    Serial.println("Enable failed");
  }

  uint8_t status = 0;
  servo.setMode(0x05, status);
  servo.setCurrentMa(1600, status);
  servo.setMicrostep(16, status);

  stepStartMs = millis();
  startStep();
}

void loop() {
  const unsigned long now = millis();
  if (now - stepStartMs >= kSteps[stepIndex].holdMs) {
    stepIndex = (stepIndex + 1) % (sizeof(kSteps) / sizeof(kSteps[0]));
    stepStartMs = now;
    startStep();
  }

  if (now - lastTelemetryMs >= 500) {
    lastTelemetryMs = now;
    int16_t rpm = 0;
    int64_t position = 0;
    if (servo.readSpeedRpm(rpm) && servo.readEncoderAddition(position)) {
      Serial.print("Telemetry rpm=");
      Serial.print(rpm);
      Serial.print(" pos=");
      Serial.println((long)position);
    } else {
      Serial.println("Telemetry read failed");
    }
  }
}

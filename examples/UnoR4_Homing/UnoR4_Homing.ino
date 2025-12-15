#include <MKSServoE.h>
#include <transport/adapters/UnoR4CanBus.h>

UnoR4CanBus bus;
MKSServoE servo(bus);

enum class HomeState {
  Idle,
  Running,
  Done,
  Failed,
};

HomeState state = HomeState::Idle;
unsigned long lastPollMs = 0;

static void requestHome() {
  uint8_t status = 0;
  if (servo.goHome(status)) {
    Serial.print("GoHome status=");
    Serial.println(status);
    if (status == 2) {
      state = HomeState::Done;
    } else if (status == 1) {
      state = HomeState::Running;
    } else {
      state = HomeState::Failed;
    }
  } else {
    Serial.println("GoHome send failed");
    state = HomeState::Failed;
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
  servo.setMode(0x05, status);          // Bus closed-loop FOC
  servo.setCurrentMa(1200, status);
  servo.setMicrostep(16, status);

  // trigLevel=0 (low), homeDir=0 (CW), homeSpeed=120rpm, endLimitEnable=1, mode=0
  if (servo.setHomeConfig(0, 0, 120, 1, 0, status)) {
    Serial.println("Home config set");
  } else {
    Serial.println("Home config failed");
  }

  requestHome();
}

void loop() {
  const unsigned long now = millis();
  if (state == HomeState::Running && now - lastPollMs >= 200) {
    lastPollMs = now;
    uint8_t busStatus = 0;
    if (servo.queryBusStatus(busStatus)) {
      Serial.print("Homing bus status=0x");
      Serial.println(busStatus, HEX);
      if (busStatus == 1) { // stop -> likely homing complete
        uint8_t status = 0;
        servo.setAxisZero(status);
        Serial.println("Homing done, axis zeroed");
        state = HomeState::Done;
      } else if (busStatus == 0) {
        Serial.println("Homing failed (bus status 0)");
        state = HomeState::Failed;
      }
    }
  }
}

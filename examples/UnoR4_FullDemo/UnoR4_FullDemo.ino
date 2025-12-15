#include <MKSServoE.h>
#include <transport/adapters/UnoR4CanBus.h>

UnoR4CanBus bus;
MKSServoE servo(bus);
unsigned long lastQueryMs = 0;

static void printFrame(const CanFrame &f) {
  Serial.print("RX 0x");
  Serial.print(f.id, HEX);
  Serial.print(" [");
  Serial.print(f.dlc);
  Serial.print("]: ");
  for (uint8_t i = 0; i < f.dlc; i++) {
    if (i) Serial.print(' ');
    Serial.print(f.data[i], HEX);
  }
  Serial.println();
}

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 2000) { }

  if (!bus.begin(500000)) {
    Serial.println("CAN init failed");
  }

  servo.setTargetId(0x01);
  servo.enable();
}

void loop() {
  while (bus.available()) {
    CanFrame f;
    if (bus.read(f)) {
      printFrame(f);
    }
  }

  if (millis() - lastQueryMs >= 1000) {
    lastQueryMs = millis();
    uint8_t status = 0;
    if (servo.queryBusStatus(status)) {
      Serial.print("Status: 0x");
      Serial.println(status, HEX);
    } else {
      Serial.println("Status query failed");
    }
  }
}

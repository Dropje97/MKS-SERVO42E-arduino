#include <Arduino.h>
#include "MKSServoE.h"
#include "transport/adapters/UnoR4CanBus.h"

UnoR4CanBus bus;
MKSServoE servo(bus);

void setup() {
  Serial.begin(115200);
  bus.begin(500000);
  servo.setTargetId(0x01);
  servo.enable();
}

void loop() {
}

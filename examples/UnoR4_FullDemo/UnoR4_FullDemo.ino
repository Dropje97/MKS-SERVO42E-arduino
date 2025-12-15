#include <Arduino.h>
#include <Arduino_CAN.h>
#include "MKSServoE.h"

class UnoR4CanBus : public ICanBus {
public:
  bool begin(uint32_t bitrate) override {
    return CAN.begin(bitrate);
  }
  bool send(const CanFrame& f) override {
    arduino::CanMsg m(f.id, f.dlc, f.data);
    return CAN.write(m) == 1;
  }
  bool available() override {
    return CAN.available() > 0;
  }
  bool read(CanFrame& out) override {
    if (CAN.available() == 0) return false;
    arduino::CanMsg m = CAN.read();
    out.id = m.id;
    out.dlc = m.data_length;
    for (uint8_t i = 0; i < m.data_length; i++) out.data[i] = m.data[i];
    return true;
  }
  void setFilter(uint16_t id, uint16_t mask) override {
    CAN.setFilterMask_Standard(mask);
    CAN.setFilterId_Standard(0, id);
  }
};

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

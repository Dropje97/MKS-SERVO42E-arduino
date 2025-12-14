#include <Arduino.h>
#include <Arduino_CAN.h>
#include "transport/ICanBus.h"
#include "MKSServoE.h"

class UnoR4CanBus : public ICanBus {
public:
  bool begin(uint32_t bitrate) override {
    return CAN.begin(bitrate);
  }
  bool send(const CanFrame& f) override {
    CanMsg m(CanStandardId(f.id), f.dlc, f.data);
    return CAN.write(m);
  }
  bool available() override {
    return CAN.available();
  }
  bool read(CanFrame& out) override {
    CanMsg m;
    if (!CAN.read(m)) return false;
    out.id = m.id;
    out.dlc = m.data_length;
    for (int i=0;i<m.data_length;i++) out.data[i]=m.data[i];
    return true;
  }
  void setFilter(uint16_t id, uint16_t mask) override {
    CAN.filter(CanStandardId(id), mask);
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
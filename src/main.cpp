#include <Arduino.h>
#include "esp_can.h"
#include "VirtualTimer.h"
#define ADDRESS 0x123

ESPCAN bus{};
VirtualTimerGroup timer_group{};

MakeSignedCANSignal(uint16_t, 0, 16, 1, 0) txsignal;
CANTXMessage<1> txmsg{bus, ADDRESS, 2, 10, timer_group, txsignal};

void tenms() {
  txsignal = 50;
  bus.Tick();
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  bus.Initialize(ICAN::BaudRate::kBaud1M);
  timer_group.AddTimer(10, tenms);
}

void loop() {
  // put your main code here, to run repeatedly:
  timer_group.Tick(millis());
}


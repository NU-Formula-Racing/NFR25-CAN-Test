#include <Arduino.h>
#include <SPI.h>

#include "esp_can.h"
#include "VirtualTimer.h"

#define ADDRESS 0x123
#define APPS1_CS 19
#define APPS2_CS 21
// #define MIN_APPS1
// #define MAX_APPS1
// #define MIN_APPS2
// #define MAX_APPS2
#define MAX_THROTTLE 100

void adc_init();
void read_adcs();
void update_throttle();
void print_all();
void tenms();

ESPCAN bus{};
VirtualTimerGroup timer_group{};

MakeSignedCANSignal(uint16_t, 0, 16, 1, 0) txsignal;
CANTXMessage<1> txmsg{bus, ADDRESS, 2, 10, timer_group, txsignal};

const uint16_t kTransmissionIDSetCurrent = 0x200;
MakeSignedCANSignal(int32_t, 0, 32, 0.001, 0) Set_Current;
// CANSignal<int32_t, 0, 32, CANTemplateConvertFloat(1000), CANTemplateConvertFloat(0), false> Set_Current{};
CANTXMessage<1> ECU_Set_Current{bus, kTransmissionIDSetCurrent, 8, 10, timer_group, Set_Current};

int16_t apps1_adc;
int16_t apps2_adc;

int16_t apps1_raw;
int16_t apps2_raw;

int16_t apps1_voltage;
int16_t apps2_voltage;

float apps1_scaled;
float apps2_scaled;

int32_t throttle_percent;

void adc_init() {
  SPI.begin();
  pinMode(APPS1_CS, OUTPUT);
  pinMode(APPS2_CS, OUTPUT);
  digitalWrite(APPS1_CS, HIGH);
  digitalWrite(APPS2_CS, HIGH);
}

void read_adcs() {
  SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE2));

  // read from APPS1 adc
  digitalWrite(APPS1_CS, LOW);
  apps1_adc = SPI.transfer16(0x0000);
  digitalWrite(APPS1_CS, HIGH);

  // read from APPS2 adc
  digitalWrite(APPS2_CS, LOW);
  apps2_adc = SPI.transfer16(0x0000);
  digitalWrite(APPS2_CS, HIGH);

  SPI.endTransaction();

  // select correct bits and write to global sensor variables
  apps1_raw = int16_t(apps1_adc << 2) >> 4;
  apps2_raw = int16_t(apps2_adc << 2) >> 4;
}

void update_throttle() {
  read_adcs();

  apps1_voltage = (apps1_raw*6.6)/4096;
  apps2_voltage = (apps2_raw*6.6)/4096;

  // float apps1_scaled = float(apps1_raw-MIN_APPS1)/(MAX_APPS1-MIN_APPS1)*100;
  // float apps2_scaled = float(apps2_raw-MIN_APPS2)/(MAX_APPS2-MIN_APPS2)*100;

  // if (apps1_scaled <= 0.0) {
  //   throttle_percent = 0;
  // } else if (apps1_scaled >= 100.0) {
  //   throttle_percent = MAX_THROTTLE;
  // } else {
  //   throttle_percent = ((apps1_scaled*MAX_THROTTLE)-apps1_scaled*MAX_THROTTLE)/(MAX_APPS1-MIN_APPS1);
  // }
  // Set_Current = throttle_percent;
}

void print_all() {
  // Serial.print("APPS1_raw:");
  Serial.println(apps1_adc);
  // Serial.print("APPS2_raw:");
  // Serial.println(apps2_raw);
  // Serial.print("APPS1_voltage:");
  // Serial.println(apps1_voltage);
  // Serial.print("APPS2_voltage:");
  // Serial.println(apps2_voltage);
  // Serial.println("APPS1_scaled: %f, APPS2_scaled: %f", apps1_scaled, apps2_scaled);
  // Serial.println("Throttle_percent: %f, Set_Current: %d", throttle_percent, Set_Current);
}

void tenms() {
  txsignal = 50;
  bus.Tick();
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  bus.Initialize(ICAN::BaudRate::kBaud1M);
  adc_init();
  timer_group.AddTimer(10, tenms);
}

void loop() {
  update_throttle();
  print_all();
  timer_group.Tick(millis());
}
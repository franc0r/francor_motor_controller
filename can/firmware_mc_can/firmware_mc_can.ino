#include "mcp2515_defs.h"
#include "mcp2515.h"
#include "defaults.h"
#include "global.h"
#include "CANInterface.h"

constexpr uint16_t  TX_ADDRESS = 0x100;
constexpr uint16_t  RX_ADDRESS = 0x101;

CAN::Interface can(CAN::SPEED_500k, TX_ADDRESS, RX_ADDRESS);

/*
 * Empfängt:
 *  PWM Signal int16_t
 *  Bremssignal
 *
 * Senden:
 *  Frequenz -> Drehzahl
 *
 */


void setup() {
  Serial.begin(115200);
  
  can.init();
}

void loop() {

  int16_t value = 0;
  
  while(true) {
    can.update();

    if(Serial.available()){
      Serial.print("Speed Cmd: ");
      Serial.print(can.getSpeedCmd());
      Serial.print(" Break Cmd: ");
      Serial.print(can.getStopCmd());
      Serial.println();
    }

    value ++;
    can.setSpeedHz(value);

    delay(100);
  }
}

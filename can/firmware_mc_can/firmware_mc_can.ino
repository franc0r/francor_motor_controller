#include "mcp2515_defs.h"
#include "mcp2515.h"
#include "defaults.h"
#include "global.h"
#include "CANInterface.h"

constexpr uint16_t  TX_ADDRESS_1 = 0x10;
constexpr uint16_t  TX_ADDRESS_2 = 0x20;

constexpr uint16_t  RX_ADDRESS_1 = 0x11;
constexpr uint16_t  RX_ADDRESS_2 = 0x21;

CAN::Interface can(CAN::SPEED_500k, TX_ADDRESS_1, RX_ADDRESS_1, TX_ADDRESS_2, RX_ADDRESS_2);

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
      Serial.print("Speed Cmd[0]: ");
      Serial.print(can._rx_data[0]._speed_cmd);
      Serial.print(" Break Cmd[0]: ");
      Serial.print(can._rx_data[0]._stop_cmd);
      Serial.println();
      Serial.print("Speed Cmd[1]: ");
      Serial.print(can._rx_data[1]._speed_cmd);
      Serial.print(" Break Cmd[1]: ");
      Serial.print(can._rx_data[1]._stop_cmd);
      Serial.println();
    }

    value ++;
    can._tx_data[0]._speed_hz = value;
    can._tx_data[1]._speed_hz = value*2;

    delay(100);
  }
}

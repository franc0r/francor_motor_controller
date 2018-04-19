#include "mcp2515_defs.h"
#include "mcp2515.h"
#include "defaults.h"
#include "global.h"
#include "CANInterface.h"


CAN::Interface can(CAN::SPEED_500k);

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
  tCAN rx_message;


  
  
  while(true) {
    if(can.receive(&rx_message)) {
      CAN::BLDCData rxBLDC(rx_message.data);
      Serial.print("Speed: ");
      Serial.print(rxBLDC._speed_cmd);
      Serial.print(" Break: ");
      Serial.print(rxBLDC._stop_cmd);
      Serial.println();
     
      
    }

    const CAN::BLDCData test(10, 0);
    for(uint8_t i = 0; i < 8; i++) {
      Serial.print(test._raw_data[i]);
      Serial.print(" ");
    }
    Serial.println();

    delay(1000);
  }
}

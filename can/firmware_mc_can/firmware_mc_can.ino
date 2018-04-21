#include "Arduino.h"

#include "mcp2515_defs.h"
#include "mcp2515.h"
#include "defaults.h"
#include "global.h"
#include "CANInterface.h"

#include <SoftPWM_timer.h>
#include <SoftPWM.h>

class Motorcontroller {
public:
  Motorcontroller(int pin_speed, int pin_rot, int pin_break, bool reverse = false)
  {
    _pin_speed = pin_speed;
    _pin_rot   = pin_rot  ;
    _pin_break = pin_break;

    _reverse = reverse ? -1 : 1;
  }
  ~Motorcontroller()
  { }

  void init()
  {
    //set output
    // ::pinMode(_pin_speed , OUTPUT);
    ::pinMode(_pin_rot   , OUTPUT);
    ::pinMode(_pin_break , OUTPUT);

    // ::digitalWrite(_pin_speed , LOW);
    ::digitalWrite(_pin_rot   , LOW);
    ::digitalWrite(_pin_break , LOW);

    SoftPWMSet(_pin_speed, 0);
    SoftPWMSetFadeTime(_pin_speed, 10, 10);
  }

  /**
   * @param speed : -255 .. 255;
   */
  void setSpeed(const int speed)
  {
    //set rotation
    if(speed * _reverse > 0)
    {
      ::digitalWrite(_pin_rot, true);
    }
    else
    {
      ::digitalWrite(_pin_rot, false);
    }

    //set speed
    uint8_t sp = (uint8_t)abs(speed);

    SoftPWMSetFadeTime(_pin_speed, 10, 10);
    //::analogWrite(_pin_speed, sp);
  }

  void brk(const bool brk)
  {
    ::digitalWrite(_pin_break, brk);
  }


private:
  int _pin_speed;
  int _pin_rot  ;
  int _pin_break;

  int _reverse;
};


constexpr uint16_t  TX_ADDRESS_1 = 0x10;
constexpr uint16_t  TX_ADDRESS_2 = 0x20;

constexpr uint16_t  RX_ADDRESS_1 = 0x11; //TODO Fix address before flashing
constexpr uint16_t  RX_ADDRESS_2 = 0x12; //TODO Fix address before flashing

CAN::Interface can(CAN::SPEED_500k, TX_ADDRESS_1, RX_ADDRESS_1, TX_ADDRESS_2, RX_ADDRESS_2);

Motorcontroller g_mc_0(A0, A1, A2, false);
Motorcontroller g_mc_1(A4, A5, A6, true);

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
  // Serial.begin(115200);
  
  SoftPWMBegin();
  g_mc_0.init();
  g_mc_1.init();
  // SoftPWMSet(A0, 0);

  // SoftPWMSetFadeTime(13, 10, 10);

  // //in loop PWM 
  // SoftPWMSet(A0, 128);

  can.init();
}

void loop() {

  can.update();

  // if(Serial.available()){
  //   Serial.print("Speed Cmd[0]: ");
  //   Serial.print(can._rx_data[0]._speed_cmd);
  //   Serial.print(" Break Cmd[0]: ");
  //   Serial.print(can._rx_data[0]._stop_cmd);
  //   Serial.println();
  //   Serial.print("Speed Cmd[1]: ");
  //   Serial.print(can._rx_data[1]._speed_cmd);
  //   Serial.print(" Break Cmd[1]: ");
  //   Serial.print(can._rx_data[1]._stop_cmd);
  //   Serial.println();
  // }

  g_mc_0.setSpeed(can._rx_data[0]._speed_cmd); //set speed -255 .. 255
  g_mc_1.setSpeed(can._rx_data[1]._speed_cmd); //set speed -255 .. 255
  g_mc_0.brk(can._rx_data[0]._stop_cmd ? true : false);
  g_mc_1.brk(can._rx_data[1]._stop_cmd ? true : false);

  SoftPWMSet(A0, 25);
  delay(10);
}

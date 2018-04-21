#include "Arduino.h"

#include "mcp2515_defs.h"
#include "mcp2515.h"
#include "defaults.h"
#include "global.h"
#include "MotorControllerMsg.h"

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

    //SoftPWMSetFadeTime(_pin_speed, 10, 10);
    //::analogWrite(_pin_speed, sp);
    SoftPWMSet(_pin_speed, sp);
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

constexpr uint8_t CAN_SPEED_100k  = 9;
constexpr uint8_t CAN_SPEED_125k  = 7;
constexpr uint8_t CAN_SPEED_250k  = 3;
constexpr uint8_t CAN_SPEED_500k  = 1;
constexpr uint8_t CAN_SPEED_1M    = 0;


Motorcontroller g_mc_0(A0, A1, A2, false);
Motorcontroller g_mc_1(A4, A5, A6, true);

constexpr uint16_t CAN_ID = 0x10;
francor::MotorControlMsg  can_control_msgs[]  = {francor::MotorControlMsg(CAN_ID + 1), francor::MotorControlMsg(CAN_ID + 2)};

bool receiveCanMsgs(void) {
  tCAN msg;

  if(mcp2515_check_message()) {
    if(mcp2515_get_message(&msg)) {
      for(uint8_t idx = 0; idx < (sizeof(can_control_msgs) / sizeof(francor::MotorControlMsg)); idx++) {
        if(can_control_msgs[idx]._can_id == msg.id) {
          // copy data
          memcpy(can_control_msgs[idx]._raw_data, msg.data, 8);
          return true;
        }
      }
    }
  }

  return false;
}


void setup() {
  // Serial.begin(115200);
  mcp2515_init(CAN_SPEED_500k);
  
  SoftPWMBegin();
  g_mc_0.init();
  g_mc_1.init();
  
  // SoftPWMSet(A0, 0);

  // SoftPWMSetFadeTime(13, 10, 10);

  // //in loop PWM 
  // SoftPWMSet(A0, 128);

}

void loop() {

  receiveCanMsgs();

  g_mc_0.setSpeed(can_control_msgs[0]._power); //set speed -255 .. 255
  g_mc_1.setSpeed(can_control_msgs[1]._power); //set speed -255 .. 255
  g_mc_0.brk(can_control_msgs[0]._stop ? true : false);
  g_mc_1.brk(can_control_msgs[1]._stop ? true : false);

  tCAN txMsg;
  txMsg.id = CAN_ID;
  txMsg.header.rtr = 0;
  txMsg.header.length = 8;
  txMsg.data[0] = (uint8_t)(can_control_msgs[0]._power);
  txMsg.data[1] = (uint8_t)(can_control_msgs[0]._power>>8);
  txMsg.data[2] = (uint8_t)(can_control_msgs[0]._stop);
  txMsg.data[3] = (uint8_t)(can_control_msgs[1]._power);
  txMsg.data[4] = (uint8_t)(can_control_msgs[1]._power>>8);
  txMsg.data[5] = (uint8_t)(can_control_msgs[1]._stop);
  txMsg.data[6] = 0;
  txMsg.data[7] = 0;

  mcp2515_bit_modify(CANCTRL, (1<<REQOP2)|(1<<REQOP1)|(1<<REQOP0), 0);
  mcp2515_send_message(&txMsg);

  delay(10);

}

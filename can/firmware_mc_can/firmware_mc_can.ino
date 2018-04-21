#include "Arduino.h"

#include "mcp2515_defs.h"
#include "mcp2515.h"
#include "defaults.h"
#include "global.h"
#include "MotorcontrollerMsg.h"
#include "Motorcontroller.h"


constexpr uint8_t CAN_SPEED_100k  = 9;
constexpr uint8_t CAN_SPEED_125k  = 7;
constexpr uint8_t CAN_SPEED_250k  = 3;
constexpr uint8_t CAN_SPEED_500k  = 1;
constexpr uint8_t CAN_SPEED_1M    = 0;


Motorcontroller g_mc_0(A0, A1, A2, false);
Motorcontroller g_mc_1(A4, A5, A6, true);

constexpr uint16_t CAN_ID = 0x10;
francor::MotorcontrolMsg  can_control_msgs[]  = {francor::MotorcontrolMsg(CAN_ID + 1), francor::MotorcontrolMsg(CAN_ID + 2)};

bool receiveCanMsgs(void) {
  tCAN msg;

  if(mcp2515_check_message()) {
    if(mcp2515_get_message(&msg)) {
      for(uint8_t idx = 0; idx < (sizeof(can_control_msgs) / sizeof(francor::MotorcontrolMsg)); idx++) {
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
  static uint8_t tick_cnt_msg_tx = 0;

  receiveCanMsgs();

  g_mc_0.setSpeed(can_control_msgs[0]._power); //set speed -255 .. 255
  g_mc_1.setSpeed(can_control_msgs[1]._power); //set speed -255 .. 255
  g_mc_0.brk(can_control_msgs[0]._stop ? true : false);
  g_mc_1.brk(can_control_msgs[1]._stop ? true : false);

  if(tick_cnt_msg_tx > 100){
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
    tick_cnt_msg_tx = 0;
  }
  

  tick_cnt_msg_tx++;

  delay(1);
}

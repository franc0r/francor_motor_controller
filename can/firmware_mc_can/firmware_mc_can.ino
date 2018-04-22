#include "Arduino.h"

#include "mcp2515_defs.h"
#include "mcp2515.h"
#include "defaults.h"
#include "global.h"
#include "MotorcontrollerMsg.h"
#include "Motorcontroller.h"

/*
 * CAN Speed Definitions
 */
constexpr uint8_t CAN_SPEED_100k  = 9;
constexpr uint8_t CAN_SPEED_125k  = 7;
constexpr uint8_t CAN_SPEED_250k  = 3;
constexpr uint8_t CAN_SPEED_500k  = 1;
constexpr uint8_t CAN_SPEED_1M    = 0;

/*
 * Definitions
 */
constexpr uint16_t CAN_ID          = 0x10; //!< Base CAN-ID of the motor controller (Base-ID: Status | CAN_ID + 1: Left Cmd | CAN_ID + 2: Right Cmd) 
constexpr uint16_t CAN_TX_TIME_MS  = 250;  //!< TX time delay of status update send via Base-ID (CAN_ID)

/*
 * Global Variables
 */
francor::MotorcontrolMsg  can_control_msgs[]  = {francor::MotorcontrolMsg(CAN_ID + 1), francor::MotorcontrolMsg(CAN_ID + 2)};

Motorcontroller g_mc_0(A0, A1, A2, false);
//Motorcontroller g_mc_1(A4, A5, A6, true);
Motorcontroller g_mc_1(A4, A5, 5, true); //Motorcontroller break port changed


void receiveCanMsgs(void) {
  tCAN msg;

  // Check if new message is available
  if(mcp2515_check_message()) {
    // Get new message
    if(mcp2515_get_message(&msg)) {
      for(uint8_t idx = 0; idx < (sizeof(can_control_msgs) / sizeof(francor::MotorcontrolMsg)); idx++) {
        if(can_control_msgs[idx]._can_id == msg.id) {
          // copy data
          memcpy(can_control_msgs[idx]._raw_data, msg.data, 8);
        }
      }
    }
  }
}


void setup() {
  // Serial.begin(115200);

  // Init CAN network with 500 kBit
  mcp2515_init(CAN_SPEED_500k);

  // Start software PWM
  SoftPWMBegin();

  // Init motorcontrollers
  g_mc_0.init();
  g_mc_1.init();
  
  // SoftPWMSet(A0, 0);

  // SoftPWMSetFadeTime(13, 10, 10);

  // //in loop PWM 
  // SoftPWMSet(A0, 128);

}

void loop() {
  static uint16_t tick_cnt_msg_tx = 0;

  // receive CAN messages
  receiveCanMsgs();

  // update speed signals (PWM)
  g_mc_0.setSpeed(can_control_msgs[0]._power); //set speed -255 .. 255
  g_mc_1.setSpeed(can_control_msgs[1]._power); //set speed -255 .. 255

  // update brake signals
  g_mc_0.brk(can_control_msgs[0]._stop ? true : false);
  g_mc_1.brk(can_control_msgs[1]._stop ? true : false);

  if(tick_cnt_msg_tx > CAN_TX_TIME_MS){
    // Check for free send buffers
    if(mcp2515_check_free_buffer()) {
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
  }
  
  tick_cnt_msg_tx++;

  delay(1);
}

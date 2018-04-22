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
constexpr uint16_t SYS_TICK_DLY_MS = 1;    //!< Systemtick delay time in ms

constexpr uint16_t CAN_BASE_ID        = 0x10; //!< Base CAN-ID of the motor controller (Base-ID: Status | CAN_ID + 1: Left Cmd | CAN_ID + 2: Right Cmd) 
constexpr uint16_t CAN_TX_TIME_MS     = 250;  //!< TX time delay of status update send via Base-ID (CAN_ID)
constexpr uint16_t CAN_RX_TIMEOUT_MS  = 250;  //!< Timeout threshold MCs go to break

/*
 * Global Variables
 */
francor::MotorcontrolMsg  can_control_msgs[]  = {francor::MotorcontrolMsg(CAN_BASE_ID + 1), 
                                                 francor::MotorcontrolMsg(CAN_BASE_ID + 2)};

Motorcontroller g_mc_0(A0, A1, A2, false);
//Motorcontroller g_mc_1(A4, A5, A6, true);
Motorcontroller g_mc_1(A4, A5, 5, true); //Motorcontroller break port changed

void sendStatusOnCan(void) {
  static uint16_t sys_tick_cnt = 0;

  // increase system tick counter
  sys_tick_cnt++;

  // check if sys tick cnt is greater than
  // delay threshold
  if(sys_tick_cnt >= (CAN_TX_TIME_MS / SYS_TICK_DLY_MS)) {
      tCAN txMsg;
      txMsg.id = CAN_BASE_ID;
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
      sys_tick_cnt = 0;
  }
}

bool receiveCanMsgs(void) {
  static uint16_t sys_tick_cnt[2] = {0, 0};
  bool new_data = false;

  // increase system tick counter
  sys_tick_cnt[0]++;
  sys_tick_cnt[1]++;

  // Check if new message is available
  if(mcp2515_check_message()) {
    // message buffer
    tCAN msg;

    // read message from internal buffer
    if(mcp2515_get_message(&msg)) {
      for(uint8_t idx = 0; idx < (sizeof(can_control_msgs) / sizeof(francor::MotorcontrolMsg)); idx++) {

        // check if ID is correct
        if(can_control_msgs[idx]._can_id == msg.id) {

          // copy data
          memcpy(can_control_msgs[idx]._raw_data, msg.data, 8);

          // reset related system tick counter to prevent timeout detection
          sys_tick_cnt[idx] = 0;

          new_data = true;
        }
      }
    }
  }

  // check if timeout occured
  for(uint8_t idx = 0; idx < (sizeof(can_control_msgs) / sizeof(francor::MotorcontrolMsg)); idx++) {
    if(sys_tick_cnt[idx] > (CAN_RX_TIMEOUT_MS / SYS_TICK_DLY_MS)) {
      // reset timeout counter
      sys_tick_cnt[idx] = 0;

      // timeout reset drives
      can_control_msgs[idx]._power = 0;
      can_control_msgs[idx]._stop = 1;

      new_data = true;
    }
  }

  return new_data;
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
  // receive CAN messages
  const bool new_can_msg = receiveCanMsgs();

  // Check if new CAN message is received
  if(new_can_msg) {
    // update speed signals (PWM)
    g_mc_0.setSpeed(can_control_msgs[0]._power); //set speed -255 .. 255
    g_mc_1.setSpeed(can_control_msgs[1]._power); //set speed -255 .. 255

    // update brake signals
    g_mc_0.brk(can_control_msgs[0]._stop ? true : false);
    g_mc_1.brk(can_control_msgs[1]._stop ? true : false);
  }

  // send status on CAN
  sendStatusOnCan();

  delay(SYS_TICK_DLY_MS);
}

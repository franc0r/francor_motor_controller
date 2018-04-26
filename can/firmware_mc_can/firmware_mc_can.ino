#include <PinChangeInterrupt.h>
#include <PinChangeInterruptBoards.h>
#include <PinChangeInterruptPins.h>
#include <PinChangeInterruptSettings.h>

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
constexpr uint16_t CAN_BASE_ID               = 0x30; //!< Base CAN-ID of the motor controller (Base-ID: Status | CAN_ID + 1: Left Cmd | CAN_ID + 2: Right Cmd) 
constexpr uint16_t CAN_TX_STATUS_TIME_MS     = 50;   //!< TX time delay of status update send via Base-ID (CAN_ID)
constexpr uint16_t CAN_RX_TIMEOUT_MS         = 1000; //!< Timeout threshold MCs go to brake
constexpr uint16_t SPEED_UPDATE_TIME_MS      = 50;   //!< Time delay before update of wheel speed is performed

/*
 * Global Variables
 */
uint8_t timeout_detd[]  = {false, false};

francor::MotorcontrolMsg  g_can_control_msgs[]  = {francor::MotorcontrolMsg(CAN_BASE_ID + 1), 
                                                 francor::MotorcontrolMsg(CAN_BASE_ID + 2)};
 
francor::MotorStatusMsg   g_can_status_msgs[]   = {francor::MotorStatusMsg(CAN_BASE_ID + 5 + 1),
                                                francor::MotorStatusMsg(CAN_BASE_ID + 5 + 2)};

francor::Motorcontroller  g_motorcontrollers[]   = {francor::Motorcontroller(A0, A1, A2, A3, false),
                                                  francor::Motorcontroller(A4, A5, 5, 2, true)};

constexpr uint8_t NUM_MOTORCONTROLLER         = (sizeof(g_motorcontrollers)/sizeof(francor::Motorcontroller));

//Motorcontroller g_mc_0(A0, A1, A2, A3, false);
//Motorcontroller g_mc_1(A4, A5, A6, A7, true);
//Motorcontroller g_mc_1(A4, A5, 5, true); //Motorcontroller brake port changed

void tickMotorcontroller1(void) {
  g_motorcontrollers[0].tick();
}

void tickMotorcontroller2(void) {
  g_motorcontrollers[1].tick(); 
}

void sendStatusOnCan(void) {
  static uint16_t sys_tick_cnt = 0;

  // increase system tick counter
  sys_tick_cnt++;

  // check if sys tick cnt is greater than
  // delay threshold
  if(sys_tick_cnt >= (CAN_TX_STATUS_TIME_MS)) {
    // update status messages
    for(uint8_t idx = 0; idx < NUM_MOTORCONTROLLER; idx++) {
      if(timeout_detd[idx]) {
        g_can_status_msgs[idx]._state = francor::MotorStatusMsg::State::MSM_CMD_TIMEOUT;
      }
      else {
        g_can_status_msgs[idx]._state = francor::MotorStatusMsg::State::MSM_OPERATIONAL;
      }
      
      g_can_status_msgs[idx]._speed_ticks = g_motorcontrollers[idx].getSpeedTicks();
      g_can_status_msgs[idx]._dtime_ticks = g_motorcontrollers[idx].getDTimeSpeedTicks();

      tCAN txMsg;
      txMsg.id = g_can_status_msgs[idx]._can_id;
      txMsg.header.ide = 0;
      txMsg.header.rtr = 0;
      txMsg.header.length = 8;

      memcpy(txMsg.data, g_can_status_msgs[idx]._raw_data, 8);

      mcp2515_bit_modify(CANCTRL, (1<<REQOP2)|(1<<REQOP1)|(1<<REQOP0), 0);
      mcp2515_send_message(&txMsg);
      sys_tick_cnt = 0;
    }
  }
}

bool receiveCanMsgs(void) {
  static uint16_t sys_tick_cnt[2] = {0, 0};
  bool new_data = false;

  // increase system tick counter
  for(uint8_t idx = 0; idx < NUM_MOTORCONTROLLER; idx++) {
    sys_tick_cnt[idx]++;
  }

  // Check if new message is available
  if(mcp2515_check_message()) {
    // message buffer
    tCAN msg;

    // read message from internal buffer
    if(mcp2515_get_message(&msg)) {
      for(uint8_t idx = 0; idx < NUM_MOTORCONTROLLER; idx++) {

        // check if ID is correct
        if(g_can_control_msgs[idx]._can_id == msg.id) {

          // copy data
          memcpy(g_can_control_msgs[idx]._raw_data, msg.data, 8);

          // reset related system tick counter to prevent timeout detection
          sys_tick_cnt[idx] = 0;

          // no timeout detected
          timeout_detd[idx] = 0;

          new_data = true;
        }
      }
    }
  }

  // check if timeout occured
  for(uint8_t idx = 0; idx < NUM_MOTORCONTROLLER; idx++) {
    if(sys_tick_cnt[idx] > (CAN_RX_TIMEOUT_MS)) {
      // reset timeout counter
      sys_tick_cnt[idx] = 0;

      // timeout reset drives
      g_can_control_msgs[idx]._power = 0;
      g_can_control_msgs[idx]._brake = 1;

      // timeout detected
      timeout_detd[idx] = 1;

      new_data = true;
    }
  }

  return new_data;
}

/* Speed calculation not used */
void updateWheelSpeed(void) {
  static uint16_t sys_tick_cnt = 0;

  // increase system tick counter
  sys_tick_cnt++;

  // check if delay time has elapsed
  if(sys_tick_cnt > SPEED_UPDATE_TIME_MS) {
    for(uint8_t idx = 0; idx < NUM_MOTORCONTROLLER; idx++) {
      g_motorcontrollers[idx].calculateTicksPerSec();
    }

    // reset tick counter
    sys_tick_cnt = 0;
  }  
}

void setup() {
  // Serial.begin(115200);

  // Init CAN network with 500 kBit
  mcp2515_init(CAN_SPEED_500k);

  // Start software PWM
  SoftPWMBegin();

  // Init motorcontrollers
  for(uint8_t idx = 0; idx < NUM_MOTORCONTROLLER; idx++) {
    g_motorcontrollers[idx].init();
  }
  
  // SoftPWMSet(A0, 0);

  // SoftPWMSetFadeTime(13, 10, 10);

  // //in loop PWM 
  // SoftPWMSet(A0, 128);

  //attachPinChangeInterrupt(digitalPinToPinChangeInterrupt(g_motorcontrollers[0].getPinSignal()), 
  //                                                        tickMotorcontroller1, CHANGE);
                                                          
  //attachPinChangeInterrupt(digitalPinToPinChangeInterrupt(g_motorcontrollers[1].getPinSignal()), 
  //                                                        tickMotorcontroller2, CHANGE);

}

void loop() {
  static unsigned long tmp_sys_tick = millis();

  // check if 1 ms has elapsed
  if((millis() - tmp_sys_tick) != 0){
    tmp_sys_tick = millis();

    // receive CAN messages
    const bool new_can_msg = receiveCanMsgs();

    // Check if new CAN message is received
    if(new_can_msg) {
      // update motor controllers
      for(uint8_t idx = 0; idx < NUM_MOTORCONTROLLER; idx++) {
        g_motorcontrollers[idx].setSpeed(g_can_control_msgs[idx]._power); //set speed -255 .. 255
        g_motorcontrollers[idx].setBrake(g_can_control_msgs[idx]._brake ? true : false);
      }
    }

    // send status on CAN
    //sendStatusOnCan();

    /* No TX of status */
    // update wheel speeds
    // updateWheelSpeed();
  }
}

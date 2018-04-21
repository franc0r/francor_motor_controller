/*
 * CANInterface.h
 *
 *  Created on: 19.04.2018
 *      Author: feesmrt
 *
 * BSD 3-Clause License
 * Copyright (c) 2018, FRANC0R - Franconian Open Robotics
 * All rights reserved.
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * * Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 * * Redistributions in binary form must reproduce the above copyright notice,
 *  this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 * * Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from
 *   this software without specific prior written permission.
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#ifndef __CAN_INTERFACE_H_
#define __CAN_INTERFACE_H_

namespace CAN
{
    constexpr uint8_t SPEED_100k  = 9;
    constexpr uint8_t SPEED_125k  = 7;
    constexpr uint8_t SPEED_250k  = 3;
    constexpr uint8_t SPEED_500k  = 1;
    constexpr uint8_t SPEED_1M    = 0;

/*
 * @brief: BLDCData which is exchanged between board and pc
 */
class BLDCData
{
public:
  union {
    struct {
      int16_t _speed_cmd; //!< Speed command from PC to Board
      uint8_t _stop_cmd;  //!< Stop command from PC to Board
      uint8_t _n1[5];     //!< Placeholder
    };
    
     struct {
      uint8_t _n2[6];     //!< Placeholder
      int16_t _speed_hz;  //!< Current speed from Board to PC
     };
     
     uint8_t _raw_data[8]; //!< Raw data buffer
   };

  BLDCData()     : _raw_data{0, 0, 0, 0, 0, 0, 0, 0}  {}
  
  BLDCData(const int16_t& speed_cmd, const uint8_t& stop_cmd) 
    : _speed_cmd(speed_cmd), _stop_cmd(stop_cmd), _n1{0, 0, 0, 0, 0} {}

  BLDCData(const int16_t& speed_hz) : _speed_hz(speed_hz), _n2{0, 0, 0, 0, 0, 0}  {}
    
  BLDCData(const uint8_t* data) 
    : _raw_data{data[0], data[1], data[2], data[3], data[4], data[5], data[6], data[7]} {}
  
};

class Interface
{
public:
  /**
   * @brief Constructor of interface class
   */
  Interface(const uint8_t can_speed,
            const uint16_t tx_address1,
            const uint16_t rx_address1,
            const uint16_t tx_address2,
            const uint16_t rx_address2) :
  _tx_address({tx_address1, tx_address2}),
  _rx_address({rx_address1, rx_address2}),
  _can_speed(can_speed),
  _tx_id(0)
  {
    
  }

  /**
   * @brief Initializes communication with setup speed
   */
  uint8_t init(void) const {
    return mcp2515_init(_can_speed);
  }

  /*
   * @brief Receives and transmits CAN messages
   */
  bool update(void) {
    receive();
    
    // transmit data
    //send();
  }

  BLDCData      _rx_data[2];  //!< Buffer of received data
  BLDCData      _tx_data[2];  //!< Buffer of transmit data

private:

  bool send() {
    _msg_buff.id = _tx_address[_tx_id];
    _msg_buff.header.rtr = 0;
    _msg_buff.header.length = 8;
    memcpy(_msg_buff.data, _tx_data[_tx_id]._raw_data, 8);
    
    mcp2515_bit_modify(CANCTRL, (1<<REQOP2)|(1<<REQOP1)|(1<<REQOP0), 0);

    if(mcp2515_send_message(&_msg_buff)) {
      return true;
    }
    else {
      return false;
    }

    if(_tx_id == 0) {
      _tx_id = 1;
    }
    else {
      _tx_id = 0;
    }
    
    
  }

  bool receive() {
    if(mcp2515_check_message()) {
        if(mcp2515_get_message(&_msg_buff)) {
          for(uint8_t idx = 0; idx < 2; idx++) {
            if(_msg_buff.id == _rx_address[idx]) {
              memcpy(_rx_data[idx]._raw_data, _msg_buff.data, 8);
              return true;
            }
          }
          
        }
      }
    return false;
  }

private:
  const uint8_t   _can_speed;  //!< CAN Speed
  
  const uint16_t  _tx_address[2]; //!< CAN address of transmited data 1
  const uint16_t  _rx_address[2]; //!< CAN address of transmited data 2
  
  uint8_t         _tx_id;         //!< Changes ID to send every message

  tCAN            _msg_buff; //!< CAN message buffer
};
};



#endif /*__CAN_INTERFACE_H_*/

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

class BLDCData
{
public:
  union {
    struct {
      int16_t _speed_cmd;
      uint8_t _stop_cmd;
      uint8_t _nui[5];
    };

    uint8_t _raw_data[8];
  };

  BLDCData()     : _raw_data{0, 0, 0, 0, 0, 0, 0, 0}  {}
  
  BLDCData(const int16_t& speed_cmd, const uint8_t& stop_cmd) 
    : _speed_cmd(speed_cmd), _stop_cmd(stop_cmd), _nui{0, 0, 0, 0, 0} {}
    
  BLDCData(const uint8_t* data) 
    : _raw_data{data[0], data[1], data[2], data[3], data[4], data[5], data[6], data[7]} {}
  
};

class Interface
{
public:
  Interface(const uint8_t can_speed) :
  _can_speed(can_speed)
  {
    
  }

  uint8_t init(void) const {
    return mcp2515_init(_can_speed);
  }

  bool send(const tCAN& msg) const {
    tCAN message = msg;
    
    mcp2515_bit_modify(CANCTRL, (1<<REQOP2)|(1<<REQOP1)|(1<<REQOP0), 0);

    if(mcp2515_send_message(&message)) {
      return true;
    }
    else {
      return false;
    }
  }

  bool receive(tCAN* msg) const {
    if(mcp2515_check_message()) {
      if(mcp2515_get_message(msg)) {
        return true;
      }
    }

    return false;
  }

private:
  const uint8_t _can_speed;
};
};



#endif /*__CAN_INTERFACE_H_*/

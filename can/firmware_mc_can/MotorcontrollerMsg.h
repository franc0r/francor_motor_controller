/*
 * MotorcontrollerMsg.h
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

#ifndef MOTORCONTROLLER_MSG_H_
#define MOTORCONTROLLER_MSG_H_

namespace francor
{
    /**
     * @brief Motor control messages
     * 
     * This struct defines the CAN message to control the robot
     * motor controller
     */
    struct MotorcontrolMsg {
        MotorcontrolMsg() 
          : _can_id(0), _raw_data{0, 0, 0, 0, 0, 0, 0, 0} {}
        MotorcontrolMsg(const uint16_t& can_id) 
          : _can_id(can_id), _power(0), _brake(1) {}
        MotorcontrolMsg(const int16_t& power, const bool& brake)
          : _can_id(0), _power(power), _brake(brake)  {}
      
        union {
            uint8_t     _raw_data[8];   //!< RAW Data of the CAN message

            struct {
                int16_t     _power;      //!< Max: 100 % = 10000, Min: -100% = -10000; Scaling: 1 = 0.01 %
                uint8_t     _brake  : 1; //!< Boolean flag for brake command
            }__attribute__((packed));
        };

        const uint16_t _can_id;  //!< CAN ID of the message
    };

    /**
     * @brief Motor status
     * 
     * This struct defines the CAN message which is send for every 
     * motor 
     * 
     */
    struct MotorStatusMsg {
      MotorStatusMsg()
        : _can_id(0), _raw_data{0, 0, 0, 0, 0, 0, 0, 0} {}
      MotorStatusMsg(const uint16_t& can_id)
        : _can_id(can_id), _raw_data{0, 0, 0, 0, 0, 0, 0, 0} {}

        // TO DO: Move enum declarations to Motorcontroller.h
        enum State {
            MSM_ERROR           = 0,  //!< General error detected
            MSM_OPERATIONAL     = 1,  //!< Operational
            MSM_CMD_TIMEOUT     = 2,  //!< Timeout error (no commands received)
        };

        union {
            uint8_t _raw_data[8]; //!< Raw data of buffer

            struct {
                State       _state : 4;     //!< Current state of actuator
                uint16_t    _speed_ticks;   //!< Speed of actuator (ticks/sec)
                uint16_t    _dtime_ticks;   //!< Delta time of ticks 

            }__attribute__((packed));
        };
        

        const uint16_t _can_id;  //!< CAN ID of the message
    };
};

#endif /* MOTORCONTROLLER_MSG_H_ */

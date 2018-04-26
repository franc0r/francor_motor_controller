/*
 * Motorcontroller.h
 *
 *  Created on: 19.04.2018
 *      Author: m4ffle
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

#ifndef __MOTORCONTROLLER_H
#define __MOTORCONTROLLER_H

#include <SoftPWM_timer.h>
#include <SoftPWM.h>

#include "MotorcontrollerMsg.h"

namespace francor 
{

class Motorcontroller {
public:
  Motorcontroller(uint8_t pin_speed, uint8_t pin_rot, 
                  uint8_t pin_brake, uint8_t pin_signal, 
                  bool reverse = false) :
    _pin_speed(pin_speed),
    _pin_rot(pin_rot),
    _pin_brake(pin_brake),
    _pin_signal(pin_signal),
    _reverse(reverse ? -1 : 1)
    //_old_tick_timestamp(0),
    //_ticks_per_sec(0)
  {

  }
  ~Motorcontroller()
  { }

  void init() const
  {
    //set output
    // ::pinMode(_pin_speed , OUTPUT);
    ::pinMode(_pin_rot   , OUTPUT);
    ::pinMode(_pin_brake , OUTPUT);

    // ::digitalWrite(_pin_speed , LOW);
    ::digitalWrite(_pin_rot   , LOW);
    ::digitalWrite(_pin_brake , LOW);

    SoftPWMSet(_pin_speed, 0);
    SoftPWMSetFadeTime(_pin_speed, 10, 10);
  }

  /**
   * @param speed : -255 .. 255;
   */
  void setSpeed(const int speed) const
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

  void setBrake(const bool brk) const
  {
    ::digitalWrite(_pin_brake, brk);
  }

  void tick(void) {
    _ticks++;
  }

  void calculateTicksPerSec(void) {   
    const uint32_t timestamp = millis();

    // 
    // 100 ticks t -> 100 ticks / 50 ms -> 2 ticks / ms ->

    _speed_ticks = (uint16_t)(_ticks);
    _delta_time_ticks   = (uint16_t)(timestamp - _old_tick_timestamp);

    _ticks = 0;
    _old_tick_timestamp = timestamp;
  }

  const uint8_t getPinSignal() const {return _pin_signal;}
  const uint16_t getSpeedTicks(void) const {return _speed_ticks;}
  const uint16_t getDTimeSpeedTicks(void) const {return _delta_time_ticks;}

private:
  volatile uint32_t _ticks;
  uint32_t  _old_tick_timestamp;

  uint16_t  _delta_time_ticks;
  uint16_t  _speed_ticks;

  const uint8_t _pin_speed;
  const uint8_t _pin_rot  ;
  const uint8_t _pin_brake;
  const uint8_t _pin_signal;

  const int8_t   _reverse;
};

};

#endif /*__MOTORCONTROLLER_H */


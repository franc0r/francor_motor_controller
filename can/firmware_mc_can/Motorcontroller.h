/*
 * MotorcontrollerMsg.h
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


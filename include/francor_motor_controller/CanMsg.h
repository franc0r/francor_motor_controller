/*
 *  Created on: 22.04.2018
 *      Author: Christian Merkl
 *      E-Mail: knueppl@gmx.de
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
#ifndef ___FRANCOR_CAN_MSG_H___
#define ___FRANCOR_CAN_MSG_H___

#include <linux/can.h>

#include <ostream>
#include <string>
#include <cstring>
#include <cstdint>
#include <array>

namespace francor {

namespace motor_controller {

class CanMsg
{
public:
  CanMsg(void) = default;
  CanMsg(const can_frame& frame)
    : _id(frame.can_id),
      _dlc(frame.can_dlc)
  {
    std::memcpy(_data.data(), frame.data, sizeof(frame.data));
  }
  CanMsg(const std::uint32_t id, const std::uint8_t dlc, const char* data)
    : _id(id),
      _dlc(dlc)
  {
    std::memcpy(_data.data(), data, dlc);
  }
  CanMsg(const CanMsg&) = default;
  CanMsg(CanMsg&&) = default;
  ~CanMsg(void) = default;

  can_frame toCanFrame(void) const noexcept
  {
    can_frame frame;

    frame.can_id = _id;
    frame.can_dlc = _dlc;
    std::memcpy(frame.data, _data.data(), sizeof(frame.data));

    return frame;
  }

  CanMsg& operator= (const CanMsg&) = default;
  CanMsg& operator= (CanMsg&&) = default;

  std::string toString(void) const
  {
    std::ostringstream out;

    out << std::setfill('0') << std::setw(3) << std::hex << _id << ":" << static_cast<unsigned int>(_dlc) << ":";

    for (std::size_t digit = 0; digit < _dlc; ++digit)
      out << std::setfill('0') << std::setw(2) << std::hex << static_cast<unsigned int>(_data[digit]);

    return out.str();
  }

private:
  std::uint32_t _id;
  std::uint8_t _dlc;
  std::array<std::uint8_t, 8> _data;
};

} // end namespace motor_controller

} // end namespace francor


inline std::ostream& operator <<(std::ostream& os, const francor::motor_controller::CanMsg& msg)
{
  os << msg.toString();

  return os;
}

#endif

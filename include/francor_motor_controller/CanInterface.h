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
#ifndef ___FRANCOR_CAN_INTERFACE_H___
#define ___FRANCOR_CAN_INTERFACE_H___
#include "francor_motor_controller/CanMsg.h"

#include <linux/can.h>
#include <sys/ioctl.h>
#include <net/if.h>

#include <thread>
#include <atomic>
#include <functional>

namespace francor {

namespace motor_controller {

class CanInterface
{
public:
  CanInterface(void) = default;
  ~CanInterface(void);

  bool initialize(const std::string device, std::function<void(const francor::motor_controller::CanMsg&)>& callback);

  void start(void);
  void stop(void);
  inline bool isInitialized(void) const noexcept { return _socket < 0; }
  void send(const CanMsg& msg);

private:
  void receiveCan(void);

  std::function<void(const francor::motor_controller::CanMsg&)> _callback_received_msg;
  std::thread _thread_receive;
  std::atomic<bool> _thread_receive_run{false};

  struct sockaddr_can _can_address;
  struct ifreq _ifr;
  int _socket = -1;
};

} // end namespace motor_controller

} // end namespace francor


#endif

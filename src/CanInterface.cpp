#include <ros/ros.h>

#include "francor_motor_controller/CanInterface.h"

#include <cstring>


namespace francor {

namespace motor_controller {

CanInterface::~CanInterface(void)
{
  this->stop();
}

bool CanInterface::initialize(const std::string device, std::function<void(const CanMsg&)> callback)
{
  if (this->isInitialized())
  {
    ROS_ERROR("CanInterface: can't interface is already intialized. --> return.");
    return false;
  }

  // Create socket and bind it.
  _socket = ::socket(PF_CAN, SOCK_RAW, CAN_RAW);

  if (_socket < 0)
  {
    ROS_ERROR("CanInterface: can't create a socket. --> return.");
    return false;
  }

  std::strcpy(_ifr.ifr_name, device.c_str());
  ::ioctl(_socket, SIOCGIFINDEX, &_ifr);

  _can_address.can_family = AF_CAN;
  _can_address.can_ifindex = _ifr.ifr_ifindex;

  if (::bind(_socket, (struct sockaddr *)&_can_address, sizeof(_can_address)) < 0)
  {
    ROS_ERROR("CanInterface: can't bind the socket. --> return.");
    return false;
  }

  // All fine.
  _callback_received_msg = callback;

  return true;
}

void CanInterface::start(void)
{
  if (!this->isInitialized())
  {
    ROS_ERROR("CanInterface: can't start. Interface is not initialized. --> return;");
    return;
  }

  _thread_receive_run = true;
  _thread_receive = std::thread(&CanInterface::receiveCan, this);
}

void CanInterface::stop(void)
{
  if (!_thread_receive_run)
    return;

  _thread_receive_run = false;
  _thread_receive.join();
}

void CanInterface::receiveCan(void)
{
  constexpr int sleepTime = 20; // 20 ms

  while (_thread_receive_run)
  {
    can_frame frame;

    const int nbytes = ::read(_socket, &frame, sizeof(can_frame));

    if (nbytes < 0)
    {
      ROS_ERROR("CanInterface: can't read from can interface.");
      continue;
    }

    if (nbytes < sizeof(can_frame))
    {
      ROS_ERROR("CanInterface: read incompleted can frame.");
      continue;
    }

    // Call callback with the received can frame.
    _callback_received_msg(CanMsg(frame));

    ::usleep(sleepTime * 1000); // Use ROS Time and sleep only the rest.
    // I think it is not necessary, because if the read function blocks then we haven't a high cpi
    // load. And then we should read what we can.
  }
}

void CanInterface::send(const CanMsg& msg)
{
  if (!this->isInitialized())
  {
    ROS_ERROR("CanInterface: can't send. Interface is not initialized. --> return.");
    return;
  }

  can_frame frame = msg.toCanFrame();

  if (::write(_socket, &frame, sizeof(can_frame)) < 0)
    ROS_ERROR("CanInterface: sent bytes != frame.bytes. Msh id = %x." , frame.can_id);
}

} // end namespace motor_controller

} // end namespace francor


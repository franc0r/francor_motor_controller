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
#include <ros/ros.h>
#include <geometry_msgs/TwistStamped.h>
#include <std_msgs/String.h>

#include "francor_motor_controller/CanInterface.h"
#include "firmware_mc_can/MotorcontrollerMsg.h"

//  #include "francor_motor_controller/motor_controller_node.h"

francor::motor_controller::CanInterface can;

class RotationSpeed
{
public:

  enum class Mode : std::uint8_t {
    NORMAL,
    BOGIE_UP_DRIVING,
    BOGIE_UP_STANDING,
  };

  RotationSpeed(void) = default;
  ~RotationSpeed(void) = default;

  void calculate(const geometry_msgs::Twist& msg)
  {
    _left  = -msg.angular.z;
    _right =  msg.angular.z;

    _left  += msg.linear.x;
    _right += msg.linear.x;

    this->cut(_left);
    this->cut(_right);
  }

  double speedLeft(void) const noexcept { return _left; }
  double speedRight(void) const noexcept { return _right; }
  double speedLeftMiddle(void) const noexcept
  {
    if (this->disableMiddle())
      return 0.0;

    switch (_mode)
    {
    case Mode::NORMAL:
      return _left;

    case Mode::BOGIE_UP_DRIVING:
      return _left + _plus_bogie_up_driving; // Maximum is more than 1.0!

    case Mode::BOGIE_UP_STANDING:
      return _left + _speed_bogie_up_standing;

    default:
      return 0.0;
    }
  }
  double speedRightMiddle(void) const noexcept
  {
    if (this->disableMiddle())
      return 0.0;

    switch (_mode)
    {
    case Mode::NORMAL:
      return _right;

    case Mode::BOGIE_UP_DRIVING:
      return _right + _plus_bogie_up_driving; // Maximum is more than 1.0!

    case Mode::BOGIE_UP_STANDING:
      return _right + _speed_bogie_up_standing;

    default:
      return 0.0;
    }
  }
  static void setPlusBogieUpDriving(const double value) { _plus_bogie_up_driving = value; }
  static void setSpeedBogieUp(const double value) { _speed_bogie_up_standing = value; }
  void setMode(const Mode mode) { _mode = mode; }

private:
  void cut(double& value)
  {
    if (value > 1.0)
      value = 1.0;

    if (value < -1.0)
      value = - 1.0;
  }
  bool disableMiddle(void) const noexcept
  {
    return (_left > 0.0 && _right < 0.0) || (_left < 0.0 && _right > 0.0);
  }

  double _left = 0.0;
  double _right = 0.0;
  Mode _mode = Mode::NORMAL;

  static double _plus_bogie_up_driving;
  static double _speed_bogie_up_standing;
};

double RotationSpeed::_plus_bogie_up_driving = 0.1;
double RotationSpeed::_speed_bogie_up_standing = 0.2;

RotationSpeed rs;

void sendVelocity(const geometry_msgs::Twist& msg)
{
  francor::MotorcontrolMsg commandLeft;
  francor::MotorcontrolMsg commandRight;

  rs.calculate(msg);

  commandLeft._power = static_cast<std::int16_t>(rs.speedLeft() * 255.0);
  commandLeft._stop = (rs.speedLeft() == 0.0 ? 1 : 0);

  commandRight._power = static_cast<std::int16_t>(rs.speedRight() * 255.0);
  commandRight._stop = (rs.speedRight() == 0.0 ? 1 : 0);

  const bool disableMiddle = (commandLeft._power > 0.0 && commandRight._power < 0.0) ||
                             (commandLeft._power < 0.0 && commandRight._power > 0.0);

  // Left Side
  can.send(francor::motor_controller::CanMsg(0x11, 8, reinterpret_cast<char*>(commandLeft._raw_data)));
  std::this_thread::sleep_for(std::chrono::milliseconds(10));

  if (disableMiddle)
  {
    francor::MotorcontrolMsg command;

    command._power = 0.0;
    command._stop = 0;

    can.send(francor::motor_controller::CanMsg(0x21, 8, reinterpret_cast<char*>(command._raw_data)));
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  else
  {
    can.send(francor::motor_controller::CanMsg(0x21, 8, reinterpret_cast<char*>(commandLeft._raw_data)));
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  can.send(francor::motor_controller::CanMsg(0x31, 8, reinterpret_cast<char*>(commandLeft._raw_data)));
  std::this_thread::sleep_for(std::chrono::milliseconds(10));


  // Right Side
  can.send(francor::motor_controller::CanMsg(0x12, 8, reinterpret_cast<char*>(commandRight._raw_data)));
  std::this_thread::sleep_for(std::chrono::milliseconds(10));

  if (disableMiddle)
  {
    francor::MotorcontrolMsg command;

    command._power = 0.0;
    command._stop = 0;

    can.send(francor::motor_controller::CanMsg(0x22, 8, reinterpret_cast<char*>(command._raw_data)));
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  else
  {
    can.send(francor::motor_controller::CanMsg(0x22, 8, reinterpret_cast<char*>(commandRight._raw_data)));
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  can.send(francor::motor_controller::CanMsg(0x32, 8, reinterpret_cast<char*>(commandRight._raw_data)));
  std::this_thread::sleep_for(std::chrono::milliseconds(10));
}

void callbackTwistStamped(const geometry_msgs::TwistStamped& msg)
{
  sendVelocity(msg.twist);
}

void callbackTwist(const geometry_msgs::Twist& msg)
{
  sendVelocity(msg);
}

void callbackDriveMode(const std_msgs::String& msg)
{
  if (msg.data == "NONE")
    rs.setMode(RotationSpeed::Mode::NORMAL);
  else if (msg.data == "BOGIE_UP")
    rs.setMode(RotationSpeed::Mode::BOGIE_UP_STANDING);
  else if (msg.data == "BOGIE_UP_DRIVE")
    rs.setMode(RotationSpeed::Mode::BOGIE_UP_DRIVING);
  else
    ROS_ERROR("Motorcontroller: drive mode it not supported.");
}

void callbackReceiveCan(const francor::motor_controller::CanMsg& msg)
{
  std::cout << msg << std::endl;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "motor_controller_node");

  ros::NodeHandle privateNh("~");
  double plusBogieUp = 0.0;
  double speedBogieUp = 0.0;

  privateNh.param<double>("plus_bogie_up", plusBogieUp, plusBogieUp);
  privateNh.param<double>("speed_bogie_up", speedBogieUp, speedBogieUp);

  RotationSpeed::setPlusBogieUpDriving(plusBogieUp);
  RotationSpeed::setSpeedBogieUp(speedBogieUp);

  ros::NodeHandle nh;
  ros::Subscriber subTwist(nh.subscribe("/morty/twist", 2, callbackTwist));
  ros::Subscriber subTwistStamped(nh.subscribe("/morty/twist_stamped", 2, callbackTwistStamped));
  ros::Subscriber subMode(nh.subscribe("drive/action", 2, callbackDriveMode));

  can.initialize("can0", callbackReceiveCan);
  can.start();

  ros::spin();

  can.stop();

  return 0;
}

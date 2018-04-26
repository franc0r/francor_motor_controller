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
    CLIMP,
    BOGIE_UP_BOOSTED,
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
  double speedLeftFront(void) const noexcept
  {
    switch (_mode)
    {
    case Mode::NORMAL:
      return _left;

    case Mode::BOGIE_UP_DRIVING:
      return _left;

    case Mode::BOGIE_UP_STANDING:
      return _left;

    case Mode::CLIMP:
    case Mode::BOGIE_UP_BOOSTED:
      return _left + _climp_boost;

    default:
      ROS_ERROR("MotorController: unsportet mode.");
      return 0.0;
    }
  }
  double speedRightFront(void) const noexcept
  {
    switch (_mode)
    {
    case Mode::NORMAL:
      return _right;

    case Mode::BOGIE_UP_DRIVING:
      return _right;

    case Mode::BOGIE_UP_STANDING:
      return _right;

    case Mode::CLIMP:
    case Mode::BOGIE_UP_BOOSTED:
      return _right + _climp_boost;

    default:
      ROS_ERROR("MotorController: unsportet mode.");
      return 0.0;
    }
  }
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
    case Mode::BOGIE_UP_BOOSTED:
      return _left + _speed_bogie_up_standing;

    case Mode::CLIMP:
      return _left;

    default:
      ROS_ERROR("MotorController: unsportet mode.");
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
    case Mode::BOGIE_UP_BOOSTED:
      return _right + _speed_bogie_up_standing;

    case Mode::CLIMP:
      return _right;

    default:
      ROS_ERROR("MotorController: unsportet mode.");
      return 0.0;
    }
  }
  double speedLeftRear(void) const noexcept
  {
    switch (_mode)
    {
    case Mode::NORMAL:
      return _left;

    case Mode::BOGIE_UP_DRIVING:
      return _left;

    case Mode::BOGIE_UP_STANDING:
    case Mode::BOGIE_UP_BOOSTED:
      return  -_speed_bogie_up_standing;

    case Mode::CLIMP:
      return _left;

    default:
      ROS_ERROR("MotorController: unsportet mode.");
      return 0.0;
    }
  }
  double speedRightRear(void) const noexcept
  {
    switch (_mode)
    {
    case Mode::NORMAL:
      return _right;

    case Mode::BOGIE_UP_DRIVING:
      return _right;

    case Mode::BOGIE_UP_STANDING:
    case Mode::BOGIE_UP_BOOSTED:
      return -_speed_bogie_up_standing;

    case Mode::CLIMP:
      return _right;

    default:
      ROS_ERROR("MotorController: unsportet mode.");
      return 0.0;
    }
  }

  bool disableMiddle(void) const noexcept
  {
    return (_left > 0.0 && _right < 0.0) || (_left < 0.0 && _right > 0.0);
  }

  static void setPlusBogieUpDriving(const double value) { _plus_bogie_up_driving = value; }
  static void setSpeedBogieUp(const double value) { _speed_bogie_up_standing = value; }
  static void setClimpBoost(const double value) { _climp_boost = value; }
  void setMode(const Mode mode) { _mode = mode; }

private:
  void cut(double& value)
  {
    if (value > 1.0)
      value = 1.0;

    if (value < -1.0)
      value = - 1.0;
  }


  double _left = 0.0;
  double _right = 0.0;
  Mode _mode = Mode::NORMAL;

  static double _plus_bogie_up_driving;
  static double _speed_bogie_up_standing;
  static double _climp_boost;
};

double RotationSpeed::_plus_bogie_up_driving = 0.1;
double RotationSpeed::_speed_bogie_up_standing = 0.2;
double RotationSpeed::_climp_boost = 0.3;

RotationSpeed rs;

void sendVelocity(const geometry_msgs::Twist& msg)
{
  francor::MotorcontrolMsg commandLeft;
  francor::MotorcontrolMsg commandRight;

  rs.calculate(msg);


  commandLeft._power = static_cast<std::int16_t>(rs.speedLeftFront() * 255.0);
  commandLeft._brake = (rs.speedLeftFront() == 0.0 ? 1 : 0);

  commandRight._power = static_cast<std::int16_t>(rs.speedRightFront() * 255.0);
  commandRight._brake = (rs.speedRightFront() == 0.0 ? 1 : 0);

  const bool disableMiddle = rs.disableMiddle();

  // Left Side
  can.send(francor::motor_controller::CanMsg(0x11, 8, reinterpret_cast<char*>(commandLeft._raw_data)));
  std::this_thread::sleep_for(std::chrono::milliseconds(10));

  if (disableMiddle)
  {
    francor::MotorcontrolMsg command;

    command._power = 0.0;
    command._brake = 0;

    can.send(francor::motor_controller::CanMsg(0x21, 8, reinterpret_cast<char*>(command._raw_data)));
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  else
  {
    francor::MotorcontrolMsg command;

    command._power = static_cast<std::int16_t>(rs.speedLeftMiddle() * 255.0);
    command._brake = 0;

    ROS_INFO("MotorController: set speed middle to %f.", rs.speedLeftMiddle());
    can.send(francor::motor_controller::CanMsg(0x21, 8, reinterpret_cast<char*>(command._raw_data)));
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  commandLeft._brake = (rs.speedLeftRear() == 0.0 ? 1 : 0);
  commandLeft._power = static_cast<std::int16_t>(rs.speedLeftRear() * 255.0);
  can.send(francor::motor_controller::CanMsg(0x31, 8, reinterpret_cast<char*>(commandLeft._raw_data)));
  std::this_thread::sleep_for(std::chrono::milliseconds(10));


  // Right Side
  can.send(francor::motor_controller::CanMsg(0x12, 8, reinterpret_cast<char*>(commandRight._raw_data)));
  std::this_thread::sleep_for(std::chrono::milliseconds(10));

  if (disableMiddle)
  {
    francor::MotorcontrolMsg command;

    command._power = 0.0;
    command._brake = 0;

    can.send(francor::motor_controller::CanMsg(0x22, 8, reinterpret_cast<char*>(command._raw_data)));
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  else
  {
    francor::MotorcontrolMsg command;

    command._power = static_cast<std::int16_t>(rs.speedRightMiddle() * 255.0);
    command._brake = 0;

    ROS_INFO("MotorController: set speed middle to %f.", rs.speedRightMiddle());
    can.send(francor::motor_controller::CanMsg(0x22, 8, reinterpret_cast<char*>(command._raw_data)));
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  commandRight._power = static_cast<std::int16_t>(rs.speedRightRear() * 255.0);
  commandRight._brake = (rs.speedRightRear() == 0.0 ? 1 : 0);
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
  {
    rs.setMode(RotationSpeed::Mode::NORMAL);
    ROS_INFO("MotorController: set mode normal.");
  }
  else if (msg.data == "BOGIE_UP")
  {
    rs.setMode(RotationSpeed::Mode::BOGIE_UP_STANDING);
    ROS_INFO("MotorController: set mode bogie up standing.");
  }
  else if (msg.data == "BOGIE_UP_DRIVE")
  {
    rs.setMode(RotationSpeed::Mode::BOGIE_UP_DRIVING);
    ROS_INFO("MotorController: set mode bogie up driving.");
  }
  else if (msg.data == "CLIMP")
  {
    rs.setMode(RotationSpeed::Mode::CLIMP);
    ROS_INFO("MotorController: set mode climp.");
  }
  else if (msg.data == "BOGIE_UP_BOOSTED")
  {
    rs.setMode(RotationSpeed::Mode::BOGIE_UP_BOOSTED);
    ROS_INFO("MotorController: set mode bogie up boosted.");
  }
  else
  {
    ROS_ERROR("Motorcontroller: drive mode it not supported.");
  }
}

void callbackReceiveCan(const francor::motor_controller::CanMsg& msg)
{
  std::cout << msg << std::endl;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "motor_controller_node");

  ros::NodeHandle privateNh("~");
  double plusBogieUp = 0.1;
  double speedBogieUp = 0.4;
  double climpBoost = 0.3;

  privateNh.param<double>("plus_bogie_up", plusBogieUp, plusBogieUp);
  privateNh.param<double>("speed_bogie_up", speedBogieUp, speedBogieUp);
  privateNh.param<double>("climp_boost", climpBoost, climpBoost);

  RotationSpeed::setPlusBogieUpDriving(plusBogieUp);
  RotationSpeed::setSpeedBogieUp(speedBogieUp);
  RotationSpeed::setClimpBoost(climpBoost);

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

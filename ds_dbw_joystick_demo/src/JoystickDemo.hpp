/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2020-2021, Dataspeed Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Dataspeed Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/
#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <std_msgs/msg/empty.hpp>

#include <ds_dbw_msgs/msg/brake_cmd.hpp>
#include <ds_dbw_msgs/msg/gear_cmd.hpp>
#include <ds_dbw_msgs/msg/steering_cmd.hpp>
#include <ds_dbw_msgs/msg/throttle_cmd.hpp>
#include <ds_dbw_msgs/msg/misc_cmd.hpp>

namespace ds_dbw_joystick_demo {

class JoystickDemo : public rclcpp::Node {
public:
  JoystickDemo(const rclcpp::NodeOptions& options);

private:
  void recvJoy(const sensor_msgs::msg::Joy::ConstSharedPtr msg);
  void cmdCallback();

  // Topics
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr sub_joy_;
  rclcpp::Publisher<ds_dbw_msgs::msg::BrakeCmd>::SharedPtr pub_brake_;
  rclcpp::Publisher<ds_dbw_msgs::msg::ThrottleCmd>::SharedPtr pub_throttle_;
  rclcpp::Publisher<ds_dbw_msgs::msg::SteeringCmd>::SharedPtr pub_steering_;
  rclcpp::Publisher<ds_dbw_msgs::msg::GearCmd>::SharedPtr pub_gear_;
  rclcpp::Publisher<ds_dbw_msgs::msg::MiscCmd>::SharedPtr pub_misc_;
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr pub_enable_;
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr pub_disable_;

  // Parameters
  bool brake_ = true;    // Send brake commands
  bool throttle_ = true; // Send throttle commands
  bool steer_ = true;    // Send steering commands
  bool shift_ = true;    // Send shift commands
  bool misc_ = true;     // Send misc commands

  // Parameters
  float brake_gain_ = 1;    // Adjust brake value
  float throttle_gain_ = 1; // Adjust throttle value

  // Parameters
  bool ignore_ = false; // Ignore driver overrides
  bool enable_ = true;  // Use enable and disable buttons
  bool strq_ = false;   // Steering torque command (otherwise angle)
  float svel_ = 0;      // Steering velocity limit (deg/s)
  float sacl_ = 0;      // Steering acceleration limit (deg/s^2)

  // Variables
  struct JoystickDataStruct {
    rclcpp::Time stamp;
    float brake_joy = 0;
    float throttle_joy = 0;
    float steering_joy = 0;
    uint8_t gear_cmd = 0;
    uint8_t turn_signal_cmd = 0;
    uint8_t door_select = 0;
    uint8_t door_action = 0;
    bool steering_mult = false;
    bool steering_cal = false;
    bool joy_throttle_valid = false;
    bool joy_brake_valid = false;
  } data_;
  rclcpp::TimerBase::SharedPtr timer_;
  sensor_msgs::msg::Joy joy_;
  float last_steering_filt_output_ = 0;

  enum {
    BTN_PARK = 3,
    BTN_REVERSE = 1,
    BTN_NEUTRAL = 2,
    BTN_DRIVE = 0,
    BTN_ENABLE = 5,
    BTN_DISABLE = 4,
    BTN_STEER_MULT_1 = 6,
    BTN_STEER_MULT_2 = 7,
    BTN_TRUNK_OPEN = 9,
    BTN_TRUNK_CLOSE = 10,
    BTN_COUNT_X = 11,
    BTN_COUNT_D = 12,
    AXIS_THROTTLE = 5,
    AXIS_BRAKE = 2,
    AXIS_STEER_1 = 0,
    AXIS_STEER_2 = 3,
    AXIS_TURN_SIG = 6,
    AXIS_DOOR_SELECT = 6,
    AXIS_DOOR_ACTION = 7,
    AXIS_COUNT_D = 6,
    AXIS_COUNT_X = 8,
  };
};

} // namespace ds_dbw_joystick_demo

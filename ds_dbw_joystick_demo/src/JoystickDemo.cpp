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

#include "JoystickDemo.hpp"

#include <algorithm> // std::clamp()

namespace ds_dbw_joystick_demo {

JoystickDemo::JoystickDemo(const rclcpp::NodeOptions &options) : rclcpp::Node("joy_demo", options) {
  joy_.axes.resize(AXIS_COUNT_X, 0);
  joy_.buttons.resize(BTN_COUNT_X, 0);

  brake_ = declare_parameter<bool>("brake", brake_);
  throttle_ = declare_parameter<bool>("throttle", throttle_);
  steer_ = declare_parameter<bool>("steer", steer_);
  shift_ = declare_parameter<bool>("shift", shift_);
  misc_ = declare_parameter<bool>("misc", misc_);
  brake_gain_ = std::clamp<float>(declare_parameter<float>("brake_gain", brake_gain_), 0, 1);
  throttle_gain_ = std::clamp<float>(declare_parameter<float>("throttle_gain", throttle_gain_), 0, 1);

  ignore_ = declare_parameter<bool>("ignore", ignore_);
  enable_ = declare_parameter<bool>("enable", enable_);
  strq_ = declare_parameter<bool>("strq", strq_);
  svel_ = declare_parameter<float>("svel", svel_);
  sacl_ = declare_parameter<float>("sacl", sacl_);

  using std::placeholders::_1;
  sub_joy_ = create_subscription<sensor_msgs::msg::Joy>("/joy", 1, std::bind(&JoystickDemo::recvJoy, this, _1));

  data_.brake_joy = 0.0;
  data_.gear_cmd = ds_dbw_msgs::msg::Gear::NONE;
  data_.steering_joy = 0.0;
  data_.steering_mult = false;
  data_.steering_cal = false;
  data_.throttle_joy = 0.0;
  data_.joy_throttle_valid = false;
  data_.joy_brake_valid = false;

  if (brake_) {
    pub_brake_ = create_publisher<ds_dbw_msgs::msg::BrakeCmd>("brake_cmd", 1);
  }
  if (throttle_) {
    pub_throttle_ = create_publisher<ds_dbw_msgs::msg::ThrottleCmd>("throttle_cmd", 1);
  }
  if (steer_) {
    pub_steering_ = create_publisher<ds_dbw_msgs::msg::SteeringCmd>("steering_cmd", 1);
  }
  if (shift_) {
    pub_gear_ = create_publisher<ds_dbw_msgs::msg::GearCmd>("gear_cmd", 1);
  }
  if (misc_) {
    pub_misc_ = create_publisher<ds_dbw_msgs::msg::MiscCmd>("misc_cmd", 1);
  }
  if (enable_) {
    pub_enable_ = create_publisher<std_msgs::msg::Empty>("enable", 1);
    pub_disable_ = create_publisher<std_msgs::msg::Empty>("disable", 1);
  }

  // Initilize timestamp to be old (timeout)
  data_.stamp = now() - std::chrono::seconds(1);
  timer_ = create_wall_timer(std::chrono::milliseconds(20), std::bind(&JoystickDemo::cmdCallback, this));
}

void JoystickDemo::cmdCallback() {
  // Detect joy timeouts and reset
  if (now() - data_.stamp > std::chrono::milliseconds(100)) {
    data_.joy_throttle_valid = false;
    data_.joy_brake_valid = false;
    last_steering_filt_output_ = 0.0;
    return;
  }

  // Brake
  if (brake_) {
    ds_dbw_msgs::msg::BrakeCmd msg;
    msg.enable = true;
    msg.ignore = ignore_;
    #if 0
    msg.cmd_type = ds_dbw_msgs::msg::BrakeCmd::CMD_PRESSURE;
    msg.cmd = data_.brake_joy * brake_gain_ * 100.1; // 0-100 bar
    #elif 0
    msg.cmd_type = ds_dbw_msgs::msg::BrakeCmd::CMD_TORQUE;
    msg.cmd = data_.brake_joy * brake_gain_ * 10.1e3f; // 0-10k Nm
    #elif 0
    msg.cmd_type = ds_dbw_msgs::msg::BrakeCmd::CMD_ACCEL;
    msg.cmd = data_.brake_joy * brake_gain_ * -10.2f + 0.1f; // +0.1-10 m/s^2
    #elif 0
    msg.cmd_type = ds_dbw_msgs::msg::BrakeCmd::CMD_ACCEL_ACC;
    msg.cmd = data_.brake_joy * brake_gain_ * -5.2f + 0.1f; // +0.1-5 m/s^2
    #elif 0
    msg.cmd_type = ds_dbw_msgs::msg::BrakeCmd::CMD_ACCEL_AEB;
    msg.cmd = data_.brake_joy * brake_gain_ * -10.2f + 0.1f; // +0.1-10 m/s^2
    #else
    msg.cmd_type = ds_dbw_msgs::msg::BrakeCmd::CMD_PERCENT;
    msg.cmd = data_.brake_joy * brake_gain_ * 100.1; // 0-100%
    #endif
    #if 0
    msg.rate_inc = INFINITY;
    msg.rate_dec = INFINITY;
    #endif
    pub_brake_->publish(msg);
  }

  // Throttle
  if (throttle_) {
    ds_dbw_msgs::msg::ThrottleCmd msg;
    msg.enable = true;
    msg.ignore = ignore_;
    #if 0
    msg.cmd_type = ds_dbw_msgs::msg::ThrottleCmd::CMD_PEDAL_RAW;
    msg.cmd = data_.throttle_joy * throttle_gain_ * 100.1f; // 0-100%
    #else
    msg.cmd_type = ds_dbw_msgs::msg::ThrottleCmd::CMD_PERCENT;
    msg.cmd = data_.throttle_joy * throttle_gain_ * 100.1f; // 0-100%
    #endif
    #if 0
    msg.rate_inc = INFINITY;
    msg.rate_dec = INFINITY;
    #endif
    pub_throttle_->publish(msg);
  }

  // Steering
  if (steer_) {
    ds_dbw_msgs::msg::SteeringCmd msg;
    msg.enable = true;
    msg.ignore = ignore_;
    if (data_.steering_cal) {
      msg.cmd_type = ds_dbw_msgs::msg::SteeringCmd::CMD_CALIBRATE;
      msg.cmd = 0.0;
    } else if (!strq_) {
      // Scale and filter angle command
      float raw_steering_cmd = data_.steering_joy;
      if (!data_.steering_mult) {
        raw_steering_cmd *= 0.5;
      }
      constexpr float TAU = 0.1;
      float filtered_steering_cmd = 0.02f / TAU * raw_steering_cmd + (1.0f - 0.02f / TAU) * last_steering_filt_output_;
      last_steering_filt_output_ = filtered_steering_cmd;

      #if 0
      msg.cmd_type = ds_dbw_msgs::msg::SteeringCmd::CMD_ANGLE;
      msg.cmd = filtered_steering_cmd * (float)(M_PI / 180) * 500.1; // +-500 deg
      #elif 0
      msg.cmd_type = ds_dbw_msgs::msg::SteeringCmd::CMD_CURVATURE;
      msg.cmd = filtered_steering_cmd * 0.201f; // +-0.2/m
      #elif 0
      msg.cmd_type = ds_dbw_msgs::msg::SteeringCmd::CMD_YAW_RATE;
      msg.cmd = filtered_steering_cmd * 5.01; // +-5 rad/s
      #else
      msg.cmd_type = ds_dbw_msgs::msg::SteeringCmd::CMD_PERCENT;
      msg.cmd = filtered_steering_cmd * 100.1f; // +-100%
      #endif
      msg.cmd_rate  = svel_ * (float)(M_PI / 180);
      msg.cmd_accel = sacl_ * (float)(M_PI / 180);
    } else {
      msg.cmd_type = ds_dbw_msgs::msg::SteeringCmd::CMD_TORQUE;
      msg.cmd = data_.steering_joy * 12; // 12 Nm
    }
    pub_steering_->publish(msg);
  }

  // Gear
  if (shift_) {
    if (data_.gear_cmd != ds_dbw_msgs::msg::Gear::NONE) {
      ds_dbw_msgs::msg::GearCmd msg;
      msg.cmd.value = data_.gear_cmd;
      pub_gear_->publish(msg);
    }
  }

  // Turn signal
  if (misc_) {
    ds_dbw_msgs::msg::MiscCmd msg;
    msg.turn_signal.value = data_.turn_signal_cmd;
    // msg.parking_brake.value = 0;
    // msg.door.select = data_.door_select;
    // msg.door.action = data_.door_action;
    pub_misc_->publish(msg);
  }
}

void JoystickDemo::recvJoy(const sensor_msgs::msg::Joy::ConstSharedPtr msg) {
  // Check for expected sizes
  if (msg->axes.size() != (size_t)AXIS_COUNT_X && msg->buttons.size() != (size_t)BTN_COUNT_X) {
    if (msg->axes.size() == (size_t)AXIS_COUNT_D && msg->buttons.size() == (size_t)BTN_COUNT_D) {
      RCLCPP_ERROR_THROTTLE(
          get_logger(), *get_clock(), 2e3,
          "Detected Logitech Gamepad F310 in DirectInput (D) mode. Please select (X) with the switch on "
          "the back to select XInput mode.");
    }
    if (msg->axes.size() != (size_t)AXIS_COUNT_X) {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2e3, "Expected %zu joy axis count, received %zu",
                            (size_t)AXIS_COUNT_X, msg->axes.size());
    }
    if (msg->buttons.size() != (size_t)BTN_COUNT_X) {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2e3, "Expected %zu joy button count, received %zu",
                            (size_t)BTN_COUNT_X, msg->buttons.size());
    }
    return;
  }

  // Handle joystick startup
  if (msg->axes[AXIS_THROTTLE] != 0.0) {
    data_.joy_throttle_valid = true;
  }
  if (msg->axes[AXIS_BRAKE] != 0.0) {
    data_.joy_brake_valid = true;
  }

  // Throttle
  if (data_.joy_throttle_valid) {
    data_.throttle_joy = 0.5 - 0.5 * msg->axes[AXIS_THROTTLE];
  }

  // Brake
  if (data_.joy_brake_valid) {
    data_.brake_joy = 0.5 - 0.5 * msg->axes[AXIS_BRAKE];
  }

  // Gear
  if (msg->buttons[BTN_PARK]) {
    data_.gear_cmd = ds_dbw_msgs::msg::Gear::PARK;
  } else if (msg->buttons[BTN_REVERSE]) {
    data_.gear_cmd = ds_dbw_msgs::msg::Gear::REVERSE;
  } else if (msg->buttons[BTN_DRIVE]) {
    data_.gear_cmd = ds_dbw_msgs::msg::Gear::DRIVE;
  } else if (msg->buttons[BTN_NEUTRAL]) {
    data_.gear_cmd = ds_dbw_msgs::msg::Gear::NEUTRAL;
  } else {
    data_.gear_cmd = ds_dbw_msgs::msg::Gear::NONE;
  }

  // Steering
  if(std::abs(msg->axes[AXIS_STEER_1]) > std::abs(msg->axes[AXIS_STEER_2])) {
    data_.steering_joy = msg->axes[AXIS_STEER_1];
  } else {
    data_.steering_joy = msg->axes[AXIS_STEER_2];
  }
  data_.steering_mult = msg->buttons[BTN_STEER_MULT_1] || msg->buttons[BTN_STEER_MULT_2];
  data_.steering_cal = msg->buttons[BTN_STEER_MULT_1] && msg->buttons[BTN_STEER_MULT_2];

  // Turn signal
  if (msg->axes[AXIS_TURN_SIG] != joy_.axes[AXIS_TURN_SIG]) {
    if (std::abs(msg->axes[AXIS_DOOR_ACTION]) < 0.5) {
      switch (data_.turn_signal_cmd) {
        case ds_dbw_msgs::msg::TurnSignal::NONE:
          if (msg->axes[AXIS_TURN_SIG] < -0.5) {
            data_.turn_signal_cmd = ds_dbw_msgs::msg::TurnSignal::RIGHT;
          } else if (msg->axes[AXIS_TURN_SIG] > 0.5) {
            data_.turn_signal_cmd = ds_dbw_msgs::msg::TurnSignal::LEFT;
          }
          break;
        case ds_dbw_msgs::msg::TurnSignal::LEFT:
          if (msg->axes[AXIS_TURN_SIG] < -0.5) {
            data_.turn_signal_cmd = ds_dbw_msgs::msg::TurnSignal::RIGHT;
          } else if (msg->axes[AXIS_TURN_SIG] > 0.5) {
            data_.turn_signal_cmd = ds_dbw_msgs::msg::TurnSignal::NONE;
          }
          break;
        case ds_dbw_msgs::msg::TurnSignal::RIGHT:
          if (msg->axes[AXIS_TURN_SIG] < -0.5) {
            data_.turn_signal_cmd = ds_dbw_msgs::msg::TurnSignal::NONE;
          } else if (msg->axes[AXIS_TURN_SIG] > 0.5) {
            data_.turn_signal_cmd = ds_dbw_msgs::msg::TurnSignal::LEFT;
          }
          break;
      }
    }
  }

  #if 0
  // Doors and trunk
  data_.door_select = dbw_fca_msgs::msg::DoorCmd::NONE;
  data_.door_action = dbw_fca_msgs::msg::DoorCmd::NONE;
  if (msg->buttons[BTN_TRUNK_OPEN]) {
    data_.door_select = dbw_fca_msgs::msg::DoorCmd::TRUNK;
    data_.door_action = dbw_fca_msgs::msg::DoorCmd::OPEN;
  } else if (msg->buttons[BTN_TRUNK_CLOSE]) {
    data_.door_select = dbw_fca_msgs::msg::DoorCmd::TRUNK;
    data_.door_action = dbw_fca_msgs::msg::DoorCmd::CLOSE;
  }
  if (msg->axes[AXIS_DOOR_ACTION] > 0.5) {
    if (msg->axes[AXIS_DOOR_SELECT] < -0.5) {
      data_.door_select = dbw_fca_msgs::msg::DoorCmd::RIGHT;
      data_.door_action = dbw_fca_msgs::msg::DoorCmd::OPEN;
    } else if (msg->axes[AXIS_TURN_SIG] > 0.5) {
      data_.door_select = dbw_fca_msgs::msg::DoorCmd::LEFT;
      data_.door_action = dbw_fca_msgs::msg::DoorCmd::OPEN;
    }
  }
  if (msg->axes[AXIS_DOOR_ACTION] < -0.5) {
    if (msg->axes[AXIS_DOOR_SELECT] < -0.5) {
      data_.door_select = dbw_fca_msgs::msg::DoorCmd::RIGHT;
      data_.door_action = dbw_fca_msgs::msg::DoorCmd::CLOSE;
    } else if (msg->axes[AXIS_TURN_SIG] > 0.5) {
      data_.door_select = dbw_fca_msgs::msg::DoorCmd::LEFT;
      data_.door_action = dbw_fca_msgs::msg::DoorCmd::CLOSE;
    }
  }
  #endif

  // Optional enable and disable buttons
  if (enable_) {
    if (msg->buttons[BTN_ENABLE]) {
      pub_enable_->publish(std_msgs::msg::Empty());
    }
    if (msg->buttons[BTN_DISABLE]) {
      pub_disable_->publish(std_msgs::msg::Empty());
    }
  }

  data_.stamp = now();
  joy_ = *msg;
}

} // namespace ds_dbw_joystick_demo

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(ds_dbw_joystick_demo::JoystickDemo)

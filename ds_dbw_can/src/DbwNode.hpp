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

// ROS messages
#include <can_msgs/msg/frame.hpp>
#include <dataspeed_can_msg_filters/ApproximateTime.hpp>
#include <ds_dbw_msgs/msg/steering_cmd.hpp>
#include <ds_dbw_msgs/msg/steering_diagnostics.hpp>
#include <ds_dbw_msgs/msg/steering_report.hpp>
#include <ds_dbw_msgs/msg/brake_cmd.hpp>
#include <ds_dbw_msgs/msg/brake_diagnostics.hpp>
#include <ds_dbw_msgs/msg/brake_report.hpp>
#include <ds_dbw_msgs/msg/throttle_cmd.hpp>
#include <ds_dbw_msgs/msg/throttle_diagnostics.hpp>
#include <ds_dbw_msgs/msg/throttle_report.hpp>
#include <ds_dbw_msgs/msg/gear_cmd.hpp>
#include <ds_dbw_msgs/msg/gear_diagnostics.hpp>
#include <ds_dbw_msgs/msg/gear_report.hpp>
#include <ds_dbw_msgs/msg/system_report.hpp>
#include <ds_dbw_msgs/msg/vehicle_velocity.hpp>
#include <ds_dbw_msgs/msg/throttle_info.hpp>
#include <ds_dbw_msgs/msg/brake_info.hpp>
#include <ds_dbw_msgs/msg/ulc_cmd.hpp>
#include <ds_dbw_msgs/msg/ulc_report.hpp>
#include <ds_dbw_msgs/msg/wheel_speeds.hpp>
#include <ds_dbw_msgs/msg/wheel_positions.hpp>
#include <ds_dbw_msgs/msg/misc_cmd.hpp>
#include <ds_dbw_msgs/msg/misc_report.hpp>
#include <ds_dbw_msgs/msg/tire_pressures.hpp>
#include <ds_dbw_msgs/msg/ecu_info.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/empty.hpp>

// The following messages are deprecated
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>

// CAN messages
#include <ds_dbw_can/dispatch.hpp>

// Platform and module version map
#include <ds_dbw_can/PlatformMap.hpp>

namespace ds_dbw_can {

class DbwNode : public rclcpp::Node {
public:
  DbwNode(const rclcpp::NodeOptions& options);

private:
  void timerCallback();
  void recvEnable(const std_msgs::msg::Empty::ConstSharedPtr);
  void recvDisable(const std_msgs::msg::Empty::ConstSharedPtr);
  void recvCAN(const can_msgs::msg::Frame::ConstSharedPtr);
  void recvCanImu(const std::vector<can_msgs::msg::Frame::ConstSharedPtr>& msgs);
  void recvCanMisc(const std::vector<can_msgs::msg::Frame::ConstSharedPtr>& msgs);
  void recvSteeringCmd(const ds_dbw_msgs::msg::SteeringCmd::ConstSharedPtr msg);
  void recvBrakeCmd(const ds_dbw_msgs::msg::BrakeCmd::ConstSharedPtr msg);
  void recvThrottleCmd(const ds_dbw_msgs::msg::ThrottleCmd::ConstSharedPtr msg);
  void recvGearCmd(const ds_dbw_msgs::msg::GearCmd::ConstSharedPtr msg);
  void recvMiscCmd(const ds_dbw_msgs::msg::MiscCmd::ConstSharedPtr msg);
  void recvUlcCmd(const ds_dbw_msgs::msg::UlcCmd::ConstSharedPtr msg);
  void recvSteeringCalibrate(const std_msgs::msg::Empty::ConstSharedPtr msg);

  // CAN messages
  MsgSteerCmdUsr     msg_steer_cmd_;
  MsgSteerReport1    msg_steer_rpt_1_;
  MsgSteerReport2    msg_steer_rpt_2_;
  MsgSteerReport3    msg_steer_rpt_3_;
  MsgBrakeCmdUsr     msg_brake_cmd_;
  MsgBrakeReport1    msg_brake_rpt_1_;
  MsgBrakeReport2    msg_brake_rpt_2_;
  MsgBrakeReport3    msg_brake_rpt_3_;
  MsgThrtlCmdUsr     msg_thrtl_cmd_;
  MsgThrtlReport1    msg_thrtl_rpt_1_;
  MsgThrtlReport2    msg_thrtl_rpt_2_;
  MsgThrtlReport3    msg_thrtl_rpt_3_;
  MsgGearCmdUsr      msg_gear_cmd_;
  MsgGearReport1     msg_gear_rpt_1_;
  MsgGearReport2     msg_gear_rpt_2_;
  MsgGearReport3     msg_gear_rpt_3_;
  MsgSystemReport    msg_system_rpt_;
  MsgSystemCmd       msg_system_cmd_;
  MsgVehicleVelocity msg_veh_vel_;
  MsgThrtlInfo       msg_thrtl_info_;
  MsgBrakeInfo       msg_brake_info_;
  MsgUlcCmd          msg_ulc_cmd_;
  MsgUlcCfg          msg_ulc_cfg_;
  MsgUlcReport       msg_ulc_rpt_;
  MsgAccel           msg_accel_;
  MsgGyro            msg_gyro_;
  MsgWheelSpeed      msg_wheel_speed_;
  MsgWheelPosition   msg_wheel_position_;
  MsgMiscCmd         msg_misc_cmd_;
  MsgMiscReport1     msg_misc_rpt_1_;
  MsgMiscReport2     msg_misc_rpt_2_;
  MsgTirePressure    msg_tire_pressure_;

  //
  static constexpr const char * WARN_CMD_TXT = "Another node on the CAN bus is commanding the vehicle!!! Subsystem: %s, Id: 0x%03X";

  // Rate limited commands
  std_msgs::msg::Header::_stamp_type msg_ulc_cfg_stamp_;

  rclcpp::TimerBase::SharedPtr timer_;
  bool prev_enable_ = true;
  bool enable_ = false;
  bool fault() const {
    return (msg_steer_rpt_1_.fault && !msg_steer_rpt_1_.timeout)
        || (msg_brake_rpt_1_.fault && !msg_brake_rpt_1_.timeout)
        || (msg_thrtl_rpt_1_.fault && !msg_thrtl_rpt_1_.timeout)
        || (msg_gear_rpt_1_.fault);
  }
  bool overrideActive() const {
    return (msg_steer_rpt_1_.override_active && !msg_steer_rpt_1_.timeout)
        || (msg_brake_rpt_1_.override_active && !msg_brake_rpt_1_.timeout)
        || (msg_thrtl_rpt_1_.override_active && !msg_thrtl_rpt_1_.timeout)
        || (msg_gear_rpt_1_.override_active);
  }
  bool overrideOther() const {
    return (msg_steer_rpt_1_.override_other && !msg_steer_rpt_1_.timeout)
        || (msg_brake_rpt_1_.override_other && !msg_brake_rpt_1_.timeout)
        || (msg_thrtl_rpt_1_.override_other && !msg_thrtl_rpt_1_.timeout)
        || (msg_gear_rpt_1_.override_other);
  }
  bool overrideLatched() const {
    return (msg_steer_rpt_1_.override_latched && !msg_steer_rpt_1_.timeout)
        || (msg_brake_rpt_1_.override_latched && !msg_brake_rpt_1_.timeout)
        || (msg_thrtl_rpt_1_.override_latched && !msg_thrtl_rpt_1_.timeout);
  }
  bool override() const {
    return overrideActive()
        || overrideOther()
        || overrideLatched();
  }
  bool enabled() const {
    return enable_ && !fault() && !override();
  }
  bool publishDbwEnabled(bool force = false);
  void enableSystem();
  void disableSystem();
  void buttonCancel();

  uint8_t gear_reject_ = 0;
  void warnRejectGear(uint8_t reject);
  void warnBadCrcRc(bool bad_crc, bool bad_rc, const char *name);

  template <typename Cmd, typename Rpt>
  class WarnTimeout {
  public:
    WarnTimeout(const rclcpp::Node &node, const char *name) : node_(node), name_(name) {}
    void recv(const Rpt &msg) {
      if (enabled_ && !msg.enabled
      && !timeout_ &&  msg.timeout) {
        RCLCPP_WARN(node_.get_logger(), "%s subsystem disabled after %zums command timeout", name_, Cmd::TIMEOUT_MS);
      }
      enabled_ = msg.enabled;
      timeout_ = msg.timeout;
    }
  private:
    const rclcpp::Node &node_;
    const char * const name_;
    bool enabled_ = false;
    bool timeout_ = false;
  };
  WarnTimeout<MsgSteerCmd, MsgSteerReport1> warn_timeout_steer_ = WarnTimeout<MsgSteerCmd, MsgSteerReport1>(*this, "Steering");
  WarnTimeout<MsgBrakeCmd, MsgBrakeReport1> warn_timeout_brake_ = WarnTimeout<MsgBrakeCmd, MsgBrakeReport1>(*this, "Brake");
  WarnTimeout<MsgThrtlCmd, MsgThrtlReport1> warn_timeout_thrtl_ = WarnTimeout<MsgThrtlCmd, MsgThrtlReport1>(*this, "Throttle");

#if 0
  template <typename T>
  class Recv {
  public:
    using Stamp = std_msgs::msg::Header::_stamp_type;
    bool valid() const {
      // Calculate difference
      int64_t diff_ns = (rclcpp::Time(stamp) - rclcpp::Time(stamp_)).nanoseconds();
      return diff_ns > TIMEOUT_NS;
    }
  private:
    static constexpr int64_t TIMEOUT_NS = T::TIMEOUT_MS * 1000000;
    Stamp stamp_;
  };
  Recv<MsgBrakeReport1>    asdf_;
  #endif

  // Rolling counter validation for received messages
  template <typename T>
  class RollingCounterValidation {
  public:
    using Stamp = std_msgs::msg::Header::_stamp_type;
    bool valid(const T& msg, Stamp stamp) {
      // Calculate difference
      int64_t diff_ns = (rclcpp::Time(stamp) - rclcpp::Time(stamp_)).nanoseconds();
      bool valid_rc = msg.validRc(rc_);
      // Save inputs for next time
      rc_ = msg.rc;
      stamp_ = stamp;
      // Valid if rolling counter is valid or enough time has passed to accept any value
      return valid_rc || (diff_ns > TIMEOUT_NS);
    }
  private:
    static constexpr int64_t TIMEOUT_NS = T::TIMEOUT_MS * 1000000;
    uint8_t rc_ = 0;
    Stamp stamp_;
  };
  RollingCounterValidation<MsgSteerReport1>    msg_steer_rpt_1_rc_;
  RollingCounterValidation<MsgSteerReport2>    msg_steer_rpt_2_rc_;
  RollingCounterValidation<MsgBrakeReport1>    msg_brake_rpt_1_rc_;
  RollingCounterValidation<MsgBrakeReport2>    msg_brake_rpt_2_rc_;
  RollingCounterValidation<MsgThrtlReport1>    msg_thrtl_rpt_1_rc_;
  RollingCounterValidation<MsgThrtlReport2>    msg_thrtl_rpt_2_rc_;
  RollingCounterValidation<MsgGearReport1>     msg_gear_rpt_1_rc_;
  RollingCounterValidation<MsgGearReport2>     msg_gear_rpt_2_rc_;
  RollingCounterValidation<MsgSystemReport>    msg_system_rpt_rc_;
  RollingCounterValidation<MsgVehicleVelocity> msg_veh_vel_rc_;
  RollingCounterValidation<MsgThrtlInfo>       msg_thrtl_info_rc_;
  RollingCounterValidation<MsgBrakeInfo>       msg_brake_info_rc_;
  RollingCounterValidation<MsgUlcReport>       msg_ulc_rpt_rc_;
  RollingCounterValidation<MsgAccel>           msg_accel_rc_;
  RollingCounterValidation<MsgGyro>            msg_gyro_rc_;
  RollingCounterValidation<MsgMiscReport1>     msg_misc_rpt_1_rc_;
  RollingCounterValidation<MsgMiscReport2>     msg_misc_rpt_2_rc_;

#if 0
  template <typename T>
  class ReportRecv {
  public:
    bool timeout = false;
    bool fault = false;
    bool override_active = false;
    bool override_other = false;
    bool override_latched = false;
    bool recv(const T &msg) {
      timeout = msg.timeout;
      fault = msg.fault;
      override_active = msg.override_active;
      override_other = msg.override_other;
      override_latched = msg.override_latched;
    }
    bool override(bool ignore_timeout) const {
      if (!ignore_timeout || !timeout) {
        return override_active || override_other || override_latched;
      }
      return false;
    }
    bool fault(bool ignore_timeout) const {
      return !ignore_timeout && fault;
      if (!ignore_timeout || !timeout) {
        return fault;
      }
      return false;
    }
  };
  #endif

  // ECU Info
  std::map<uint16_t, ds_dbw_msgs::msg::EcuInfo> ecu_info_;
  std::map<uint8_t, uint32_t> cfg_hash_;
  std::map<uint8_t, std::array<uint8_t,6>> mac_;
  std::map<uint8_t, std::string> ldate_; // License date
  std::map<uint8_t, std::string> ldate_recv_;
  std::map<uint8_t, std::string> bdate_; // Build date
  std::map<uint8_t, std::string> bdate_recv_;
  std::string vin_;
  std::string vin_recv_;

  // Firmware Versions
  ds_dbw_can::PlatformMap firmware_;

  // Frame ID
  std::string frame_id_ = "base_footprint";

  // Command warnings
  bool warn_crc_ = true;
  bool warn_cmds_ = true;
  bool warn_unknown_ = true;

  // Subscribed topics
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr sub_enable_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr sub_disable_;
  rclcpp::Subscription<can_msgs::msg::Frame>::SharedPtr sub_can_;
  rclcpp::Subscription<ds_dbw_msgs::msg::SteeringCmd>::SharedPtr sub_steer_;
  rclcpp::Subscription<ds_dbw_msgs::msg::BrakeCmd>::SharedPtr sub_brake_;
  rclcpp::Subscription<ds_dbw_msgs::msg::ThrottleCmd>::SharedPtr sub_thrtl_;
  rclcpp::Subscription<ds_dbw_msgs::msg::GearCmd>::SharedPtr sub_gear_;
  rclcpp::Subscription<ds_dbw_msgs::msg::MiscCmd>::SharedPtr sub_misc_;
  rclcpp::Subscription<ds_dbw_msgs::msg::UlcCmd>::SharedPtr sub_ulc_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr sub_calibrate_steering_;

  // Published topics
  rclcpp::Publisher<can_msgs::msg::Frame>::SharedPtr pub_can_;
  rclcpp::Publisher<ds_dbw_msgs::msg::SteeringReport>::SharedPtr pub_steer_rpt_;
  rclcpp::Publisher<ds_dbw_msgs::msg::SteeringDiagnostics>::SharedPtr pub_steer_diag_;
  rclcpp::Publisher<ds_dbw_msgs::msg::BrakeReport>::SharedPtr pub_brake_rpt_;
  rclcpp::Publisher<ds_dbw_msgs::msg::BrakeDiagnostics>::SharedPtr pub_brake_diag_;
  rclcpp::Publisher<ds_dbw_msgs::msg::ThrottleReport>::SharedPtr pub_thrtl_rpt_;
  rclcpp::Publisher<ds_dbw_msgs::msg::ThrottleDiagnostics>::SharedPtr pub_thrtl_diag_;
  rclcpp::Publisher<ds_dbw_msgs::msg::GearReport>::SharedPtr pub_gear_rpt_;
  rclcpp::Publisher<ds_dbw_msgs::msg::GearDiagnostics>::SharedPtr pub_gear_diag_;
  rclcpp::Publisher<ds_dbw_msgs::msg::SystemReport>::SharedPtr pub_system_rpt_;
  rclcpp::Publisher<ds_dbw_msgs::msg::VehicleVelocity>::SharedPtr pub_veh_vel_;
  rclcpp::Publisher<ds_dbw_msgs::msg::ThrottleInfo>::SharedPtr pub_thrtl_info_;
  rclcpp::Publisher<ds_dbw_msgs::msg::BrakeInfo>::SharedPtr pub_brake_info_;
  rclcpp::Publisher<ds_dbw_msgs::msg::UlcReport>::SharedPtr pub_ulc_;
  rclcpp::Publisher<ds_dbw_msgs::msg::WheelSpeeds>::SharedPtr pub_wheel_speeds_;
  rclcpp::Publisher<ds_dbw_msgs::msg::WheelPositions>::SharedPtr pub_wheel_positions_;
  rclcpp::Publisher<ds_dbw_msgs::msg::MiscReport>::SharedPtr pub_misc_;
  rclcpp::Publisher<ds_dbw_msgs::msg::TirePressures>::SharedPtr pub_tire_pressures_;
  rclcpp::Publisher<ds_dbw_msgs::msg::EcuInfo>::SharedPtr pub_ecu_info_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub_imu_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_vin_;      // Deprecated message
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_sys_enable_; // Deprecated message

  // Time Synchronization
  dataspeed_can_msg_filters::ApproximateTime sync_imu_;
  dataspeed_can_msg_filters::ApproximateTime sync_misc_;
  bool debug_sync_ = false;
  void printSyncDelta(const can_msgs::msg::Frame::ConstSharedPtr &msg0, const can_msgs::msg::Frame::ConstSharedPtr &msg1, const char *name) const {
    if (debug_sync_) {
      const auto &stamp0 = msg0->header.stamp;
      const auto &stamp1 = msg1->header.stamp;
      RCLCPP_INFO(get_logger(), "Time: %u.%09u, %u.%09u, delta: %0.1fms for %s",
                  stamp0.sec, stamp0.nanosec, stamp1.sec, stamp1.nanosec,
                  std::abs((rclcpp::Time(stamp0) - rclcpp::Time(stamp1)).nanoseconds()) / 1e6, name);
    }
  }
};

} // namespace ds_dbw_can

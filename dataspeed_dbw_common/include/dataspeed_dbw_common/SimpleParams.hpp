/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2015-2021, Dataspeed Inc.
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
#include "rclcpp/rclcpp.hpp"

namespace dataspeed_dbw_common {

#define SP_LOG_NAME "simple_params"

/**
 * @brief Class designed to simplify interaction with the ROS2 parameter system.
 *
 * @details This classs is designed to simplify itneraction with the ROS2
 * parameter system. By default in ROS2, a parameter can be undeclared, but set.
 * However it must be declared to be accessed by a node. Once a node declares a
 * parameter, it has one of several states, initialized and set, uninitialized
 * and unset, or initialized and unset. Which state it is depends on several
 * factors. With the default node options, there is no way to detect anything
 * other than declared outside of a try/catch block. This class is designed to
 * have symantics closer to that of the original ROS, while still having the ROS
 * 2 capabilities.
 *
 * @version 1.1
 */
class SimpleParams {
 public:
  SimpleParams(rclcpp::Node *node) : node_(node) {
#ifdef DEBUG
    rcutils_logging_set_logger_level(SP_LOG_NAME, RCUTILS_LOG_SEVERITY_DEBUG);
#endif
  }

  /**
   * @brief Removes any declared, but unset parameters.
   *
   * @return true if any parameters were undeclared, false otherwise.
   */
  bool cleanup() {
    auto params = node_->list_parameters({}, 0);
    std::vector<std::string> to_remove;
    for (auto &name : params.names) {
      try {
        auto param = node_->get_parameter(name);
        if (param.get_type() == rclcpp::PARAMETER_NOT_SET) {
          to_remove.push_back(name);
        }
      } catch (...) {
        // declared, but unset
        to_remove.push_back(name);
      }
    }
    for (auto &name : to_remove) {
      if (node_->has_parameter(name)) {
        node_->undeclare_parameter(name);
      }
    }
    return to_remove.size() > 0;
  }

  /**
   * @brief Retrieves the value of the result. Defining it if needed.
   * @tparam T C++ type of the result.
   * @param[in] name the parameter name
   * @param[out] result the result output
   * @return true if the parameter value was retrieved as the templated type,
   * false otherwise.
   */
  template <typename T>
  bool get(const std::string &name, T &result) {
    auto param = get_or_create_parameter(name);
    try {
      result = param.get_value<T>();
      RCUTILS_LOG_DEBUG_NAMED(SP_LOG_NAME, "Got parameter: '%s' Type: %s",
                              name.c_str(), param.get_type_name().c_str());
      return true;
    } catch (...) {
      RCUTILS_LOG_DEBUG_NAMED(SP_LOG_NAME,
                              "Failed to get parameter: '%s' Type: %s",
                              name.c_str(), param.get_type_name().c_str());
    }
    return false;
  }

  /**
   * @brief Retrieves the value of the result, or returns an alternative value.
   * Defining it if needed.
   *
   * @tparam T C++ type of the result.
   * @param[in] name the parameter name
   * @param[out] result the result output
   * @param[in] def the value to use if the parameter is not set
   * @return true if the parameter value was retrieved as the templated type,
   * false if the alternative value was used.
   */
  template <typename T>
  bool get(const std::string &name, T &result, const T &def) {
    if (get(name, result)) {
      return true;
    }
    result = def;
    set(name, result);
    return false;
  }

  /**
   * @brief Retrieves the value of the result, or returns an alternative value.
   * Defining it if needed.
   *
   * @tparam T C++ type of the result.
   * @param[in] name the parameter name
   * @param[out] result the result output
   * @param[in] min the minimum value of the parameter
   * @param[in] max the maximum value of the parameter
   * @return true if the parameter value was retrieved as the templated type,
   * false if the alternative value was used.
   */
  template <typename T>
  bool get(const std::string &name, T &result, const T &min, const T &max) {
    if (get(name, result)) {
      if (result < min) {
        result = min;
        set(name, result);
      } else if (result > max) {
        result = max;
        set(name, result);
      }
      return true;
    }
    return false;
  }

  /**
   * @brief Retrieves the value of the result as a long value, regardless of the
   * type of the original parameter. Integers are retrurned directly, floats and
   * doubles are cast to an integer and strings are parsed (including octal and
   * hexidecimal values).
   *
   * @param[in] name the parameter name
   * @param[out] result the result output
   * @return true if the parameter value was retrieved as a long, false
   * otherwise.
   */
  bool get_long(const std::string &name, int64_t &result) {
    auto param = get_or_create_parameter(name);
    const auto type = param.get_type();
    if (type == rclcpp::PARAMETER_INTEGER) {
      RCUTILS_LOG_DEBUG_NAMED(SP_LOG_NAME, "Got parameter: '%s' from integer.",
                              name.c_str());
      result = param.as_int();
      return true;
    }
    if (type == rclcpp::PARAMETER_DOUBLE) {
      result = static_cast<int64_t>(param.as_double());
      RCUTILS_LOG_DEBUG_NAMED(SP_LOG_NAME, "Got parameter: '%s' from double.",
                              name.c_str());
      return true;
    }
    if (type == rclcpp::PARAMETER_STRING) {
      try {
        auto value = param.as_string();
        result = std::stol(value, nullptr, 0);
        RCUTILS_LOG_DEBUG_NAMED(SP_LOG_NAME, "Got parameter: '%s' from string.",
                                name.c_str());
        return true;
      } catch (...) {
      }
    }
    RCUTILS_LOG_DEBUG_NAMED(SP_LOG_NAME,
                            "Failed to get long parameter: '%s' Type: %s",
                            name.c_str(), param.get_type_name().c_str());
    return false;
  }

  /**
   * @brief Retrieves the value of the result as a long value, regardless of the
   * type of the original parameter. Returns an alternative value if it failed.
   * Integers are retrurned directly, floats and doubles are cast to an integer
   * and strings are parsed (including octal and hexidecimal values).
   *
   * @param[in] name the parameter name
   * @param[out] result the result output
   * @param[in] def the default parameter
   * @return true if the parameter value was retrieved as a long, false
   * otherwise.
   */
  bool get_long(const std::string &name, int64_t &result, const int64_t &def) {
    if (get_long(name, result)) {
      return true;
    }
    result = def;
    set(name, result);
    return false;
  }

  /**
   * @brief Determines if the given parameter is declared.
   *
   * @param name the name of the parameter
   * @return true if it has been declared, false otherwise.
   */
  bool is_declared(const std::string &name) const {
    return node_->has_parameter(name);
  }

  /**
   * @brief Determines if the given parameter is declared and initalized.
   *
   * @param name the name of the parameter
   * @return true if has been declared and initialized, false otherwise.
   */
  bool is_initalized(const std::string &name) const {
    if (node_->has_parameter(name)) {
      try {
        node_->get_parameter(name);
        return true;
      } catch (...) {
      }
    }
    return false;
  }

  /**
   * @brief Determines if the given parameter is declared, initilized and set.
   *
   * @param name the name of the parameter
   * @return true if is declared, initialized, and set, false otherwise.
   */
  bool is_set(const std::string &name) const {
    if (node_->has_parameter(name)) {
      try {
        auto param = node_->get_parameter(name);
        return param.get_type() != rclcpp::PARAMETER_NOT_SET;
      } catch (...) {
      }
    }
    return false;
  }

  /**
   * @brief Sets the given parameter name to the given value, declaring if
   * needed.
   *
   * @tparam T value type
   * @param[in] name the name of the parameter
   * @param[in] value the value to set
   */
  template <typename T>
  void set(const std::string &name, const T &value) {
    if (!node_->has_parameter(name)) {
      node_->declare_parameter(name, rclcpp::ParameterValue());
    }
    rclcpp::Parameter param(name, value);
    node_->set_parameter(param);
  }

 private:
  rclcpp::Parameter get_or_create_parameter(const std::string &name) {
    if (!node_->has_parameter(name)) {
      node_->declare_parameter(name, rclcpp::ParameterValue());
    }
    try {
      return node_->get_parameter(name);
    } catch (...) {
      // declared, but unset
    }
    // set it to undefined
    rclcpp::Parameter parameter(name);
    node_->set_parameter(parameter);
    return parameter;
  }

  rclcpp::Node *node_;
};

}  // namespace dataspeed

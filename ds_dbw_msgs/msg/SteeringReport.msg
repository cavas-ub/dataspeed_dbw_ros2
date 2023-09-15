std_msgs/Header header

# Steering Wheel
float32 steering_wheel_angle  # rad
float32 steering_column_torque # Nm
float32 cmd # Changes with cmd_type
uint8 cmd_type # Options defined in SteeringCmd message

#
bool limiting_value
bool limiting_rate
bool external_control

# Status
bool ready    # Ready to be enabled
bool enabled  # Enabled
bool override_active  # Active override
bool override_other   # Override from other system
bool override_latched # Latched override
bool timeout  # Command timeout (of valid messages)
bool fault # Fault of any kind
bool bad_crc # CRC error on most recent command
bool bad_rc # Repeated rolling counter on most recent command

##### Start of slower secondary reports #####

# Rate limits
float32 limit_rate
float32 limit_value

# Command source
CmdSrc cmd_src

# Degraded
bool degraded
bool degraded_cmd_type
bool degraded_comms
bool degraded_comms_dbw
bool degraded_comms_dbw_gateway
bool degraded_comms_dbw_brake
bool degraded_comms_dbw_thrtl
bool degraded_comms_dbw_gear
bool degraded_comms_vehicle
bool degraded_comms_actuator
bool degraded_internal
bool degraded_control_performance
bool degraded_param_mismatch
bool degraded_vehicle
bool degraded_actuator
bool degraded_vehicle_speed
bool degraded_calibration

# Faults
bool fault_power
bool fault_comms
bool fault_comms_dbw
bool fault_comms_dbw_gateway
bool fault_comms_dbw_brake
bool fault_comms_dbw_thrtl
bool fault_comms_dbw_gear
bool fault_comms_vehicle
bool fault_comms_actuator
bool fault_internal
bool fault_vehicle
bool fault_vehicle_speed
bool fault_actuator
bool fault_actuator_torque_sensor
bool fault_actuator_config
bool fault_calibration
